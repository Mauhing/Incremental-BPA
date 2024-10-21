/**
 * @file Mesher.cpp
 * @brief defines methods for building a surface mesh from points stored in an 
 * octree these methods are declared in Mesher.h
 * @author Julie Digne julie.digne@liris.cnrs.fr
 * @date 2012/10/17
 * @copyright This file implements an algorithm possibly linked to the patent 
 * US6968299B1.
 * This file is made available for the exclusive aim of serving as
 * scientific tool to verify the soundness and completeness of the
 * algorithm description. Compilation, execution and redistribution
 * of this file may violate patents rights in certain countries.
 * The situation being different for every country and changing
 * over time, it is your responsibility to determine which patent
 * rights restrictions apply to you before you compile, use,
 * modify, or redistribute this file. A patent lawyer is qualified
 * to make this determination.
 * If and only if they don't conflict with any patent terms, you
 * can benefit from the following license terms attached to this
 * file.
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published 
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 */
#include "Mesher.h"
#include "OctreeIterator.h"
#include <cstdlib>
#include <cmath>
#include <cassert>
#ifndef USE_CLANG
    #include <omp.h>
#endif
#include <sstream>
#include <memory>
#include <algorithm>

const double PI = 3.1415926535;

using namespace std;

Mesher::Mesher()
{
    m_octree_vertices = NULL;
    m_iterator_vertices = NULL;
    m_nfacets = 0;
    m_nvertices = 0;
    m_num_ball_centers = 0;
    m_recycle_vertices_idx = std::unordered_set<unsigned int>();
    m_octree_ball_centers = NULL;
    m_octree_ball_centers_iterator = NULL;
}

Mesher::Mesher(OctreeVertices* octree, OctreeIteratorVertices* iterator,
               OctreePoints* octree_ball_centers, OctreeIteratorPoints* octree_ball_centers_iterator)
{
    m_octree_vertices = octree;
    m_iterator_vertices = iterator;
    m_ball_radius = iterator->getR();
    m_sq_ball_radius = m_ball_radius * m_ball_radius;
    m_nfacets = 0;
    m_nvertices = 0;
    m_num_ball_centers = 0;
    m_recycle_vertices_idx = std::unordered_set<unsigned int>();
    m_octree_ball_centers = octree_ball_centers;
    m_octree_ball_centers_iterator = octree_ball_centers_iterator;
}

Mesher::~Mesher()
{
    m_octree_vertices = NULL;
    m_iterator_vertices = NULL;
    m_edge_front.clear();
    m_border_edges.clear();

    Facet_star_list::iterator fi;
    for(fi = m_facets.begin(); fi != m_facets.end(); ++fi)
    {
        delete *fi;
        *fi = NULL;
    }

    m_facets.clear();
    m_vertices.clear();
    m_nfacets = 0;
    m_nvertices = 0;
}



void Mesher::setBallRadius(double r)
{
    m_ball_radius = r;
    m_sq_ball_radius = r*r;
}

double Mesher::getBallRadius() const
{
    return m_ball_radius;
}

double Mesher::getSquareBallRadius() const
{
    return m_sq_ball_radius;
}

unsigned int Mesher::nVertices() const
{
    return m_nvertices;
}


unsigned int Mesher::nFacets() const
{
    return m_nfacets;
}

unsigned int Mesher::nFrontEdges() const
{
    return (unsigned int) m_edge_front.size();
}

unsigned int Mesher::nBorderEdges() const
{
    return (unsigned int) m_border_edges.size();
}

unsigned int Mesher::getNumBallCenters() const
{
    return m_num_ball_centers;
}

const Point_UnOrdSet& Mesher::getBallCenters() const
{
    return m_ball_centers;
}

void Mesher::reconstruct()
{
    std::cout<<"***********Ball radius "<<m_ball_radius
             <<" ***********"<<std::endl;

    if(m_edge_front.empty())
    {
        std::cout<<"No front edge found, looking for seed triangle."
                 <<std::endl;
        bool ok = findSeedTriangle();
        if(!ok)
            std::cout<<"No seed triangle found, no triangulation done!"
            <<std::endl;
    }
    else
    {
        // If there are only one radius. This scope will never be executed.
        expandTriangulation();
    }
}


void Mesher::reconstruct(const std::list<double>& radii)
{
    std::cout << "single threaded reconstruction" << std::endl;
    for (double radius : radii)
    {
        changeRadius(radius);
        reconstruct();
    }
}

void Mesher::furtherReconstruct()
{
    Edge_star_list::iterator ei = m_border_edges.begin();
    while(ei != m_border_edges.end())
    {
        Edge *e = *ei;
        e->setType(Edge::FRONT);
        m_edge_front.push_back(e);
        ei = m_border_edges.erase(ei);
    } 


    unsigned int depth = m_octree_vertices->getDepth();
    m_iterator_vertices->setDepth(depth);
    
    reconstruct();
}


void Mesher::changeRadius(double radius)
{
    setBallRadius(radius);
    Edge_star_list::iterator ei = m_border_edges.begin();
    while(ei != m_border_edges.end())
    {
        Edge *e = *ei;
        Facet *f = e->getFacet1();

        Point center;
        if(emptyBallConfiguration(f->vertex(0), f->vertex(1),
            f->vertex(2),center))
        {
            e->setType(Edge::FRONT);
            m_edge_front.push_back(e);
            ei = m_border_edges.erase(ei);
            continue;
        }
        ++ei;
    }
    std::cout<<"After changing radius, "<<m_edge_front.size()
             <<" front edges and "<<m_border_edges.size()
             <<" border edges."<<std::endl;
}


void Mesher::resetOctree(OctreeVertices *octree, OctreeIteratorVertices *iterator)
{
    m_octree_vertices = octree;
    m_iterator_vertices = iterator;
}


bool Mesher::findSeedTriangle()
{
    bool found = false;
    OctreeNodeV *node = m_octree_vertices->getRoot();
    findSeedTriangle(node, found);
    return found;
}

void Mesher::findSeedTriangle(OctreeNodeV* node, bool &found)
{
    if( node->getDepth() != 0)
    {
        for(unsigned int i = 0; i<8; i++)
        {
            if(node->getChild(i) != NULL)
                findSeedTriangle(node->getChild(i), found);
        }
    }
    else if( node->getNpts() != 0)
    {
        //Vertex_list::iterator pi = node->points_begin();
        typename Vertex_UnOrdSet::iterator pi = node->points_begin();
        while( pi != node->points_end())
        {
            // current approach is find seed and expand
            // then find a seed again and expand.
            Vertex* v = *pi;
            if(v->getType() == Vertex::ORPHAN) //0
            {
                if(trySeed(*v))
                {
                    found = true;
                    expandTriangulation();
                }
            }
            ++pi;
        }
    }
}



bool Mesher::trySeed(Vertex& v)
{
    // First, get the neighbors of the current vertex v.
    // ni are the next vertices in the sorted neighbor list
    // nj are the second next vertices in the sorted neighbor list
    // than sliding over the sorted neighbor list until we find a valid seed triangle

    Neighbor_star_map neighbors;
    m_iterator_vertices->setR(2.0 * m_ball_radius);
    m_iterator_vertices->getSortedNeighbors(v, neighbors);
    m_iterator_vertices->setR(m_ball_radius);

    if(neighbors.size()<3)
        return false;

    Neighbor_iterator ni = neighbors.begin();
    while(ni != neighbors.end())
    {
        Vertex &vtest = *(ni->second); // The "second" is the vertex pointer stored in the neighbor star map
        if( (vtest.getType() != Vertex::ORPHAN) || (&vtest == &v) ) //0
        {
            ++ni;
            continue;
        }

        Neighbor_iterator nj = ni;
        ++nj;

        Vertex *candidate = NULL;
        Point center;
        
        bool changeHandness = false;
        while(nj != neighbors.end())
        {
            if(tryTriangleSeed(&v, &vtest, nj->second, neighbors, center, changeHandness))
            {
                candidate = nj->second;
                break;
            }
            ++nj;
        }

        if(candidate != NULL)
        {
            // <<<
            // To check any of the edges are not a front edge, we can use the getLinkingEdge method
            Edge *e1 = v.getLinkingEdge(candidate);
            Edge *e2 = vtest.getLinkingEdge(candidate);
            Edge *e3 = v.getLinkingEdge(&vtest);

            if( ((e1!=NULL)&&(e1->getType()!=Edge::FRONT))
                ||((e2!=NULL)&&(e2->getType()!=Edge::FRONT))
                ||((e3!=NULL)&&(e3->getType()!=Edge::FRONT)) )
            {
                ++ni;
                continue;
            }
            // >>>
            // continue tommorrow with adding the flipIndex part.
            Vertex *v0 = &v;
            Vertex *v1 = &vtest;
            Vertex *v2 = candidate;
            if(changeHandness)
            {
                Vertex temp = *v1;
                *v1 = *v2;
                *v2 = temp;
            } 
            // <<<
            // Now, the seed triangle will take account of the handness.
            //Facet *facet = new Facet(&v, &vtest, candidate, center);
            Facet *facet = new Facet(v0, v1, v2, center);
            // >>>
            addFacet(facet);

            if(m_nfacets % 10000 == 0)
            std::cout<<m_nvertices<<" vertices. "<<m_nfacets<<" facets. "
            <<m_edge_front.size()<<" front edges. "
            <<m_border_edges.size()<<" border edges."<<std::endl;

            e1 = v.getLinkingEdge(candidate);
            e2 = vtest.getLinkingEdge(candidate);
            e3 = v.getLinkingEdge(&vtest);

            //std::cout << "e1 start vertex: " << e1->getSource()->index() << std::endl;
            //std::cout << "e2 start vertex: " << e2->getSource()->index() << std::endl;
            //std::cout << "e3 start vertex: " << e3->getSource()->index() << std::endl; 
            //std::cout << "================================" << std::endl; 

            if(e1->getType() == Edge::FRONT)
                m_edge_front.push_front(e1);
            if(e2->getType() == Edge::FRONT)
                m_edge_front.push_front(e2);
            if(e3->getType() == Edge::FRONT)
                m_edge_front.push_front(e3);

            if(m_edge_front.size() > 0)
                return true;
        }
        ++ni;
    }
    return false;
}


bool Mesher::tryTriangleSeed(Vertex* v1, Vertex* v2, Vertex *v3, 
                             Neighbor_star_map &neighbors,
                             Point &center,
                             bool &changeHandness) const
{
    //if((v3->getType() != Vertex::ORPHAN) || ( !v3->isCompatibleWith(*v1, *v2)))
    //    return false;
    if((v3->getType() != Vertex::ORPHAN) || ( !v3->isCompatibleWithAndHandnessCheck(*v1, *v2, changeHandness)))
        return false;

    Edge *e1 = v1->getLinkingEdge(v3);
    Edge *e2 = v2->getLinkingEdge(v3);
    if(  ((e1!=NULL)&&(e1->getType()==Edge::INNER))
        || ((e2!=NULL)&&(e2->getType()==Edge::INNER)))
        return false;

    m_iterator_vertices->setR(m_ball_radius);
    if(! computeBallCenter(*v1, *v2, *v3, center))
        return false;

    Neighbor_iterator ni;
    for(ni = neighbors.begin(); ni != neighbors.end(); ++ni)
    {
        Vertex *v = ni->second;
        if((v == v1)||(v == v2)||(v == v3))
            continue;
        if( dist2(center,*v) < m_sq_ball_radius - 1e-16)
            return false;
    }
    return true;
}


bool Mesher::emptyBallConfiguration(Vertex* v1, Vertex* v2, Vertex* v3,
                                    Point & center) const
{
    m_iterator_vertices->setR(m_ball_radius);
    if(! computeBallCenter(*v1, *v2, *v3, center))
        return false;

    std::set<Vertex*> facet_vertices;

    facet_vertices.insert(v1);
    facet_vertices.insert(v2);
    facet_vertices.insert(v3);

    return m_iterator_vertices->containsOnly(center, facet_vertices);
}

bool Mesher::checkEmptyBallConfiguration(Vertex* v1, Vertex* v2, Vertex* v3,
                                         const Vertex_star_list &neighbors,
                                         const Point & center) const
{
   Vertex_star_list::const_iterator ni;
   for(ni = neighbors.begin() ; ni != neighbors.end(); ++ni)
   {
       const Vertex *v = *ni;
       if((v == v1)||(v == v2)||(v == v3))
           continue;
       if(dist2(*v,center)<m_sq_ball_radius - 1e-16)
       {
        //if(v->getType() == Vertex::INNER)//2
        // print v type
        //std::cout << "v type: " << v->getType() << std::endl;
            //std::cout << "v is inner" << std::endl;
           return false;
       }
   }
    return true;
}

bool Mesher::computeBallCenterUsingOrderOfVertices(const Vertex &v1, const Vertex &v2,
                               const Vertex &v3, Point &center) const
{
    //compute the circumcenter barycentric coordinates
    double c = dist2(v2, v1);
    double b = dist2(v1, v3);
    double a = dist2(v3, v2);
    double alpha = a *( b + c - a);
    double beta  = b *( a + c - b);
    double gamma = c *( a + b - c);
    double temp = alpha + beta + gamma;

    if(temp<1e-30)//aligned case
	    return false;


    alpha = alpha / temp;
    beta  =  beta / temp;
    gamma = gamma / temp;


    //computing the triangle circumcircle center
    double x = alpha * v1.x() + beta * v2.x() + gamma * v3.x();
    double y = alpha * v1.y() + beta * v2.y() + gamma * v3.y();
    double z = alpha * v1.z() + beta * v2.z() + gamma * v3.z();

    //computing the radius of the circumcircle
    double sq_circumradius = a * b * c;


    a = sqrt(a);
    b = sqrt(b);
    c = sqrt(c);

    sq_circumradius = sq_circumradius /
          ( (a + b + c) * (b + c - a) * (c + a - b) * (a + b - c) );

    //compute the ortogonal distance from the hypothetic center to the triangle
    double height = m_sq_ball_radius - sq_circumradius;

    //compute the normal of the three points
    double nx,ny,nz = 0;

    if(height >= 0.0)
    {
        computeNormal(v1, v2, v3, nx, ny, nz);
        height = sqrt(height);
        center = Point( x + height*nx, y + height*ny, z + height*nz);
        return true;
    }
    return false; 
}

bool Mesher::computeBallCenter(const Vertex &v1, const Vertex &v2,
                               const Vertex &v3, Point &center) const
{
    //compute the circumcenter barycentric coordinates
    double c = dist2(v2, v1);
    double b = dist2(v1, v3);
    double a = dist2(v3, v2);
    double alpha = a *( b + c - a);
    double beta  = b *( a + c - b);
    double gamma = c *( a + b - c);
    double temp = alpha + beta + gamma;

    if(temp<1e-30)//aligned case
	    return false;


    alpha = alpha / temp;
    beta  =  beta / temp;
    gamma = gamma / temp;


    //computing the triangle circumcircle center
    double x = alpha * v1.x() + beta * v2.x() + gamma * v3.x();
    double y = alpha * v1.y() + beta * v2.y() + gamma * v3.y();
    double z = alpha * v1.z() + beta * v2.z() + gamma * v3.z();

    //computing the radius of the circumcircle
    double sq_circumradius = a * b * c;


    a = sqrt(a);
    b = sqrt(b);
    c = sqrt(c);

    sq_circumradius = sq_circumradius /
          ( (a + b + c) * (b + c - a) * (c + a - b) * (a + b - c) );

    //compute the ortogonal distance from the hypothetic center to the triangle
    double height = m_sq_ball_radius - sq_circumradius;

    //compute the normal of the three points
    double nx,ny,nz = 0;

    if(height >= 0.0)
    {
        computeNormal(v1, v2, v3, nx, ny, nz);
        height = sqrt(height);
        center = Point( x + height*nx, y + height*ny, z + height*nz);
        return true;
    }
    return false;
}

// This function still use the normal of the input vertices.
void Mesher::computeNormal(const Vertex& v1, const Vertex& v2, const Vertex& v3,
                           double &nx, double &ny, double &nz) const
{
    cross_product( v2.x() - v1.x(), v2.y() - v1.y(), v2.z() - v1.z(),
		   v3.x() - v1.x(), v3.y() - v1.y(), v3.z() - v1.z(),
		   nx, ny, nz);
    normalize(nx,ny,nz);

    double mnx = v1.nx() + v2.nx() + v3.nx();
    double mny = v1.ny() + v2.ny() + v3.ny();
    double mnz = v1.nz() + v2.nz() + v3.nz();

    normalize(mnx,mny,mnz);

    if(nx*mnx + ny*mny + nz*mnz <0)
    {
      nx = -nx;
      ny = -ny;
      nz = -nz;
    }
}

void Mesher::computeNormalUsingOrderOfVertices(const Vertex& v1, const Vertex& v2, const Vertex& v3,
                           double &nx, double &ny, double &nz) const
{
    cross_product( v2.x() - v1.x(), v2.y() - v1.y(), v2.z() - v1.z(),
		   v3.x() - v1.x(), v3.y() - v1.y(), v3.z() - v1.z(),
		   nx, ny, nz);
    normalize(nx,ny,nz);
}

ReconstructionType Mesher::computeReconstructionType(
                                                    const Edge* eSource, 
                                                    const Edge* eTarget,
                                                    const Vertex* candidate) const
{
    // We can already assume the candidate is not INNER vertex.
    bool doesESourceExist = eSource != NULL;
    bool doesETargetExist = eTarget != NULL;
    bool isCandidateORPHAN = candidate->getType() == Vertex::ORPHAN;

    // If both e1 and e2 do not exist, then the reconstruction type is expansion.
    if (!doesESourceExist && !doesETargetExist && isCandidateORPHAN)
    {
        return ReconstructionType::EXPANSION;
    }

    // If both e1 and e2 exist, then the reconstruction type is glue.
    if (!doesESourceExist && !doesETargetExist && !isCandidateORPHAN)
    {
        return ReconstructionType::GLUE;
    }

    // If e1 or e2 is not a front edge, then no reconstruction is needed.
    bool isESourceNotFront = doesESourceExist && (eSource->getType() != Edge::FRONT);
    bool isETargetNotFront = doesETargetExist && (eTarget->getType() != Edge::FRONT);
    
    if (isESourceNotFront || isETargetNotFront)
    {
        return ReconstructionType::NO_RECONSTRUCTION;
    }

    bool isESourceFront = doesESourceExist && (eSource->getType() == Edge::FRONT);
    bool isETargetFront = doesETargetExist && (eTarget->getType() == Edge::FRONT);

    if (isESourceFront != isETargetFront)
    {
        if (isESourceFront)
            return ReconstructionType::EAR_FILLING_FrontEdge_SOURCE;
        else
            return ReconstructionType::EAR_FILLING_FrontEdge_TARGET;
    }

    if (isESourceFront == true && isETargetFront == true)
    {
        return ReconstructionType::HOLE_FILLING;
    }

    // If none of the above conditions are met, there are something wrong with the code.
    throw std::runtime_error("Unknown reconstruction type in computeReconstructionType");    
}

static bool isGoodOrentationWithSource(Edge *sideFrontEdge, Vertex *Source, Vertex *candidate)
{
    Facet *facet = sideFrontEdge->getFacet1(); // Since it is sideFrontEdge, there should only one facet.
    bool isGoodOrentation = false;
    
    // We expect the facet go from candidate to source
    // such that the twin we are creating will go from source to candidate.
    for(int i = 0; i < 3; i++)
    {
        Vertex *v0 = facet->vertex(i);
        Vertex *v1 = facet->vertex((i+1)%3);
        if(v0 == candidate && v1 == Source)
        {
            isGoodOrentation = true;
            break;
        }
    }
    return isGoodOrentation;
}

static bool isGoodOrentationWithTarget(Edge *sideFrontEdge, Vertex *Target, Vertex *candidate)
{
    Facet *facet = sideFrontEdge->getFacet1(); // Since it is sideFrontEdge, there should only one facet.
    bool isGoodOrentation = false;
    
    // We expect the facet go from target to candidate
    // such that the twin we are creating will go from candidate to target.
    for(int i = 0; i < 3; i++)
    {
        Vertex *v0 = facet->vertex(i);
        Vertex *v1 = facet->vertex((i+1)%3);
        if(v0 == Target && v1 == candidate)
        {
            isGoodOrentation = true;
            break;
        }
    }
    return isGoodOrentation;
}


void Mesher::expandTriangulation()
{
    while(! m_edge_front.empty() )
    {
        Edge *edge = m_edge_front.front();
        m_edge_front.pop_front();


        if(edge->getType() != Edge::FRONT)
            continue;

        Point center;
        Vertex *candidate = findCandidateVertex(edge, center);

        if((candidate == NULL) || (candidate->getType()==Vertex::INNER) //2
            ||(! candidate->isCompatibleWith(*edge)))
        {
            // If the candidate is type 2, which means it is an inner vertex.
            // In this case, we set the type of edge as BORDER.
            // No expansion is done.
            edge->setType(Edge::BORDER);
            m_border_edges.push_back(edge);
            continue;
        }

        Edge *eSource = candidate->getLinkingEdge(edge->getSource());
        Edge *eTarget = candidate->getLinkingEdge(edge->getTarget());

        ReconstructionType reconstructionType = computeReconstructionType(eSource, eTarget, candidate);

        if(reconstructionType == ReconstructionType::NO_RECONSTRUCTION)
        {
            edge->setType(Edge::BORDER);
            m_border_edges.push_back(edge);
            continue;
        }


        // <<< One more condition with good orentation.
        if(reconstructionType == ReconstructionType::EAR_FILLING_FrontEdge_SOURCE)
        {
            if(!isGoodOrentationWithSource(eSource, edge->getSource(), candidate))
            {
                std::cout << "Edge is not good orentation with source" << std::endl;
                edge->setType(Edge::BORDER);
                m_border_edges.push_back(edge);
                continue;
            }
        }
        if(reconstructionType == ReconstructionType::EAR_FILLING_FrontEdge_TARGET)
        {
            if(!isGoodOrentationWithTarget(eTarget, edge->getTarget(), candidate))
            {
                std::cout << "Edge is not good orentation with target" << std::endl;
                edge->setType(Edge::BORDER);
                m_border_edges.push_back(edge);
                continue;
            }
        }
        // >>>
        

        Facet * facet = new Facet(edge, candidate, center);
        addFacet(facet);

        Edge *e1 = candidate->getLinkingEdge(edge->getSource());
        Edge *e2 = candidate->getLinkingEdge(edge->getTarget());

        if(e1->getType() == Edge::FRONT)
            m_edge_front.push_front(e1);

        if(e2->getType() == Edge::FRONT)
            m_edge_front.push_front(e2);

        //if(m_nfacets % 10000 == 0)
        //    std::cout<<m_nvertices<<" vertices. "<<m_nfacets<<" facets. "
        //    <<m_edge_front.size()<<" front edges. "
        //    <<m_border_edges.size()<<" border edges."<<std::endl;
    }
}

Vertex* Mesher::findCandidateVertex(Edge *edge, Point &candidate_ball_center)
{
    Vertex *src = edge->getSource();
    Vertex *tgt = edge->getTarget();

    Point mp = midpoint(*src, *tgt);
    Vertex_star_list neighbors;

    double d = m_ball_radius + sqrt( m_sq_ball_radius - dist2(mp, *src) );
    m_iterator_vertices->setR(d);
    m_iterator_vertices->getNeighbors(mp,neighbors);
    m_iterator_vertices->setR(m_ball_radius);

    Facet *facet = edge->getFacet1(); // Get the first facet.
    const Point &center = facet->getBallCenter();

    // <<< opp is used later to avoid adding the same vertex twice.
    Vertex * opp = edge->getOppositeVertex();
    // >>>

    double vx,vy,vz;
    vx = tgt->x()-src->x();
    vy = tgt->y()-src->y();
    vz = tgt->z()-src->z();

    normalize(vx,vy,vz);

    // <<< ax, ay, az will be used to determine the angle between two vectors.
    double ax,ay,az;
    ax = center.x() - mp.x();
    ay = center.y() - mp.y();
    az = center.z() - mp.z();
    normalize(ax,ay,az);
    // >>>

    Vertex *candidate = NULL;
    double min_angle = 2.0 * PI;

    for(Vertex_star_list::const_iterator vi = neighbors.begin();
        vi != neighbors.end(); ++vi)
    {
        Vertex *v = *vi;

        //if(v->getType() == Vertex::INNER) //2
        //    continue;
        // Type 2 is an inner vertex. In this function, we do not consider it
        // but it does matter since "findCandidateVertex" will check if the
        // candidate is type 2. If it is, it will set the edge type to 0, which is a border edge.

        // <<< Avoid adding the same vertex twice.
        if(( v == src)||(v == tgt)||(v == opp))
          continue;
        // >>>


        Point new_center;
        if(! computeBallCenter(*src, *tgt, *v, new_center))
          continue;

        // <<< angle computation
        double bx,by,bz;
        bx = new_center.x() - mp.x();
        by = new_center.y() - mp.y();
        bz = new_center.z() - mp.z();
        normalize(bx,by,bz);

        double cosinus = ax * bx + ay * by + az * bz;

        cosinus = 1.0 < cosinus ? 1.0 : cosinus;
        cosinus = -1.0 > cosinus ? -1.0 : cosinus;

        double angle = acos(cosinus);

        double cpx,cpy,cpz;
        cross_product(ax, ay, az, bx, by, bz, cpx, cpy, cpz);

        if( cpx * vx + cpy * vy + cpz * vz < 0)
          angle = 2.0 * PI - angle; //This is the angle correction since acos is only in [0,PI]
        // >>>

        if(angle > min_angle)
          continue;

        if(!checkEmptyBallConfiguration(src, tgt, v, neighbors, new_center))
	        continue;

        min_angle = angle;
        candidate = v;
        candidate_ball_center = new_center;
    }
    //print candidate type
    //if(candidate != NULL && candidate->getType() == 2)
    //    std::cout << "!!!!!!!!   candidate type: " << candidate->getType() << std::endl;
    return candidate;
}


void Mesher::addFacet(Facet* f)
{
    addVertex( f->vertex(0) );
    addVertex( f->vertex(1) );
    addVertex( f->vertex(2) );

    m_facets.push_back(f);
    m_nfacets++;
    
    // add ball center to ball center octree
    //Point ball_center = f->getBallCenter();
    //Point* ball_center_ptr = new Point(f->getBallCenter());
    //m_ball_centers.push_back(ball_center); // copy constructor
    //m_octree_ball_centers.addPoint(m_ball_centers.back());
    //m_num_ball_centers++;
}


void Mesher::addVertex(Vertex* v)
{
    if(v->index() != -1)
        return;
    
    bool is_empty_recycle_set = m_recycle_vertices_idx.empty();
    if (is_empty_recycle_set)
    {
        v->setIndex(m_nvertices);
        m_vertices.push_back(v);
        m_nvertices++;
    }
    else
    {
        unsigned int idx = *(m_recycle_vertices_idx.begin());
        m_recycle_vertices_idx.erase(idx);

        v->setIndex(idx);
        m_vertices.push_back(v);
        m_nvertices++; 
    }

}


std::list< Vertex* >::const_iterator Mesher::vertices_begin() const
{
    return m_vertices.begin();
}

std::list< Vertex* >::const_iterator Mesher::vertices_end() const
{
    return m_vertices.end();
}
std::list< Facet* >::const_iterator Mesher::facets_begin() const
{
    return m_facets.begin();
}

std::list< Facet* >::const_iterator Mesher::facets_end() const
{
    return m_facets.end();
}

void Mesher::fillHoles()
{
    Edge_star_iterator ei= m_border_edges.begin();
    while(ei != m_border_edges.end())
    {
        //during the filling process border edges become inner edges
        //hence the following check
        if((*ei)->getType() != Edge::BORDER)
        {
	        ei = m_border_edges.erase(ei);
	        continue;
        }
        Vertex *src = (*ei)->getSource();
        Vertex *tgt = (*ei)->getTarget();

        Vertex *v = src->findBorder(tgt);

        //if no oriented border links tgt to src (order is important since
        //edges of the front are oriented consistently all over the front)
        if(v == NULL)
        {
	        ++ei;
	        continue;
        }
        
        // computer ball center using order of vertices
        Point center;
        computeBallCenterUsingOrderOfVertices(*tgt, *src, *v, center);
        
        // The new facet should be from target to source to mimic the half-edge data structure.
        Facet *f = new Facet(tgt, src, v, center);
        addFacet(f);
        ei = m_border_edges.erase(ei);
    }
}



void Mesher::findSeedTriangle(OctreeNodeV* containment_node, OctreeNodeV* node,
                                double d, bool& found)
{
    if( node->getDepth() != 0)
    {
        for(unsigned int i = 0; i<8; i++)
        {
            if(node->getChild(i) != NULL)
                findSeedTriangle(containment_node, node->getChild(i), d, found);
        }
    }
    else if( node->getNpts() != 0)
    {
        //Vertex_list::iterator pi = node->points_begin();
        Vertex_UnOrdSet::iterator pi = node->points_begin();
        while( pi != node->points_end())
        {
            Vertex* v = *pi;
            if(v->getType()==Vertex::FRONT) //1
            {
                Edge_set &edges = v->adjacentEdges();
                Edge_set::iterator ei;
                for( ei = edges.begin(); ei != edges.end(); ++ei)
                { // added the parantheses for compiler warning
                    if((*ei)->getType() == Edge::FRONT) // 1
                        m_edge_front.push_front(*ei);
                    expandTriangulationAroundNode(containment_node, d);
                } // added the parantheses for compiler warning
            }
            else if(v->getType() ==Vertex::ORPHAN) //0
            {
                if(trySeed(*v, containment_node, d))
                {
                    found = true;
                    expandTriangulationAroundNode(containment_node, d);
                    continue;
                }
            }
            ++pi;
        }
    }
}

bool Mesher::trySeed(Vertex& v, OctreeNodeV *containment_node, double d)
{
 

    Neighbor_star_map neighbors;
    m_iterator_vertices->setR(2.0 * m_ball_radius);
    m_iterator_vertices->getSortedNeighbors(v, neighbors);
    m_iterator_vertices->setR(m_ball_radius);

    if(neighbors.size()<3)
        return false;

    Neighbor_iterator ni = neighbors.begin();
    while(ni != neighbors.end())
    {
        Vertex &vtest = *(ni->second);
        if( (vtest.getType() != Vertex::ORPHAN) || (&vtest == &v) //0
            || (! containment_node->isInside(vtest, d)) )
        {
            ++ni;
            continue;
        }

        Neighbor_iterator nj = ni;
        ++nj;

        Vertex *candidate = NULL;
        Point center;
        bool changeHandness = false; // just add it now for the program to run. Have to check if it works.
        while(nj != neighbors.end())
        {
            if(tryTriangleSeed(&v, &vtest, nj->second, neighbors, center, changeHandness))
            {
                candidate = nj->second;
                break;
            }
            ++nj;
        }

        // TODO: Use info from Handness further.

        if(candidate == NULL)
        {
            Edge *e = v.getLinkingEdge(&vtest);
            if((e!=NULL)&&(e->getType()==Edge::FRONT)) //1
                m_edge_front.push_front(e);
        }
        else if(containment_node->isInside(*candidate, d))
        {
            Edge *e1 = v.getLinkingEdge(candidate);
            Edge *e2 = vtest.getLinkingEdge(candidate);
            Edge *e3 = v.getLinkingEdge(&vtest);

            if( ((e1!=NULL)&&(e1->getType()!=Edge::FRONT)) //1
                ||((e2!=NULL)&&(e2->getType()!=Edge::FRONT)) //1
                ||((e3!=NULL)&&(e3->getType()!=Edge::FRONT))) //1
            {
                ++ni;
                continue;
            }

            Facet *facet = new Facet(&v, &vtest, candidate, center);
            addFacet(facet);
            
            e1 = v.getLinkingEdge(candidate);
            e2 = vtest.getLinkingEdge(candidate);
            e3 = v.getLinkingEdge(&vtest);

            if(e1->getType() == Edge::FRONT) //1
                m_edge_front.push_front(e1);
            if(e2->getType() == Edge::FRONT) //1
                m_edge_front.push_front(e2);
            if(e3->getType() == Edge::FRONT) //1
                m_edge_front.push_front(e3);

            if(m_edge_front.size() > 0)
                return true;
        }
        ++ni;
    }
    if(m_edge_front.size() > 0)
        return true;
    return false;
}



void Mesher::reconstructAroundNode(OctreeNodeV *containment_node, double d)
{
    if(!m_edge_front.empty())
        expandTriangulationAroundNode(containment_node, d);

    bool found = false;
    findSeedTriangle(containment_node, containment_node, d, found);
}

void Mesher::expandTriangulationAroundNode(OctreeNodeV* containment_node,
                                           double d)
{
    while(! m_edge_front.empty() )
    {
        Edge *edge = m_edge_front.front();
        m_edge_front.pop_front();

        if(edge->getType() != Edge::FRONT)
            continue;

        Point center;
        Vertex *candidate = findCandidateVertex(edge, center);

        if((candidate == NULL) || (candidate->getType()==Vertex::INNER) //2
            ||  (! candidate->isCompatibleWith(*edge)))
        {
            edge->setType(Edge::BORDER);
            m_border_edges.push_back(edge);
            continue;
        }

        Edge *e1 = candidate->getLinkingEdge(edge->getSource());
        Edge *e2 = candidate->getLinkingEdge(edge->getTarget());

        if( ((e1!=NULL) && (e1->getType()!=Edge::FRONT))
            || ((e2!=NULL) && (e2->getType()!=Edge::FRONT)))
        {
            edge->setType(Edge::BORDER);
            m_border_edges.push_back(edge);
            continue;
        }
        //checking that the front remains inside the given node and a small
        // band around it
        if(! containment_node->isInside(*candidate, d))
        {
            //edge->setType(1);
            edge->setType(Edge::FRONT);
            m_node_border_edges.push_back(edge);
            continue;
        }

        Facet * facet = new Facet(edge, candidate, center);
        addFacet(facet);

        e1 = candidate->getLinkingEdge(edge->getSource());
        e2 = candidate->getLinkingEdge(edge->getTarget());

        if(e1->getType() == Edge::FRONT)
            m_edge_front.push_front(e1);

        if(e2->getType() == Edge::FRONT)
            m_edge_front.push_front(e2);
    }
}


void Mesher::collectActiveEdges(OctreeNodeV* containment_node,
                                Edge_set &active_edges)
{
    if(containment_node->getDepth() != 0)
    {
        for(unsigned int i = 0; i <8; ++i)
        {
            if(containment_node->getChild(i) != NULL)
                collectActiveEdges(containment_node->getChild(i),active_edges);
        }
    }
    else
    {
        //Vertex_list::iterator vi;
        Vertex_UnOrdSet::iterator vi;
        for(vi = containment_node->points_begin();
            vi != containment_node->points_end(); ++vi)
            {
                Vertex* v = *vi;
                if(v->getType() != Vertex::FRONT) //1
                    continue;

                Edge_set &edges = v->adjacentEdges();
                Edge_set::iterator ei;
                for(ei = edges.begin(); ei != edges.end(); ++ei)
                {
                    Edge *e= *ei;
                    if(e->getType() == Edge::FRONT)
                        active_edges.insert(*ei);
                }
            }
    }
}


void Mesher::collectBorderEdges(OctreeNodeV *containment_node)
{
    Edge_set border_edges;
    collectBorderEdges(containment_node, border_edges);
    m_border_edges.insert(m_border_edges.end(), border_edges.begin(),
                          border_edges.end());
}

void Mesher::collectBorderEdges(OctreeNodeV* containment_node,
                                Edge_set& border_edges)
{
    if(containment_node->getDepth() != 0)
    {
        for(unsigned int i = 0; i <8; ++i)
        {
            if(containment_node->getChild(i) != NULL)
                collectBorderEdges(containment_node->getChild(i),border_edges);
        }
    }
    else
    {
        //Vertex_list::iterator vi;
        Vertex_UnOrdSet::iterator vi;
        for(vi = containment_node->points_begin();
            vi != containment_node->points_end(); ++vi)
            {
                Vertex* v = *vi;
                if(v->getType() != Vertex::FRONT) //1
                    continue;

                Edge_set &edges = v->adjacentEdges();
                Edge_set::iterator ei;
                for(ei = edges.begin(); ei != edges.end(); ++ei)
                {
                    Edge *e= *ei;
                    if(e->getType() == Edge::BORDER)
                        border_edges.insert(*ei);
                }
            }
    }
}




void Mesher::merge(Mesher& mesher)
{
    //merging the sets of facets
    m_facets.splice(m_facets.end(), mesher.m_facets);

    Edge_star_list::iterator ei = m_edge_front.begin();
    //merging the edge fronts
    while( ei != m_edge_front.end())
    {
        if((*ei)->getType() == Edge::INNER)
        {
            ei = m_edge_front.erase(ei);
            continue;
        }

        else if((*ei)->getType() == Edge::FRONT)
        {
            ++ei;
        }

        else if((*ei)->getType() == Edge::BORDER)
        {
            ei = m_edge_front.erase(ei);
            continue;
        }
    }

    //merging the sets of vertices and renumbering them
    Vertex_star_list::iterator vi;
    for(vi = mesher.m_vertices.begin(); vi != mesher.m_vertices.end(); ++vi)
    {
        Vertex *v = *vi;
        unsigned int index = v->index();
        v->setIndex(index + m_nvertices);
        m_vertices.push_back(v);
    }

    //adding the node border edges (edges that could not be expanded due
    //to spatial containment) and add them to the front
    for(ei = mesher.m_node_border_edges.begin();
        ei != mesher.m_node_border_edges.end() ; ++ei)
        {
            m_edge_front.push_back(*ei);
        }
        mesher.m_node_border_edges.clear();

    //get the border edges and add it to the global mesher
    for(ei = mesher.m_border_edges.begin();
        ei != mesher.m_border_edges.end() ; ++ei)
        {
            if((*ei)->getFacet2()!=NULL)
            {
                continue;
            }
            m_border_edges.push_back(*ei);
        }
        mesher.m_border_edges.clear();

    m_nfacets = m_facets.size();
    m_nvertices = m_vertices.size();
}

std::set<Facet*>& Mesher::getBoundaryFacets() const
{
    std::set<Facet*> *boundary_facets = new std::set<Facet*>();
    //auto boundary_facets = std::make_unique<std::set<Facet*>>(); // Smart pointer to manage the memory, please use #include <memory>
    for (const auto& edge : m_border_edges)
    {
        Vertex *v1 = edge->getSource();
        Vertex *v2 = edge->getTarget();
        
        Facet_set facets_set1 = v1->adjacentFacets();
        Facet_set facets_set2 = v2->adjacentFacets();
        for (const auto& facet : facets_set1)
        {
            if (facet->isNewlyArrived())
            {
                boundary_facets->insert(facet);
            }
        }
        for (const auto& facet : facets_set2)
        {
            if (facet->isNewlyArrived())
            {
                boundary_facets->insert(facet);
            }
        }
    }
    std::cout << "Address of boundary_facets in getBoundaryFacets: " << boundary_facets << std::endl;
    return *boundary_facets;
}

void Mesher::removeOrphanVertices(TOctreeNode<Vertex>* node)
{
    Vertex_UnOrdSet& points = node->GetPoints();    

    std::list<Vertex*> temporary_orphan_vertices;
    for ( auto iter = points.begin(); iter != points.end(); ++iter)
    {
        Vertex* v = *iter;
        if (v->getType() == Vertex::ORPHAN)
        {
            temporary_orphan_vertices.push_back(v);
        }
    } 
    
    for (auto v : temporary_orphan_vertices)
    {
        // Update node
        node->decreaseNptsByOne();
        // Update the vertices set of the node.
        points.erase(v);
        // Remove the vertex from the mesh.
        m_vertices.remove(v); // m_vertices is a std::list<Vertex*>.
        // Retrieve the index of the vertex.
        m_trimmed_vertices_idx.insert(v->index());
        // Delete the vertex.
        delete v;
    } 
    
    // Clear the temporary list to free the memory.
    temporary_orphan_vertices.clear();

    // Update the node that if the vertex is trimmed, change it into ORPHAN.
    for (auto v : points)
    {
        if (v->getType() == Vertex::TRIMMED)
        {
            v->setType(Vertex::ORPHAN);
        }
    }
    
}

void Mesher::trimBoundaryFacets(std::set<Facet*> &boundary_facets)
{
    for (auto facet : boundary_facets)
    {
        auto it = std::find(m_facets.begin(), m_facets.end(), facet);
        if (it != m_facets.end())
        {
            m_facets.erase(it);
        } 
        delete facet;
        facet = NULL;
    }
    
    // Important. ToDo get new boundary edges.
    m_border_edges.clear();
    std::set<Edge*> new_boundary_edges = Facet::getRecordedNewBoundaryEdges();
    m_border_edges = std::list<Edge*>(new_boundary_edges.begin(), new_boundary_edges.end());

    // Update OctreeVertices to remove the points that are no longer in use, which it is orphan.
    //m_iterator_vertices->loopOverAllNodes(&Mesher::removeOrphanVertices);
    m_iterator_vertices->loopOverAllNodes([this](TOctreeNode<Vertex>* node) {
        this->removeOrphanVertices(node); // m_vertices also gets updated.
        // TODO: If the node is empty, remove it from it's parent.
    });
      
}

const std::list<Facet*>& Mesher::getFacets() const
{
    return m_facets;
}

void Mesher::setAllFacetsToOld()
{
    for (auto facet : m_facets)
    {
        facet->setNewlyArrived(false);
    }
}

void Mesher::putBallCentersInOctree()
{
    for (auto facet : m_facets)
    {
        Point ball_center = facet->getBallCenter();
        Point* new_pt = m_octree_ball_centers->checkSizeAndaddPoint(ball_center);
        m_num_ball_centers++;
        m_ball_centers.insert(new_pt);

        #ifdef _DEBUG
        // if all zero, print the index, print all vertices
        //if (ball_center.x() == 0 && ball_center.y() == 0 && ball_center.z() == 0)
        //{
        //    Vertex* v0 = facet->getVertex(0);
        //    Vertex* v1 = facet->getVertex(1);
        //    Vertex* v2 = facet->getVertex(2);
        //    std::cout << "Vertex 0: " << v0->index() << std::endl;
        //    std::cout << "Vertex 1: " << v1->index() << std::endl;
        //    std::cout << "Vertex 2: " << v2->index() << std::endl;
        //    // Print location of the vertices
        //    std::cout << "Location of vertex 0: " << v0->x() << " " << v0->y() << " " << v0->z() << std::endl;
        //    std::cout << "Location of vertex 1: " << v1->x() << " " << v1->y() << " " << v1->z() << std::endl;
        //    std::cout << "Location of vertex 2: " << v2->x() << " " << v2->y() << " " << v2->z() << std::endl;
        //}
        #endif
    }
}

Edge_star_list Mesher::getBorderEdges() const
{
    return m_border_edges;
}

bool sameOrientation(int i, Facet* query_facet)
{
    
    Vertex* queryV_Source = query_facet->getVertex(i);
    Vertex* queryV_Target = query_facet->getVertex(i+1);
    
    Edge* e0 = queryV_Source->getLinkingEdge(queryV_Target);
    Facet* facet_1, *facet_2;
    facet_1 = e0->getFacet1();
    facet_2 = e0->getFacet2();

    Facet* adjacent_facet = (facet_1 == query_facet) ? facet_2 : facet_1;
    if (adjacent_facet == NULL)
    {
        return true;
    }
    else
    {
       for (int k = 0; k < 3; k++)
       {
            Vertex* adjV_source = adjacent_facet->getVertex(k);
            Vertex* adjV_target = adjacent_facet->getVertex(k+1);
            if (adjV_source == queryV_Target && adjV_target == queryV_Source)
            {
                return true;
            }
       }
        std::cout <<" Adress of query facet: " << query_facet << std::endl;
        std::cout <<" Adress of queryV_Source: " << queryV_Source << std::endl;
        std::cout <<" Adress of queryV_Target: " << queryV_Target << std::endl;
        std::cout <<" Adress of adjacent_facet: " << adjacent_facet << std::endl;
        std::cout <<" Adress of adjV_1: " << adjacent_facet->getVertex(0) << std::endl;
        std::cout <<" Adress of adjV_2: " << adjacent_facet->getVertex(1) << std::endl;
        std::cout <<" Adress of adjV_3: " << adjacent_facet->getVertex(2) << std::endl;

        std::cerr << "Error: The orientation of the facet is incorrect." << std::endl;
        std::exit(EXIT_FAILURE);

       return false;
    } 
}

void Mesher::SanityCheckOrientation() const
{
    for (auto facet : m_facets)
    {
        for (int i = 0; i < 3; i++)
        {
            if (sameOrientation(i, facet))
            {
                continue;
            }
            else
            {
                std::cerr << "Error: The orientation of the facet is incorrect." << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
        
    }
}
