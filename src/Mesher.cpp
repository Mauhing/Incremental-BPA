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
#include "BallCenter.h"
#include <cstdlib>
#include <cmath>
#include <cassert>
#ifndef USE_CLANG
#include <omp.h>
#endif
#include <sstream>
#include <memory>
#include <algorithm>
#include "Visualizer.h"

const double PI = 3.1415926535;

//using namespace std;


template <typename K>
static bool isInSet(const K &element, const std::set<K> &mySet)
{
    return mySet.find(element) != mySet.end();
}

template <typename K>
static bool isNotInSet(const K &element, const std::set<K> &mySet)
{
    return mySet.find(element) == mySet.end();
}

Mesher::Mesher() : visualization_mutex(nullptr),
                   visualization_cv(nullptr),
                   new_facet_added(false)
{
    m_octree_vertices = NULL;
    m_iterator_vertices = NULL;
    m_nfacets = 0;
    m_vertice_idx = 0;
    m_recycle_vertices_idx = std::unordered_set<unsigned int>();
    m_octree_ball_centers = NULL;
    m_octree_ball_centers_iterator = NULL;
}

Mesher::Mesher(OctreeVertices *octree_vertices, OctreeIteratorVertices *iterator_vertices,
               OctreeBallCenters *octree_ball_centers, OctreeIteratorBallCenters *octree_ball_centers_iterator)
    : visualization_mutex(nullptr),
      visualization_cv(nullptr),
      new_facet_added(false)
{
    m_ball_radius = iterator_vertices->getR();
    m_sq_ball_radius = m_ball_radius * m_ball_radius;
    m_nfacets = 0;
    m_vertice_idx = 0;
    m_recycle_vertices_idx = std::unordered_set<unsigned int>();

    m_octree_vertices = octree_vertices;
    m_iterator_vertices = iterator_vertices;
    m_octree_ball_centers = octree_ball_centers;
    m_octree_ball_centers_iterator = octree_ball_centers_iterator;
}

Mesher::~Mesher()
{
    std::cout << "Mesher destructor" << std::endl;
    m_octree_vertices = nullptr;
    m_iterator_vertices = nullptr;
    m_octree_ball_centers = nullptr;
    m_octree_ball_centers_iterator = nullptr;

    m_edge_front.clear();
    m_border_edges.clear();

    Facet_star_list::iterator fi;
    for (fi = m_facets.begin(); fi != m_facets.end(); ++fi)
    {
        // delete *fi;
        removeFacet(*fi);
        *fi = NULL;
    }
    m_facets.clear();

    m_vertices.clear();
    m_nfacets = 0;
    m_vertice_idx = 0;
}

void Mesher::setBallRadius(double r)
{
    m_ball_radius = r;
    m_sq_ball_radius = r * r;
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
    return static_cast<unsigned int>(m_vertices.size());
}

unsigned int Mesher::nFacets() const
{
    return m_nfacets;
}

unsigned int Mesher::nFrontEdges() const
{
    return (unsigned int)m_edge_front.size();
}

unsigned int Mesher::nBorderEdges() const
{
    return (unsigned int)m_border_edges.size();
}

// unsigned int Mesher::getNumBallCenters() const
//{
//     return m_num_ball_centers;
// }

void Mesher::reconstruct()
{
    std::cout << "***********Ball radius " << m_ball_radius
              << " ***********" << std::endl;

    if (m_edge_front.empty())
    {
        std::cout << "No front edge found, looking for seed triangle."
                  << std::endl;
        bool ok = findSeedTriangle();
        if (!ok)
            std::cout << "No seed triangle found, no triangulation done!"
                      << std::endl;
    }
    else
    {
        // If there are only one radius. This scope will never be executed.
        std::cout << "Expanding triangulation" << std::endl;
        expandTriangulation();
    }
}

void Mesher::reconstruct(const std::list<double> &radii)
{ 

    std::cout << "single threaded reconstruction" << std::endl;
    for (double radius : radii)
    {
        changeRadius(radius);
        reconstruct();
    }
}

void Mesher::changeRadius(double radius)
{
    setBallRadius(radius);
    Edge_star_list::iterator ei = m_border_edges.begin();
    while (ei != m_border_edges.end())
    {
        Edge *e = *ei;
        Facet *f = e->getFacet1();

        Point center;
        if (emptyBallConfiguration(f->vertex(0), f->vertex(1),
                                   f->vertex(2), center))
        {
            e->setType(Edge::FRONT);
            m_edge_front.push_back(e);
            ei = m_border_edges.erase(ei); // It returns the next iterator after the erased element
            continue;
        }
        ++ei;
    }
    std::cout << "After changing radius, " << m_edge_front.size()
              << " front edges and " << m_border_edges.size()
              << " border edges." << std::endl;
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

void Mesher::findSeedTriangle(OctreeNodeV *node, bool &found)
{
    if (node->getDepth() != 0)
    {
        for (unsigned int i = 0; i < 8; i++)
        {
            if (node->getChild(i) != NULL)
                findSeedTriangle(node->getChild(i), found);
        }
    }
    else if (node->getNpts() != 0)
    {
        // Vertex_list::iterator pi = node->points_begin();
        typename Vertex_UnOrdSet::const_iterator pi = node->points_begin();
        while (pi != node->points_end())
        {
            // current approach is find seed and expand
            // then find a seed again and expand.
            Vertex *v = *pi;
            if (v->getType() == Vertex::ORPHAN) // 0
            {
                if (trySeed(*v))
                {
                    found = true;
                    expandTriangulation();
                }
            }
            ++pi;
        }
    }
}

bool Mesher::trySeed(Vertex &v)
{
    // First, get the neighbors of the current vertex v.
    // ni are the next vertices in the sorted neighbor list
    // nj are the second next vertices in the sorted neighbor list
    // than sliding over the sorted neighbor list until we find a valid seed triangle

    Neighbor_star_map neighbors;
    m_iterator_vertices->setR(2.0 * m_ball_radius);
    m_iterator_vertices->getSortedNeighbors(v, neighbors);
    m_iterator_vertices->setR(m_ball_radius);

    if (neighbors.size() < 3)
        return false;

    Neighbor_iterator ni = neighbors.begin();
    while (ni != neighbors.end())
    {
        Vertex &vtest = *(ni->second);                             // The "second" is the vertex pointer stored in the neighbor star map
        if ((vtest.getType() != Vertex::ORPHAN) || (&vtest == &v)) // 0
        {
            ++ni;
            continue;
        }

        Neighbor_iterator nj = ni;
        ++nj;

        Vertex *candidate = NULL;
        Point center;

        bool changeHandness = false;
        while (nj != neighbors.end())
        {
            if (tryTriangleSeed(&v, &vtest, nj->second, neighbors, center, changeHandness))
            {
                candidate = nj->second;
                break;
            }
            ++nj;
        }

        if (candidate != NULL)
        {
            // <<<
            // To check any of the edges are not a front edge, we can use the getLinkingEdge method
            Edge *e1 = v.getLinkingEdge(candidate);
            Edge *e2 = vtest.getLinkingEdge(candidate);
            Edge *e3 = v.getLinkingEdge(&vtest);

            if (((e1 != NULL) && (e1->getType() != Edge::FRONT)) || ((e2 != NULL) && (e2->getType() != Edge::FRONT)) || ((e3 != NULL) && (e3->getType() != Edge::FRONT)))
            {
                ++ni;
                continue;
            }
            // >>>
            // continue tommorrow with adding the flipIndex part.
            Vertex *v0 = &v;
            Vertex *v1 = &vtest;
            Vertex *v2 = candidate;
            if (changeHandness)
            {
                Vertex *temp = v1;
                v1 = v2;
                v2 = temp;
            }
            // <<<
            // Now, the seed triangle will take account of the handness.
            Facet *facet = this->createFacet(v0, v1, v2, center);
            this->addFacet(facet);
            // >>>

            if (m_nfacets % 10000 == 0)
                std::cout << m_nfacets << " facets. "
                          << m_edge_front.size() << " front edges. "
                          << m_border_edges.size() << " border edges." << std::endl;

            e1 = v.getLinkingEdge(candidate);
            e2 = vtest.getLinkingEdge(candidate);
            e3 = v.getLinkingEdge(&vtest);

            // std::cout << "e1 start vertex: " << e1->getSource()->index() << std::endl;
            // std::cout << "e2 start vertex: " << e2->getSource()->index() << std::endl;
            // std::cout << "e3 start vertex: " << e3->getSource()->index() << std::endl;
            // std::cout << "================================" << std::endl;

            if (e1->getType() == Edge::FRONT)
                m_edge_front.push_front(e1);
            if (e2->getType() == Edge::FRONT)
                m_edge_front.push_front(e2);
            if (e3->getType() == Edge::FRONT)
                m_edge_front.push_front(e3);

            if (m_edge_front.size() > 0)
                return true;
        }
        ++ni;
    }
    return false;
}

bool Mesher::tryTriangleSeed(Vertex *v1, Vertex *v2, Vertex *v3,
                             Neighbor_star_map &neighbors,
                             Point &center,
                             bool &changeHandness) const
{
    // if((v3->getType() != Vertex::ORPHAN) || ( !v3->isCompatibleWith(*v1, *v2)))
    //     return false;
    if ((v3->getType() != Vertex::ORPHAN) || (!v3->isCompatibleWithAndHandnessCheck(*v1, *v2, changeHandness)))
        return false;

    Edge *e1 = v1->getLinkingEdge(v3);
    Edge *e2 = v2->getLinkingEdge(v3);
    if (((e1 != NULL) && (e1->getType() == Edge::INNER)) || ((e2 != NULL) && (e2->getType() == Edge::INNER)))
        return false;

    m_iterator_vertices->setR(m_ball_radius);
    if (!computeBallCenter(*v1, *v2, *v3, center))
        return false;

    Neighbor_iterator ni;
    for (ni = neighbors.begin(); ni != neighbors.end(); ++ni)
    {
        Vertex *v = ni->second;
        if ((v == v1) || (v == v2) || (v == v3))
            continue;
        if (dist2(center, *v) < m_sq_ball_radius - 1e-16)
            return false;
    }
    return true;
}

bool Mesher::emptyBallConfiguration(Vertex *v1, Vertex *v2, Vertex *v3,
                                    Point &center) const
{
    m_iterator_vertices->setR(m_ball_radius);
    if (!computeBallCenter(*v1, *v2, *v3, center))
        return false;

    std::set<Vertex *> facet_vertices;

    facet_vertices.insert(v1);
    facet_vertices.insert(v2);
    facet_vertices.insert(v3);

    return m_iterator_vertices->containsOnly(center, facet_vertices);
}

bool Mesher::checkEmptyBallConfiguration(Vertex *v1, Vertex *v2, Vertex *v3,
                                         const Vertex_star_list &neighbors,
                                         const Point &center) const
{
    Vertex_star_list::const_iterator ni;
    for (ni = neighbors.begin(); ni != neighbors.end(); ++ni)
    {
        const Vertex *v = *ni;
        if ((v == v1) || (v == v2) || (v == v3))
            continue;
        if (dist2(*v, center) < m_sq_ball_radius - 1e-16)
        {
            // if(v->getType() == Vertex::INNER)//2
            //  print v type
            // std::cout << "v type: " << v->getType() << std::endl;
            // std::cout << "v is inner" << std::endl;
            return false;
        }
    }
    return true;
}

bool Mesher::computeBallCenterUsingOrderOfVertices(const Vertex &v1, const Vertex &v2,
                                                   const Vertex &v3, Point &center) const
{
    // compute the circumcenter barycentric coordinates
    double c = dist2(v2, v1);
    double b = dist2(v1, v3);
    double a = dist2(v3, v2);
    double alpha = a * (b + c - a);
    double beta = b * (a + c - b);
    double gamma = c * (a + b - c);
    double temp = alpha + beta + gamma;

    if (temp < 1e-30) // aligned case
        return false;

    alpha = alpha / temp;
    beta = beta / temp;
    gamma = gamma / temp;

    // computing the triangle circumcircle center
    double x = alpha * v1.x() + beta * v2.x() + gamma * v3.x();
    double y = alpha * v1.y() + beta * v2.y() + gamma * v3.y();
    double z = alpha * v1.z() + beta * v2.z() + gamma * v3.z();

    // computing the radius of the circumcircle
    double sq_circumradius = a * b * c;

    a = sqrt(a);
    b = sqrt(b);
    c = sqrt(c);

    sq_circumradius = sq_circumradius /
                      ((a + b + c) * (b + c - a) * (c + a - b) * (a + b - c));

    // compute the ortogonal distance from the hypothetic center to the triangle
    double height = m_sq_ball_radius - sq_circumradius;

    // compute the normal of the three points
    double nx, ny, nz = 0;

    if (height >= 0.0)
    {
        computeNormal(v1, v2, v3, nx, ny, nz);
        height = sqrt(height);
        center = Point(x + height * nx, y + height * ny, z + height * nz);
        return true;
    }
    return false;
}

bool Mesher::computeBallCenter(const Vertex &v1, const Vertex &v2,
                               const Vertex &v3, Point &center) const
{
    // compute the circumcenter barycentric coordinates
    double c = dist2(v2, v1);
    double b = dist2(v1, v3);
    double a = dist2(v3, v2);
    double alpha = a * (b + c - a);
    double beta = b * (a + c - b);
    double gamma = c * (a + b - c);
    double temp = alpha + beta + gamma;

    if (temp < 1e-30) // aligned case
        return false;

    alpha = alpha / temp;
    beta = beta / temp;
    gamma = gamma / temp;

    // computing the triangle circumcircle center
    double x = alpha * v1.x() + beta * v2.x() + gamma * v3.x();
    double y = alpha * v1.y() + beta * v2.y() + gamma * v3.y();
    double z = alpha * v1.z() + beta * v2.z() + gamma * v3.z();

    // computing the radius of the circumcircle
    double sq_circumradius = a * b * c;

    a = sqrt(a);
    b = sqrt(b);
    c = sqrt(c);

    sq_circumradius = sq_circumradius /
                      ((a + b + c) * (b + c - a) * (c + a - b) * (a + b - c));

    // compute the ortogonal distance from the hypothetic center to the triangle
    double height = m_sq_ball_radius - sq_circumradius;

    // compute the normal of the three points
    double nx, ny, nz = 0;

    if (height >= 0.0)
    {
        computeNormal(v1, v2, v3, nx, ny, nz);
        height = sqrt(height);
        center = Point(x + height * nx, y + height * ny, z + height * nz);
        return true;
    }
    return false;
}

// This function still use the normal of the input vertices.
void Mesher::computeNormal(const Vertex &v1, const Vertex &v2, const Vertex &v3,
                           double &nx, double &ny, double &nz) const
{
    cross_product(v2.x() - v1.x(), v2.y() - v1.y(), v2.z() - v1.z(),
                  v3.x() - v1.x(), v3.y() - v1.y(), v3.z() - v1.z(),
                  nx, ny, nz);
    normalize(nx, ny, nz);

    double mnx = v1.nx() + v2.nx() + v3.nx();
    double mny = v1.ny() + v2.ny() + v3.ny();
    double mnz = v1.nz() + v2.nz() + v3.nz();

    normalize(mnx, mny, mnz);

    if (nx * mnx + ny * mny + nz * mnz < 0)
    {
        nx = -nx;
        ny = -ny;
        nz = -nz;
    }
}

void Mesher::computeNormalUsingOrderOfVertices(const Vertex &v1, const Vertex &v2, const Vertex &v3,
                                               double &nx, double &ny, double &nz) const
{
    cross_product(v2.x() - v1.x(), v2.y() - v1.y(), v2.z() - v1.z(),
                  v3.x() - v1.x(), v3.y() - v1.y(), v3.z() - v1.z(),
                  nx, ny, nz);
    normalize(nx, ny, nz);
}

ReconstructionType Mesher::computeReconstructionType(
    const Edge *eSource,
    const Edge *eTarget,
    const Vertex *candidate) const
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
    for (int i = 0; i < 3; i++)
    {
        Vertex *v0 = facet->vertex(i);
        Vertex *v1 = facet->vertex((i + 1) % 3);
        if (v0 == candidate && v1 == Source)
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
    for (int i = 0; i < 3; i++)
    {
        Vertex *v0 = facet->vertex(i);
        Vertex *v1 = facet->vertex((i + 1) % 3);
        if (v0 == Target && v1 == candidate)
        {
            isGoodOrentation = true;
            break;
        }
    }
    return isGoodOrentation;
}

void Mesher::expandTriangulation()
{
    while (!m_edge_front.empty())
    {
        Edge *edge = m_edge_front.front();
        m_edge_front.pop_front();

        if (edge->getType() != Edge::FRONT)
            continue;

        Point center;
        Vertex *candidate = findCandidateVertex(edge, center);

        if ((candidate == NULL) || (candidate->getType() == Vertex::INNER) // 2
            || (!candidate->isCompatibleWith(*edge)))
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

        if (reconstructionType == ReconstructionType::NO_RECONSTRUCTION)
        {
            edge->setType(Edge::BORDER);
            m_border_edges.push_back(edge);
            continue;
        }

        // <<< One more condition with good orentation.
        if (reconstructionType == ReconstructionType::EAR_FILLING_FrontEdge_SOURCE)
        {
            if (!isGoodOrentationWithSource(eSource, edge->getSource(), candidate))
            {
                std::cout << "Edge is not good orentation with source" << std::endl;
                edge->setType(Edge::BORDER);
                m_border_edges.push_back(edge);
                continue;
            }
        }
        if (reconstructionType == ReconstructionType::EAR_FILLING_FrontEdge_TARGET)
        {
            if (!isGoodOrentationWithTarget(eTarget, edge->getTarget(), candidate))
            {
                std::cout << "Edge is not good orentation with target" << std::endl;
                edge->setType(Edge::BORDER);
                m_border_edges.push_back(edge);
                continue;
            }
        }
        // >>>

        Facet *facet = this->createFacet(edge, candidate, center);
        addFacet(facet);

        Edge *e1 = candidate->getLinkingEdge(edge->getSource());
        Edge *e2 = candidate->getLinkingEdge(edge->getTarget());

        if (e1->getType() == Edge::FRONT)
            m_edge_front.push_front(e1);

        if (e2->getType() == Edge::FRONT)
            m_edge_front.push_front(e2);

        // if(m_nfacets % 10000 == 0)
        //     std::cout<<m_vertice_idx<<" vertices. "<<m_nfacets<<" facets. "
        //     <<m_edge_front.size()<<" front edges. "
        //     <<m_border_edges.size()<<" border edges."<<std::endl;
    }
}

Vertex *Mesher::findCandidateVertex(Edge *edge, Point &candidate_ball_center)
{
    Vertex *src = edge->getSource();
    Vertex *tgt = edge->getTarget();

    Point mp = midpoint(*src, *tgt);
    Vertex_star_list neighbors;

    double d = m_ball_radius + sqrt(m_sq_ball_radius - dist2(mp, *src));
    m_iterator_vertices->setR(d);
    m_iterator_vertices->getNeighbors(mp, neighbors);
    m_iterator_vertices->setR(m_ball_radius);

    Facet *facet = edge->getFacet1(); // Get the first facet.
    const Point &center = facet->getBallCenter();

    // <<< opp is used later to avoid adding the same vertex twice.
    Vertex *opp = edge->getOppositeVertex();
    // >>>

    double vx, vy, vz;
    vx = tgt->x() - src->x();
    vy = tgt->y() - src->y();
    vz = tgt->z() - src->z();

    normalize(vx, vy, vz);

    // <<< ax, ay, az will be used to determine the angle between two vectors.
    double ax, ay, az;
    ax = center.x() - mp.x();
    ay = center.y() - mp.y();
    az = center.z() - mp.z();
    normalize(ax, ay, az);
    // >>>

    Vertex *candidate = NULL;
    double min_angle = 2.0 * PI;

    for (Vertex_star_list::const_iterator vi = neighbors.begin();
         vi != neighbors.end(); ++vi)
    {
        Vertex *v = *vi;

        // if(v->getType() == Vertex::INNER) //2
        //     continue;
        //  Type 2 is an inner vertex. In this function, we do not consider it
        //  but it does matter since "findCandidateVertex" will check if the
        //  candidate is type 2. If it is, it will set the edge type to 0, which is a border edge.

        // <<< Avoid adding the same vertex twice.
        if ((v == src) || (v == tgt) || (v == opp))
            continue;
        // >>>

        Point new_center;
        if (!computeBallCenter(*src, *tgt, *v, new_center))
            continue;

        // <<< angle computation
        double bx, by, bz;
        bx = new_center.x() - mp.x();
        by = new_center.y() - mp.y();
        bz = new_center.z() - mp.z();
        normalize(bx, by, bz);

        double cosinus = ax * bx + ay * by + az * bz;

        cosinus = 1.0 < cosinus ? 1.0 : cosinus;
        cosinus = -1.0 > cosinus ? -1.0 : cosinus;

        double angle = acos(cosinus);

        double cpx, cpy, cpz;
        cross_product(ax, ay, az, bx, by, bz, cpx, cpy, cpz);

        if (cpx * vx + cpy * vy + cpz * vz < 0)
            angle = 2.0 * PI - angle; // This is the angle correction since acos is only in [0,PI]
        // >>>

        if (angle > min_angle)
            continue;

        if (!checkEmptyBallConfiguration(src, tgt, v, neighbors, new_center))
            continue;

        min_angle = angle;
        candidate = v;
        candidate_ball_center = new_center;
    }
    // print candidate type
    // if(candidate != NULL && candidate->getType() == 2)
    //     std::cout << "!!!!!!!!   candidate type: " << candidate->getType() << std::endl;
    return candidate;
}

void Mesher::addFacet(Facet *f)
{
    std::unique_lock<std::mutex> lock(*visualization_mutex);
    addVertex(f->vertex(0));
    addVertex(f->vertex(1));
    addVertex(f->vertex(2));

    m_facets.push_back(f);
    addFreshFacet(f);
    m_nfacets++;

    // Update the ball centers octree.
    Point temp_ball_center = f->getBallCenter();
    BallCenter *ball_center = m_octree_ball_centers->checkSizeAndaddPoint(BallCenter(temp_ball_center, f));
    f->setBallCenterPtr(ball_center);

    new_facet_added = true;
    visualization_cv->notify_one();

    if (m_slow_visualization)
    {
        // Release mutex before sleep to allow visualization updates
        lock.unlock();
        // std::cout << "Press enter to continue" << std::endl;
        // std::cin.get();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        lock.lock();
    }
}

std::pair<bool, Facet_set> Mesher::extractConnectedFacets(Vertex *v, const Facet_set &facets) const
{
    #ifdef _DEBUG
    if (v->getType() == Vertex::ORPHAN)
    {
        std::cerr << "\033[1;31mThe vertex is type ORPHAN\033[0m" << std::endl;
        std::cerr << "Vertex: " << v << std::endl;
        std::exit(EXIT_FAILURE);
    }
    #endif

    Facet *f = *(facets.begin());

    // Check if the facet is a disk facet.
    bool is_disk_facet_set = false;
    Facet_set connected_facets;
    Facet *starting_facet = f;
    connected_facets.insert(starting_facet);
    
    // 1. Following orientation
    Facet *current_facet = starting_facet;
    while (true)
    {
        // The problem is the code in this scope.
        Vertex *next_v = current_facet->nextVertex(v);        
        Edge* e = v->getLinkingEdge(next_v); 
        Facet *next_facet = e->anotherFacet(current_facet); 

        #ifdef _DEBUG
        if (next_facet != nullptr && isNotInSet(next_facet, facets))
        {
            std::cerr << "\033[1;31mError: Neighbor facet is not in the set of facets\033[0m" << std::endl;
            std::cerr << "Error coming from extractConnectedFacets: Following orientation" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        #endif
        if (next_facet == nullptr)
        {
            // Case: Fan
            break;
        }
        if (next_facet == starting_facet)
        {
            // Case: Circle
            #ifdef _DEBUG
            if (connected_facets.size() < 3)
            {
                std::cerr << "\033[1;31mError: Connected facets size is less than 3\033[0m" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            #endif
            is_disk_facet_set = true;
            break;
        }
        connected_facets.insert(next_facet);
        current_facet = next_facet;
    }
    
    if (is_disk_facet_set)
    {
        return std::make_pair(is_disk_facet_set, connected_facets);
    }
    
    // 2. Opposite orientation
    current_facet = starting_facet;
    while (true)
    {
        // The problem is the code in this scope.
        Vertex *prev_v = starting_facet->previousVertex(v);        
        Edge* e = v->getLinkingEdge(prev_v); 
        Facet *prev_facet = e->anotherFacet(starting_facet); 

        #ifdef _DEBUG
        if (prev_facet != nullptr && isNotInSet(prev_facet, facets))
        {
            std::cerr << "\033[1;31mError: Neighbor facet is not in the set of facets\033[0m" << std::endl;
            std::cerr << "Error coming from extractConnectedFacets: Opposite orientation" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        #endif
        if (prev_facet == nullptr)
        {
            // Case: Fan
            break;
        }

        connected_facets.insert(prev_facet);
        starting_facet = prev_facet;
    }
    return std::make_pair(is_disk_facet_set, connected_facets);
}

Mesher::VertexDiskFanInfo Mesher::getDiskFan(Vertex *v) const
{
    Facet_set neighbor_facets = v->adjacentFacets();
    VertexDiskFanInfo vertex_disk_fan_info;
    vertex_disk_fan_info.disk_fan_vertex = v;

    while (!neighbor_facets.empty())
    {
        std::pair<bool, Facet_set> result = extractConnectedFacets(v, neighbor_facets);
        bool is_disk_facet_set = result.first;
        Facet_set detected_facets = result.second;

        if (is_disk_facet_set == false)
        {
            vertex_disk_fan_info.fan_facets.push_back(detected_facets);
        }
        else
        {
            vertex_disk_fan_info.disk_facets.push_back(detected_facets);
        }

        for (Facet *f : detected_facets)
        {
            neighbor_facets.erase(f);
        }
    }
    return vertex_disk_fan_info;
}


void Mesher::removeRedundantDiskFanFacet(Vertex *nm_vertex, Facet *facet)
{
    // Policy:
    // No vertex get deleted. Edge with no neighbour facets are deleted.
    // Edge with neighbour facets are set to border and their vertices are set to FRONT.
    Vertex *vertex[3];
    for (int i = 0; i < 3; ++i)
    {
        vertex[i] = facet->getVertex(i);
    }


    std::set<Edge*> edgesTouchingNmVertex; 
    
    // Deal with the edges first.
    for (int i = 0; i < 3; ++i)
    {
        int source_idx = i;
        int target_idx = (i + 1) % 3;
        Vertex *source_vertex = vertex[source_idx];
        Vertex *target_vertex = vertex[target_idx];
        Edge *e = source_vertex->getLinkingEdge(target_vertex);

        if (nm_vertex == source_vertex || nm_vertex == target_vertex)
        {
            edgesTouchingNmVertex.insert(e);
        }

        bool doesFacet1Exist = (e->getFacet1() != NULL);
        bool doesFacet2Exist = (e->getFacet2() != NULL);
        
        bool full_edge = doesFacet1Exist && doesFacet2Exist;
        bool half_edge = !full_edge;

        if (half_edge)
        {
            #ifdef _DEBUG
            if (!m_edge_front.empty())
            {
                std::cerr << "\033[1;31mEdge front is not empty\033[0m" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            #endif
            // actually, only one of the contains has the edge.
            // but we remove it from both since we do not know which one it is.
            m_border_edges.remove(e); 
            source_vertex->removeAdjacentEdge(e);
            target_vertex->removeAdjacentEdge(e);
            delete e;
        }
        else
        {
            // full edge.
            e->removeAdjacentFacet(facet);
            e->setType(Edge::BORDER);
            m_border_edges.push_back(e);
        } 
    }
    
    // Deal with the vertices.
    for (Vertex *v : vertex)
    {
        if (v == nm_vertex)
        {
            v->setType(Vertex::INNER);
            v->removeAdjacentFacet(facet);
            continue; 
        }
        if (v->adjacentFacets().size() == 1)
        {
            // Orphan vertex.
            //v->setType(Vertex::ORPHAN);
            #ifdef _DEBUG
            if (v->getType() != Vertex::FRONT)
            {
                std::cerr << "\033[1;31mError: Vertex is not front\033[0m" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            #endif
            exileVertex(v);
            continue;
        }
        if (v->adjacentFacets().size() > 1)
        {
            // Inner vertex.
            v->setType(Vertex::FRONT);
            v->removeAdjacentFacet(facet);
            continue;
        }
    }
    
    m_facets.remove(facet);
    m_nfacets--;
    delete facet;
}

void Mesher::addVertex(Vertex *v)
{
    if (v->index() < -1)
    {
        std::cerr << "Vertex index is less than -1" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (v->index() != -1)
        return;

    bool is_empty = m_recycle_vertices_idx.empty();
    if (is_empty)
    {
        v->setIndex(m_vertice_idx);
        m_vertices.push_back(v);
        m_vertice_idx++;
    }
    else
    {
        unsigned int idx = *(m_recycle_vertices_idx.begin());
        m_recycle_vertices_idx.erase(idx);

        v->setIndex(idx);
        m_vertices.push_back(v);
    }
}

std::list<Vertex *>::const_iterator Mesher::vertices_begin() const
{
    return m_vertices.begin();
}

std::list<Vertex *>::const_iterator Mesher::vertices_end() const
{
    return m_vertices.end();
}
std::list<Facet *>::const_iterator Mesher::facets_begin() const
{
    return m_facets.begin();
}

std::list<Facet *>::const_iterator Mesher::facets_end() const
{
    return m_facets.end();
}

void Mesher::fillHoles()
{
    Edge_star_iterator ei = m_border_edges.begin();
    while (ei != m_border_edges.end())
    {
        // during the filling process border edges become inner edges
        // hence the following check
        if ((*ei)->getType() != Edge::BORDER)
        {
            // This means that a hole has been filled by previous boundary edges.
            ei = m_border_edges.erase(ei);
            continue;
        }

        Vertex *src = (*ei)->getSource();
        Vertex *tgt = (*ei)->getTarget();

        Vertex *v = src->findBorder(tgt);

        // if no oriented border links tgt to src (order is important since
        // edges of the front are oriented consistently all over the front)
        if (v == NULL)
        {
            ++ei;
            continue;
        }

        // computer ball center using order of vertices
        Point center;
        computeBallCenterUsingOrderOfVertices(*tgt, *src, *v, center);
        // The new facet should be from target to source to mimic the half-edge data structure.
        
        Facet *f = this->createFacet(tgt, src, v, center);
        addFacet(f);

        ei = m_border_edges.erase(ei);
    }
}

void Mesher::findSeedTriangle(OctreeNodeV *containment_node, OctreeNodeV *node,
                              double d, bool &found)
{
    if (node->getDepth() != 0)
    {
        for (unsigned int i = 0; i < 8; i++)
        {
            if (node->getChild(i) != NULL)
                findSeedTriangle(containment_node, node->getChild(i), d, found);
        }
    }
    else if (node->getNpts() != 0)
    {
        // Vertex_list::iterator pi = node->points_begin();
        Vertex_UnOrdSet::const_iterator pi = node->points_begin();
        while (pi != node->points_end())
        {
            Vertex *v = *pi;
            if (v->getType() == Vertex::FRONT) // 1
            {
                Edge_set &edges = v->adjacentEdges();
                Edge_set::iterator ei;
                for (ei = edges.begin(); ei != edges.end(); ++ei)
                {                                        // added the parantheses for compiler warning
                    if ((*ei)->getType() == Edge::FRONT) // 1
                        m_edge_front.push_front(*ei);
                    expandTriangulationAroundNode(containment_node, d);
                } // added the parantheses for compiler warning
            }
            else if (v->getType() == Vertex::ORPHAN) // 0
            {
                if (trySeed(*v, containment_node, d))
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

bool Mesher::trySeed(Vertex &v, OctreeNodeV *containment_node, double d)
{

    Neighbor_star_map neighbors;
    m_iterator_vertices->setR(2.0 * m_ball_radius);
    m_iterator_vertices->getSortedNeighbors(v, neighbors);
    m_iterator_vertices->setR(m_ball_radius);

    if (neighbors.size() < 3)
        return false;

    Neighbor_iterator ni = neighbors.begin();
    while (ni != neighbors.end())
    {
        Vertex &vtest = *(ni->second);
        if ((vtest.getType() != Vertex::ORPHAN) || (&vtest == &v) // 0
            || (!containment_node->isInside(vtest, d)))
        {
            ++ni;
            continue;
        }

        Neighbor_iterator nj = ni;
        ++nj;

        Vertex *candidate = NULL;
        Point center;
        bool changeHandness = false; // just add it now for the program to run. Have to check if it works.
        while (nj != neighbors.end())
        {
            if (tryTriangleSeed(&v, &vtest, nj->second, neighbors, center, changeHandness))
            {
                candidate = nj->second;
                break;
            }
            ++nj;
        }

        // TODO: Use info from Handness further.

        if (candidate == NULL)
        {
            Edge *e = v.getLinkingEdge(&vtest);
            if ((e != NULL) && (e->getType() == Edge::FRONT)) // 1
                m_edge_front.push_front(e);
        }
        else if (containment_node->isInside(*candidate, d))
        {
            Edge *e1 = v.getLinkingEdge(candidate);
            Edge *e2 = vtest.getLinkingEdge(candidate);
            Edge *e3 = v.getLinkingEdge(&vtest);

            if (((e1 != NULL) && (e1->getType() != Edge::FRONT))     // 1
                || ((e2 != NULL) && (e2->getType() != Edge::FRONT))  // 1
                || ((e3 != NULL) && (e3->getType() != Edge::FRONT))) // 1
            {
                ++ni;
                continue;
            }

            Facet *facet = this->createFacet(&v, &vtest, candidate, center);
            addFacet(facet);

            e1 = v.getLinkingEdge(candidate);
            e2 = vtest.getLinkingEdge(candidate);
            e3 = v.getLinkingEdge(&vtest);

            if (e1->getType() == Edge::FRONT) // 1
                m_edge_front.push_front(e1);
            if (e2->getType() == Edge::FRONT) // 1
                m_edge_front.push_front(e2);
            if (e3->getType() == Edge::FRONT) // 1
                m_edge_front.push_front(e3);

            if (m_edge_front.size() > 0)
                return true;
        }
        ++ni;
    }
    if (m_edge_front.size() > 0)
        return true;
    return false;
}

void Mesher::reconstructAroundNode(OctreeNodeV *containment_node, double d)
{
    if (!m_edge_front.empty())
        expandTriangulationAroundNode(containment_node, d);

    bool found = false;
    findSeedTriangle(containment_node, containment_node, d, found);
}

void Mesher::expandTriangulationAroundNode(OctreeNodeV *containment_node,
                                           double d)
{
    while (!m_edge_front.empty())
    {
        Edge *edge = m_edge_front.front();
        m_edge_front.pop_front();

        if (edge->getType() != Edge::FRONT)
            continue;

        Point center;
        Vertex *candidate = findCandidateVertex(edge, center);

        if ((candidate == NULL) || (candidate->getType() == Vertex::INNER) // 2
            || (!candidate->isCompatibleWith(*edge)))
        {
            edge->setType(Edge::BORDER);
            m_border_edges.push_back(edge);
            continue;
        }

        Edge *e1 = candidate->getLinkingEdge(edge->getSource());
        Edge *e2 = candidate->getLinkingEdge(edge->getTarget());

        if (((e1 != NULL) && (e1->getType() != Edge::FRONT)) || ((e2 != NULL) && (e2->getType() != Edge::FRONT)))
        {
            edge->setType(Edge::BORDER);
            m_border_edges.push_back(edge);
            continue;
        }
        // checking that the front remains inside the given node and a small
        //  band around it
        if (!containment_node->isInside(*candidate, d))
        {
            // edge->setType(1);
            edge->setType(Edge::FRONT);
            m_node_border_edges.push_back(edge);
            continue;
        }

        Facet *facet = this->createFacet(edge, candidate, center);
        addFacet(facet);

        e1 = candidate->getLinkingEdge(edge->getSource());
        e2 = candidate->getLinkingEdge(edge->getTarget());

        if (e1->getType() == Edge::FRONT)
            m_edge_front.push_front(e1);

        if (e2->getType() == Edge::FRONT)
            m_edge_front.push_front(e2);
    }
}

void Mesher::collectActiveEdges(OctreeNodeV *containment_node,
                                Edge_set &active_edges)
{
    if (containment_node->getDepth() != 0)
    {
        for (unsigned int i = 0; i < 8; ++i)
        {
            if (containment_node->getChild(i) != NULL)
                collectActiveEdges(containment_node->getChild(i), active_edges);
        }
    }
    else
    {
        // Vertex_list::iterator vi;
        Vertex_UnOrdSet::const_iterator vi;
        for (vi = containment_node->points_begin();
             vi != containment_node->points_end(); ++vi)
        {
            Vertex *v = *vi;
            if (v->getType() != Vertex::FRONT) // 1
                continue;

            Edge_set &edges = v->adjacentEdges();
            Edge_set::iterator ei;
            for (ei = edges.begin(); ei != edges.end(); ++ei)
            {
                Edge *e = *ei;
                if (e->getType() == Edge::FRONT)
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

void Mesher::collectBorderEdges(OctreeNodeV *containment_node,
                                Edge_set &border_edges)
{
    if (containment_node->getDepth() != 0)
    {
        for (unsigned int i = 0; i < 8; ++i)
        {
            if (containment_node->getChild(i) != NULL)
                collectBorderEdges(containment_node->getChild(i), border_edges);
        }
    }
    else
    {
        // Vertex_list::iterator vi;
        Vertex_UnOrdSet::const_iterator vi;
        for (vi = containment_node->points_begin();
             vi != containment_node->points_end(); ++vi)
        {
            Vertex *v = *vi;
            if (v->getType() != Vertex::FRONT) // 1
                continue;

            Edge_set &edges = v->adjacentEdges();
            Edge_set::iterator ei;
            for (ei = edges.begin(); ei != edges.end(); ++ei)
            {
                Edge *e = *ei;
                if (e->getType() == Edge::BORDER)
                    border_edges.insert(*ei);
            }
        }
    }
}

std::set<Facet *> &Mesher::getBoundaryFacets() const
{
    std::set<Facet *> *boundary_facets = new std::set<Facet *>();
    // auto boundary_facets = std::make_unique<std::set<Facet*>>(); // Smart pointer to manage the memory, please use #include <memory>
    for (const auto &edge : m_border_edges)
    {
        Vertex *v1 = edge->getSource();
        Vertex *v2 = edge->getTarget();

        Facet_set facets_set1 = v1->adjacentFacets();
        Facet_set facets_set2 = v2->adjacentFacets();
        for (const auto &facet : facets_set1)
        {
            boundary_facets->insert(facet);
        }
        for (const auto &facet : facets_set2)
        {
            boundary_facets->insert(facet);
        }
    }
    std::cout << "Address of boundary_facets in getBoundaryFacets: " << boundary_facets << std::endl;
    return *boundary_facets;
}

void Mesher::removeOrphanAndUpdate(TOctreeNode<Vertex> *node)
{
    Vertex_UnOrdSet &points = node->GetPoints();

    std::list<Vertex *> temporary_orphan_vertices;
    for (auto iter = points.begin(); iter != points.end(); ++iter)
    {
        Vertex *v = *iter;
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
        m_vertices.remove(v);
        // Retrieve the index of the vertex.
        #ifdef _DEBUG
        if (v->index() != -1)
        {
            std::cerr << "Error:Orphan Vertex index is not -1!" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        #endif
        // Delete the vertex.
        delete v;
    }
}

void Mesher::exileVertex(Vertex *vertex)
{
    m_recycle_vertices_idx.insert(vertex->index());
    m_vertices.remove(vertex);

    // clear the vertex
    vertex->setType(Vertex::VertexType::ORPHAN);
    vertex->setIndex(-1);
    vertex->clearAdjacentEdgesAndFacets();
}

void Mesher::removeFacet(Facet *facet)
{
    Vertex *vertex[3];
    for (int i = 0; i < 3; ++i)
    {
        vertex[i] = facet->getVertex(i);
        if (vertex[i] == nullptr) {
            std::cerr << "Error: Invalid vertex pointer at index " << i << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    // Deal with the edges first.
    for (int i = 0; i < 3; ++i)
    {
        int source_idx = i;
        int target_idx = (i + 1) % 3;
        Edge *e = vertex[source_idx]->getLinkingEdge(vertex[target_idx]);

        bool doesFacet1Exist = (e->getFacet1() != NULL);
        bool doesFacet2Exist = (e->getFacet2() != NULL);

        if (doesFacet1Exist == true && doesFacet2Exist == true)
        {
            // In this case, the edge is shared by two facets.
            e->removeAdjacentFacet(facet);
            e->setType(Edge::EdgeType::BORDER);
            m_border_edges.push_back(e);
            continue;
        }
        if (doesFacet1Exist != doesFacet2Exist)
        {
            vertex[source_idx]->removeAdjacentEdge(e);
            vertex[target_idx]->removeAdjacentEdge(e);
            m_border_edges.remove(e);
            delete e;
            e = NULL;
            continue;
        }
        #ifdef _DEBUG
        if (doesFacet1Exist == false && doesFacet2Exist == false)
        {
            std::cerr << "Error: Edge is not shared by any facets!" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        #endif
    }

    // Deal with the vertices.
    // This function does not delete the vertex.
    for (unsigned int i = 0; i < 3; ++i)
    {
        vertex[i]->removeAdjacentFacet(facet);
        bool vertex_still_has_adjacent_facets = (vertex[i]->adjacentFacets().size() != 0);
        if (vertex_still_has_adjacent_facets)
        {
            vertex[i]->setType(Vertex::VertexType::FRONT);
        }
        else // The vertex is not shared by any other facets.
        {
            exileVertex(vertex[i]);
        }
    }
    delete facet;
}

void Mesher::removeFacets(std::set<Facet *> &facets)
{
    for (auto facet : facets)
    {
        removeFacet(facet);
        m_facets.remove(facet);
        m_fresh_facets.erase(facet); // It erase if exists
        m_nfacets--;
    }
}

void Mesher::removeFacets(std::unordered_set<Facet *> &facets)
{
    for (auto facet : facets)
    {
        removeFacet(facet);
        m_facets.remove(facet);
        m_fresh_facets.erase(facet); // It erase if exists
        m_nfacets--;
    }
}

const std::list<Facet *> &Mesher::getFacets() const
{
    return m_facets;
}

Edge_star_list Mesher::getBorderEdges() const
{
    return m_border_edges;
}

bool sameOrientation(int i, Facet *query_facet)
{

    Vertex *queryV_Source = query_facet->getVertex(i);
    Vertex *queryV_Target = query_facet->getVertex(i + 1);

    Edge *e0 = queryV_Source->getLinkingEdge(queryV_Target);
    Facet *facet_1, *facet_2;
    facet_1 = e0->getFacet1();
    facet_2 = e0->getFacet2();

    Facet *adjacent_facet = (facet_1 == query_facet) ? facet_2 : facet_1;
    if (adjacent_facet == NULL)
    {
        return true;
    }
    else
    {
        for (int k = 0; k < 3; k++)
        {
            Vertex *adjV_source = adjacent_facet->getVertex(k);
            Vertex *adjV_target = adjacent_facet->getVertex(k + 1);
            if (adjV_source == queryV_Target && adjV_target == queryV_Source)
            {
                return true;
            }
        }
        std::cout << " Adress of query facet: " << query_facet << std::endl;
        std::cout << " Adress of queryV_Source: " << queryV_Source << std::endl;
        std::cout << " Adress of queryV_Target: " << queryV_Target << std::endl;
        std::cout << " Adress of adjacent_facet: " << adjacent_facet << std::endl;
        std::cout << " Adress of adjV_1: " << adjacent_facet->getVertex(0) << std::endl;
        std::cout << " Adress of adjV_2: " << adjacent_facet->getVertex(1) << std::endl;
        std::cout << " Adress of adjV_3: " << adjacent_facet->getVertex(2) << std::endl;

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

void Mesher::print_stats()
{
    std::cout << ">>>>>>>>" << std::endl;
    std::cout << "Reconstructed mesh: " << this->nVertices()
              << " vertices; " << this->nFacets() << " facets. ";
    std::cout << this->nBorderEdges() << " border edges" << std::endl;
    std::cout << "<<<<<<<<" << std::endl;
}

Facet_set Mesher::computeCollisionFacets(const std::list<Vertex> &vertices)
{
    Facet_set collision_facets;
    m_octree_ball_centers_iterator->setDepth(m_octree_ball_centers->getDepth());
    for (auto &vertex : vertices)
    {
        // Check if the vertex is in side the any ball
        // Point point = Point(vertex.x(), vertex.y(), vertex.z());
        std::map<double, BallCenter *> neighbors; // neighbor.first is the squared distance

        // unsigned int num_neighbors = octree_ball_centers_iterator.getSortedNeighbors(vertex, neighbors);
        m_octree_ball_centers_iterator->getSortedNeighbors(vertex, neighbors);

        // print the size of neighbors
        if (neighbors.size() == 0)
            continue;

        // if any squared distance is less than the squared radius, the vertex is inside a ball
        for (auto &neighbor : neighbors)
        {
            if (neighbor.first < m_sq_ball_radius && neighbor.second != nullptr)
            {
                BallCenter *ball_center = neighbor.second;
                Facet *facet = ball_center->getFacet();
                if (facet != nullptr)
                {
                    collision_facets.insert(facet);
                    // std::cout << "Address of facet: " << facet << std::endl;
                }
                else
                {
                    std::cerr << "Error: The facet is nullptr" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
            }
        }
    }
    return collision_facets;
}

void Mesher::clearOrphanVertices()
{
    m_iterator_vertices->loopOverAllNodes([this](TOctreeNode<Vertex> *node)
                                          { this->removeOrphanAndUpdate(node); });
}

std::vector<ColorVertex> Mesher::getVerticesToRender() const
{
    // 1. dubug vertices
    std::vector<ColorVertex> color_vertices;
    color_vertices.reserve(m_debug_render_vertices.size());
    Eigen::Vector3d green_color(0.0, 1.0, 0.0);

    for (auto vertex : m_debug_render_vertices) {
        color_vertices.push_back(ColorVertex(vertex, green_color));
    }
    return color_vertices;
}

std::vector<ColorFacet> Mesher::getFacetsToRender() const
{
    std::unordered_set<ColorFacet> facets_to_render;

    // 1. m_facets
    Eigen::Vector3d gray_color(0.7, 0.7, 0.7); // it is gray
    for (auto facet : m_facets) {
        facets_to_render.insert(ColorFacet(facet, gray_color));
    }
    
    // 2. m_debug_facets
    Eigen::Vector3d blue_color(0.0, 0.0, 1.0);
    for (auto facet : m_debug_render_facets) {
      // Erase any existing ColorFacet with this facet pointer, regardless of color
      for (auto it = facets_to_render.begin(); it != facets_to_render.end();) {
          if (it->facet == facet) {
              it = facets_to_render.erase(it);
          } else {
              ++it;
          }
      }
      // Insert with blue color
      facets_to_render.insert(ColorFacet(facet, blue_color));
    }

    std::vector<ColorFacet> color_facets(facets_to_render.begin(), facets_to_render.end());
    return color_facets;
}


void Mesher::expandOctree(const std::list<Vertex> &vertices)
{
    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Expand the ballcenters octree
    // std::cout << "Vertices size: " << vertices.size() << std::endl;
    std::cout << "Expanding octree ball centers" << std::endl;
    for (auto &vertex : vertices)
    {
        BallCenter ball_center(vertex, nullptr);
        m_octree_ball_centers->checkSizeAndexpand(ball_center);
    }
    std::cout << "Expanding octree vertices" << std::endl;
    for (auto &vertex : vertices)
    {
        m_octree_vertices->checkSizeAndexpand(vertex);
    }
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
}

void Mesher::checkAndRemoveCollisionFacets(const std::list<Vertex> &vertices)
{
    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Collection of collision facets
    Facet_set collision_facets = this->computeCollisionFacets(vertices);

    {
        std::lock_guard<std::mutex> lock(*visualization_mutex);
        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // Remove the collision facets
        this->removeFacets(collision_facets); // This function is very wrong
        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    }

}

void Mesher::addPointsToOctreeVertices(const std::list<Vertex> &vertices)
{
    for (auto &vertex : vertices)
    {
        m_octree_vertices->addPoint(vertex);
    }
}

void Mesher::resetBoundaryEdges()
{
    for (Edge *e : m_border_edges)
    {
        e->setType(Edge::FRONT);
        m_edge_front.push_back(e);
    }
    m_border_edges.clear();

    #ifdef _DEBUG
        if (m_edge_front.size() == 0)
        {
            std::cerr << "\033[1;31mWarning: No front edges found after further reconstruction. This is ok if this is the first batch.\033[0m" << std::endl;
        }
    #endif
}

void Mesher::batchReconstruct(const std::list<Vertex> &vertices)
{
    // Asumming radius is already set.

    // Expand the octree
    this->expandOctree(vertices);
    
    // Check and remove collision facets
    this->checkAndRemoveCollisionFacets(vertices);

    // Add points to the octree
    this->addPointsToOctreeVertices(vertices);

    // Reset boundary edges to edge_front
    this->resetBoundaryEdges();
    
    // Further reconstruction
    this->reconstruct();

    // Fill holes
    //this->fillHoles();

    std::cout << "\033[32mFresh facets size: " << this->m_fresh_facets.size() << "\033[0m" << std::endl;

    while (true) {
        // Remove singular vertices
        
        std::cout << "Removing fan fan singular vertices" << std::endl;
        bool found_fan_fan_vertex = this->removeFanFanSingular();
        std::cout << "Finished removing fan fan singular vertices" << std::endl;
 
        // Remove disk singular verticesmesher.batchReconstructNew(vertices);
        std::cout << "Removing disk fan singular vertices" << std::endl;
        bool found_disk_fan_vertex = this->removeDiskFanSingular();
        std::cout << "Finished removing disk fan singular vertices" << std::endl;

        if (!found_fan_fan_vertex && !found_disk_fan_vertex) {
            break;
        }
    }
    
    // Clear fresh facets
    // Print total new fresh facets
    std::cout << "\033[32mTotal new fresh facets: " << this->m_fresh_facets.size() << "\033[0m" << std::endl;
    this->clearFreshFacets();

    // Clear debug render facets
    this->clearDebugRender();

    // Clear orphan vertices
    this->clearOrphanVertices();

}

bool Mesher::removeFanFanSingular()
{
// If edgefront is not empty, return error
#ifdef _DEBUG
    if (m_edge_front.size() != 0)
    {
        std::cerr << "Error: Edgefront is not empty" << std::endl;
        std::exit(EXIT_FAILURE);
    }
#endif

    bool found_fan_fan_vertex = false;

    while (true)
    {
        std::unordered_map<Vertex *, unsigned int> vertex_counter;
        for (Edge *e : this->m_border_edges)
        {
            vertex_counter.insert({e->getSource(), 0});
            vertex_counter.insert({e->getTarget(), 0});
        }

        for (Edge *e : this->m_border_edges)
        {
            vertex_counter[e->getSource()]++;
            vertex_counter[e->getTarget()]++;
        }

        std::unordered_set<Vertex *> singular_vertices;
        for (auto &pair : vertex_counter)
        {
            if (pair.second > 2)
            {
                singular_vertices.insert(pair.first);
            }
        }

        if (singular_vertices.size() == 0)
        {
            break;
        }
        else
        {
            std::cout << "Fan Fan vertices found: " << singular_vertices.size() << std::endl;
            found_fan_fan_vertex = true;
        }

        // collect the facets that are adjacent to the singular vertices
        std::set<Facet *> singular_facets;
        for (Vertex *v : singular_vertices)
        {
            for (Facet *f : v->adjacentFacets())
            {
                singular_facets.insert(f);
            }
        }

        // remove the singular facets
        {
            std::lock_guard<std::mutex> lock(*visualization_mutex);
            this->removeFacets(singular_facets);

            for (auto* facet : m_facets) {
                Vertex* v1 = facet->getVertex(0);
                Vertex* v2 = facet->getVertex(1);
                Vertex* v3 = facet->getVertex(2);
                if (v1 == nullptr || v2 == nullptr || v3 == nullptr) {
                    std::cout << "Facet address: " << facet << std::endl;
                }
            }

            for (auto* facet : m_debug_render_facets) {
                Vertex* v1 = facet->getVertex(0);
                Vertex* v2 = facet->getVertex(1);
                Vertex* v3 = facet->getVertex(2);
                if (v1 == nullptr || v2 == nullptr || v3 == nullptr) {
                    std::cout << "Debug render facet address: " << facet << std::endl;
                }
            }
        }
    }
    return found_fan_fan_vertex;
}

bool Mesher::removeDiskFanSingular()
{

    bool found_disk_fan_vertex = false;

    // All boundary vertices are relevant
    std::set<Vertex *> boundary_vertices;
    for (Edge *e : m_border_edges) {
        boundary_vertices.insert(e->getSource());
        boundary_vertices.insert(e->getTarget());
    }

    // Find all relevant vertices  
    std::map<Vertex *, unsigned int> fresh_vertices_counter;
    for (Facet *f : m_fresh_facets) {
        for (int i = 0; i < 3; i++) {
            if (boundary_vertices.find(f->getVertex(i)) == boundary_vertices.end()) {
                // not a boundary vertex, skip
                continue;
            }
            // the key does not exist, then insert it with 1
            if (fresh_vertices_counter.find(f->getVertex(i)) == fresh_vertices_counter.end()) {
                fresh_vertices_counter[f->getVertex(i)] = 1;
            }
            else {
                fresh_vertices_counter[f->getVertex(i)]++;
            }
        }
    }

    // Get all the disk fan vertices
    std::list<VertexDiskFanInfo> disk_fan_vertices; 
    for (auto &[v, count] : fresh_vertices_counter) {
        if (count > 1) { 
            VertexDiskFanInfo vertex_disk_fan_info = this->getDiskFan(v);
            disk_fan_vertices.push_back(vertex_disk_fan_info);
        }
    }
    
    // Add defug print
    for (VertexDiskFanInfo &vertex_disk_fan_info : disk_fan_vertices) {
        if (vertex_disk_fan_info.disk_facets.size() > 0) {
            for (Facet_set &facet_set : vertex_disk_fan_info.disk_facets) {
                for (Facet *facet : facet_set) {
                    m_debug_render_facets.push_back(facet);
                }
            }
            std::cout << "Disk fan vertex: " << vertex_disk_fan_info.disk_fan_vertex->index() << std::endl;
        }
    }

    {
        std::lock_guard<std::mutex> lock(*visualization_mutex);
        // Remove the redundant facets
        for (VertexDiskFanInfo &vertex_disk_fan_info : disk_fan_vertices) {
            Vertex *nm_vertex = vertex_disk_fan_info.disk_fan_vertex;
            if (vertex_disk_fan_info.disk_facets.size() == 0) {
                continue;
            }
            if (vertex_disk_fan_info.disk_facets.size() == 1) {
                for (Facet_set &fan_facets : vertex_disk_fan_info.fan_facets) {
                    found_disk_fan_vertex = true;
                    for (Facet *facet : fan_facets) {
                        this->removeRedundantDiskFanFacet(nm_vertex, facet);
                    }
                }
            }

            #ifdef _DEBUG
                if (vertex_disk_fan_info.disk_facets.size() >= 2) {
                    std::cerr << "Disk facet set size: " << vertex_disk_fan_info.disk_facets.size() << std::endl;
                    std::exit(EXIT_FAILURE);
                }
            #endif
        }
    }

    return found_disk_fan_vertex;
}


void Mesher::mesh_integrityCheck() const
{
    // make a share object of o3d_mesh
    std::shared_ptr<open3d::geometry::TriangleMesh> o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();

    std::vector<ColorFacet> color_facets = this->getFacetsToRender();

    Visualizer::renderFacets(color_facets, o3d_mesh);

    // is it vertex manifold?
    bool is_vertex_manifold = o3d_mesh->IsVertexManifold();
    if (!is_vertex_manifold) {
        std::cerr << "Error: The mesh is not vertex manifold" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // check o3d_mesh properties
    // is it edge manifold?
    bool allow_boundary_edges = true;
    bool is_edge_manifold = o3d_mesh->IsEdgeManifold(allow_boundary_edges);
    if (!is_edge_manifold) {
        //get non-manifold edges
        std::vector<Eigen::Vector2i> non_manifold_edges = o3d_mesh->GetNonManifoldEdges(allow_boundary_edges);
        // print non-manifold edges 
        std::cout << "Non-manifold edges: " << non_manifold_edges.size() << std::endl;
        // print top 10 non-manifold edges
        for (size_t i = 0; i < non_manifold_edges.size(); i++) {
            std::cout << "Non-manifold edge " << i << ": " << non_manifold_edges[i][0] << " " << non_manifold_edges[i][1] << std::endl;
        }
        std::cerr << "Error: The mesh is not edge manifold" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // is it orientable?
    bool is_orientable = o3d_mesh->IsOrientable();
    if (!is_orientable) {
        std::cerr << "Error: The mesh is not orientable" << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

Facet* Mesher::createFacet(Vertex* v1, Vertex* v2, Vertex* v3, const Point& center)
{
    std::lock_guard<std::mutex> lock(*visualization_mutex);
    return new Facet(v1, v2, v3, center);
}

Facet* Mesher::createFacet(Edge* edge, Vertex* vertex, const Point& center)
{
    std::lock_guard<std::mutex> lock(*visualization_mutex);
    return new Facet(edge, vertex, center);
}
