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
#include "Region.h"

const double PI = 3.1415926535;

Mesher::Mesher()
{
    m_octree_vertices = nullptr;
    m_iterator_vertices = nullptr;
    m_vertice_idx = 1;
    m_facet_idx = 1;
    //m_recycle_vertices_idx = std::unordered_set<unsigned int>();
    m_octree_ball_centers = nullptr;
    m_octree_ball_centers_iterator = nullptr;
}

Mesher::Mesher(OctreeVertices *octree_vertices, OctreeIteratorVertices *iterator_vertices,
               OctreeBallCenters *octree_ball_centers, OctreeIteratorBallCenters *octree_ball_centers_iterator,
               const bool &random_device, const int &seed, const bool &seed_triangles_every_batch)
{
    m_ball_radius = iterator_vertices->getR();
    m_sq_ball_radius = m_ball_radius * m_ball_radius;
    m_vertice_idx = 1;
    m_facet_idx = 1;
    //m_recycle_vertices_idx = std::unordered_set<unsigned int>();
    m_octree_vertices = octree_vertices;
    m_iterator_vertices = iterator_vertices;
    m_octree_ball_centers = octree_ball_centers;
    m_octree_ball_centers_iterator = octree_ball_centers_iterator;

    m_random_device = random_device;
    m_seed = seed;
    m_seed_triangles_every_batch = seed_triangles_every_batch;
}

void Mesher::initialize(const Point &origin, double size, unsigned int depth)
{
    double ball_radius = size / ((double)pow2(depth));

    m_octree_vertices->setDepth(depth);
    m_octree_vertices->initialize(origin, size);
    m_iterator_vertices->setR(ball_radius);

    m_octree_ball_centers->setDepth(depth);
    m_octree_ball_centers->initialize(origin, size);
    m_octree_ball_centers_iterator->setR(ball_radius);

    m_ball_radius = ball_radius;
    m_sq_ball_radius = ball_radius * ball_radius;
}

Mesher::~Mesher()
{
    #ifdef _DEBUG
    std::cout << "Mesher destructor is called" << std::endl;
    #endif
    m_octree_vertices = nullptr;
    m_iterator_vertices = nullptr;
    m_octree_ball_centers = nullptr;
    m_octree_ball_centers_iterator = nullptr;

    m_edge_front.clear();
    m_border_edges.clear();

    Facet_star_list facets_to_delete = m_facets;

    Facet_star_list::iterator fi;
    for (fi = facets_to_delete.begin(); fi != facets_to_delete.end(); ++fi)
    {
        // delete *fi;
        removeFacet(*fi);
        *fi = nullptr;
    }
    facets_to_delete.clear();

    m_vertice_idx = 1;
    m_facet_idx = 1;
}

void Mesher::setBallRadius(double r)
{
    m_ball_radius = r;
    m_sq_ball_radius = r * r;
}

unsigned int Mesher::nFacets() const
{
    return static_cast<unsigned int>(m_facets.size());
}

unsigned int Mesher::nBorderEdges() const
{
    return (unsigned int)m_border_edges.size();
}

void Mesher::reconstruct()
{
    #ifdef _DEBUG
    std::cout << "***********Ball radius " << m_ball_radius
              << " ***********" << std::endl;
    #endif

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
        #ifdef _DEBUG
        std::cout << "Edge front not empty, Expanding triangulation" << std::endl;
        #endif
        expandTriangulation();
    }

    if (m_seed_triangles_every_batch)
    {
        findSeedTriangle(); // This is to find the seed triangle again.
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
            if (node->getChild(i) != nullptr)
                findSeedTriangle(node->getChild(i), found);
        }
    }
    else if (node->getNpts() != 0)
    {
        // Vertex_list::iterator pi = node->points_begin();
        typename Vertex_UnOrdSet::const_iterator pi = node->points_begin();
        while (pi != node->points_end())
        {
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

        Vertex *candidate = nullptr;
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

        if (candidate != nullptr)
        {
            // <<<
            // To check any of the edges are not a front edge, we can use the getLinkingEdge method
            Edge *e1 = v.getLinkingEdge(candidate);
            Edge *e2 = vtest.getLinkingEdge(candidate);
            Edge *e3 = v.getLinkingEdge(&vtest);

            if (((e1 != nullptr) && (e1->getType() != Edge::FRONT)) || ((e2 != nullptr) && (e2->getType() != Edge::FRONT)) || ((e3 != nullptr) && (e3->getType() != Edge::FRONT)))
            {
                ++ni;
                continue;
            }
            // >>>
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
            this->createAndAddFacet(v0, v1, v2, center);
            // >>>

            e1 = v.getLinkingEdge(candidate);
            e2 = vtest.getLinkingEdge(candidate);
            e3 = v.getLinkingEdge(&vtest);

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
    if (((e1 != nullptr) && (e1->getType() == Edge::INNER)) || ((e2 != nullptr) && (e2->getType() == Edge::INNER)))
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
        if (dist2(center, *v) < (m_sq_ball_radius - 1e-16))
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
    double sq_circumradius;

    double sq_circumradius_numerator = a * b * c;

    a = std::sqrt(a);
    b = std::sqrt(b);
    c = std::sqrt(c);

    double sq_circumradius_denominator = (a + b + c) * (b + c - a) * (c + a - b) * (a + b - c);

    sq_circumradius = sq_circumradius_numerator / sq_circumradius_denominator;

    // compute the ortogonal distance from the hypothetic center to the triangle
    double height = m_sq_ball_radius - sq_circumradius;

    if (sq_circumradius < 0 || sq_circumradius > m_sq_ball_radius) {
        //std::cout << "\033[1;31msq_circumradius: " << sq_circumradius << "\033[0m" << std::endl;
        return false;
    }

    // compute the normal of the three points
    double nx, ny, nz = 0;

    if (height >= 0.0)
    {
        computeNormal(v1, v2, v3, nx, ny, nz);
        height = sqrt(height);
        center = Point(x + height * nx, y + height * ny, z + height * nz);
        return true;
    }
    else
    {
        std::cout << "Height is negative" << std::endl;
        return false;
    }
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

ReconstructionType Mesher::computeReconstructionType(
    const Edge *eSource,
    const Edge *eTarget,
    const Vertex *candidate) const
{
    // We can already assume the candidate is not INNER vertex.
    bool doesEdgeFromSourceExist = eSource != nullptr;
    bool doesEdgeFromTargetExist = eTarget != nullptr;
    bool isCandidateORPHAN = candidate->getType() == Vertex::ORPHAN;

    // If both e1 and e2 do not exist, then the reconstruction type is expansion.
    if (!doesEdgeFromSourceExist && !doesEdgeFromTargetExist && isCandidateORPHAN)
    {
        return ReconstructionType::EXPANSION;
    }

    // If both e1 and e2 do not exist, then the reconstruction type is glue.
    if (!doesEdgeFromSourceExist && !doesEdgeFromTargetExist && !isCandidateORPHAN)
    {
        return ReconstructionType::GLUE;
    }

    // If e1 or e2 is not a front edge, then no reconstruction is needed.
    bool isESourceNotFront = doesEdgeFromSourceExist && (eSource->getType() != Edge::FRONT);
    bool isETargetNotFront = doesEdgeFromTargetExist && (eTarget->getType() != Edge::FRONT);

    if (isESourceNotFront || isETargetNotFront)
    {
        return ReconstructionType::NO_RECONSTRUCTION;
    }

    bool isESourceFront = doesEdgeFromSourceExist && (eSource->getType() == Edge::FRONT);
    bool isETargetFront = doesEdgeFromTargetExist && (eTarget->getType() == Edge::FRONT);

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

static bool isGoodOrentationHoleFilling(Edge* edge1, Edge* edge2, Edge* edge3)
{
    Vertex* v1Source = edge1->getSource();
    Vertex* v1Target = edge1->getTarget();

    Vertex* v2Source = edge2->getSource();
    Vertex* v2Target = edge2->getTarget();

    Vertex* v3Source = edge3->getSource();
    Vertex* v3Target = edge3->getTarget();

    bool loop;
    if (v1Target == v3Source && v3Target == v2Source && v2Target == v1Source)
    {
        loop = true;
    }
    else
    {
        loop = false;
    }
    return loop;
}

void Mesher::expandTriangulation()
{
    while (!m_edge_front.empty())
    {
        Edge *edge = m_edge_front.front();
        m_edge_front.pop_front();

        if (edge->getType() != Edge::FRONT)
            continue;

#ifdef _DEBUG
        // Check for invalid edge facet configuration
        if (edge->getFacet2() != nullptr && edge->getFacet1() == nullptr) {
            std::cerr << "\033[1;31mError: Edge has facet2 but no facet1\033[0m" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        bool isFacet1Exist = edge->getFacet1() != nullptr;
        bool isFacet2Exist = edge->getFacet2() != nullptr;

        if (isFacet1Exist && !(isFacet2Exist))
        {
            // This is good.
        }
        else
        {
            std::cerr << "\033[1;31mError: Edge has no facet1 or facet2\033[0m" << std::endl;
            std::exit(EXIT_FAILURE);
        }
#endif

        bool isFlipped = edge->aligningWithFacet1();
        if (isFlipped)
        {
            std::cout << "Flipped orientation of edge to align with facet 1" << std::endl;
        }

        Point center;
        Vertex *candidate = findCandidateVertex(edge, center);

        if ((candidate == nullptr) || (candidate->getType() == Vertex::INNER) // 2
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

        // Check for invalid edge facet configuration
        //if (eSource == nullptr || eTarget == nullptr) {
        //    std::cerr << "\033[1;31mError: Edge has no source or target linking edge\033[0m" << std::endl;
        //    std::exit(EXIT_FAILURE);
        //}

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
        if (reconstructionType == ReconstructionType::HOLE_FILLING)
        {
            if (!isGoodOrentationHoleFilling(edge, eSource, eTarget))
            {
                std::cout << "Edge is not good orentation for hole filling" << std::endl;
                edge->setType(Edge::BORDER);
                m_border_edges.push_back(edge);
                continue;
            }
        }

        this->createAndAddFacet(edge, candidate, center);

        Edge *e1 = candidate->getLinkingEdge(edge->getSource());
        Edge *e2 = candidate->getLinkingEdge(edge->getTarget());

        if (e1->getType() == Edge::FRONT)
            m_edge_front.push_front(e1);

        if (e2->getType() == Edge::FRONT)
            m_edge_front.push_front(e2);

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

    Vertex *candidate = nullptr;
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
    // if(candidate != nullptr && candidate->getType() == 2)
    //     std::cout << "!!!!!!!!   candidate type: " << candidate->getType() << std::endl;
    return candidate;
}

void Mesher::addFacet(Facet *f)
{
    addVertex(f->vertex(0));
    addVertex(f->vertex(1));
    addVertex(f->vertex(2));

    m_facets.push_back(f);
    addFreshFacet(f);

    // Update the ball centers octree.
    Point temp_ball_center = f->getBallCenter();
    BallCenter *ball_center = m_octree_ball_centers->checkSizeAndaddPoint(BallCenter(temp_ball_center, f));
    f->setBallCenterPtr(ball_center);

    m_batch_facets_added.insert(f);
}





void Mesher::addVertex(Vertex *v)
{
    if (v->index()  > 0)
        // The vertex is already in the map, return.
        return;
    else if (v->index() == 0)
    {
        v->setIndex(m_vertice_idx);
        m_vertice_idx++;
    }
    else
    {
        #ifdef _DEBUG
        std::cerr << "Error: Vertex index is not 0" << std::endl;
        std::cerr << "Vertex index: " << v->index() << std::endl;
        std::exit(EXIT_FAILURE);
        #endif
    }
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
        if (v == nullptr)
        {
            ++ei;
            continue;
        }

        Edge *edge = *ei;
        Edge *eSource = edge->getSource()->getLinkingEdge(v);
        Edge *eTarget = edge->getTarget()->getLinkingEdge(v);

        if (!isGoodOrentationHoleFilling(edge, eSource, eTarget))
        {
            ++ei;
            continue;
        }

        // computer ball center using order of vertices
        // Beware that the fill hole does not use the normal vector of the vertices.
        Point center;
        computeBallCenter(*tgt, *src, *v, center);

        this->createAndAddFacet(*ei, v, center);

        ei = m_border_edges.erase(ei);
    }
}


void Mesher::removeOrphanAndUpdate(TOctreeNode<Vertex> *node)
{
    Vertex_UnOrdSet &node_points = node->GetPoints();

    std::list<Vertex *> temporary_orphan_vertices;
    for (auto iter = node_points.begin(); iter != node_points.end(); ++iter)
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
        // Update the vertices set of the node.
        node_points.erase(v);
// Retrieve the index of the vertex.
#ifdef _DEBUG
        if (v->index() != 0)
        {
            std::cerr << "Error:Orphan Vertex index is not 0!" << std::endl;
            std::exit(EXIT_FAILURE);
        }
#endif
        // Delete the vertex.
        delete v;
    }
}

void Mesher::exileVertex(Vertex *vertex)
{
    // clear the vertex
    vertex->setType(Vertex::VertexType::ORPHAN);
    vertex->setIndex(0);
    vertex->clearAdjacentEdgesAndFacets();
}

template <bool exile = false>
void Mesher::removeFacet(Facet *facet)
{
    Vertex *vertex[3];
    for (int i = 0; i < 3; ++i)
    {
        vertex[i] = facet->getVertex(i);
        vertex[i]->removeAdjacentFacet(facet);
    }

    Edge *edge[3];
    for (int i = 0; i < 3; ++i)
    {
        edge[i] = vertex[i]->getLinkingEdge(vertex[(i + 1) % 3]);
        edge[i]->removeAdjacentFacet(facet);
    }

    // Update edges
    for (int i = 0; i < 3; ++i)
    { 
        bool edge_has_no_facet = (edge[i]->getFacet1() == nullptr && edge[i]->getFacet2() == nullptr);
        if (edge_has_no_facet) {
            Vertex *source_vertex = edge[i]->getSource();
            source_vertex->removeAdjacentEdge(edge[i]);

            Vertex *target_vertex = edge[i]->getTarget();
            target_vertex->removeAdjacentEdge(edge[i]);

            m_border_edges.remove(edge[i]);
            delete edge[i];
        }
        else
        {
            edge[i]->setType(Edge::EdgeType::BORDER);
            m_border_edges.push_back(edge[i]);
        }
    }
    

    // Deal with the vertices.
    for (Vertex *v : vertex)
    {
        v->updateType();
        if (v->adjacentFacets().size() == 0)
        {
            if constexpr (exile)
            {
                exileVertex(v);
            }
            else
            {
                v->remove_and_delete();
            }
        }
    }

    m_facets.remove(facet);
    m_fresh_facets.erase(facet);

    m_batch_facets_removed.insert(facet->getIndex());
    m_batch_facets_added.erase(facet);

    delete facet;
}

template <typename SetType, bool exile = false>
void Mesher::removeFacets(SetType &facets)
{
    // Perform compile time check
    static_assert(std::is_same_v<typename SetType::value_type, Facet*>,
                     "SetType must contain Facet pointers");

    for (auto facet : facets)
    {
        removeFacet<exile>(facet);
    }
}

const std::list<Facet *> &Mesher::getFacets() const
{
    return m_facets;
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
    if (adjacent_facet == nullptr)
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

void Mesher::debugPrintStats()
{
    std::cout << ">>>>>>>>" << std::endl;
    std::cout << "Reconstructed mesh: "<< this->nFacets() << " facets. ";
    std::cout << this->nBorderEdges() << " border edges" << std::endl;
    std::cout << "<<<<<<<<" << std::endl;
}

Facet_set Mesher::computeCollisionFacets(const std::list<Vertex *> &vertices)
{
    Facet_set collision_facets;
    for (Vertex *vertex : vertices)
    {
        // Check if the vertex is in side the any ball
        // Point point = Point(vertex.x(), vertex.y(), vertex.z());
        std::map<double, BallCenter *> neighbors; // neighbor.first is the squared distance
        //m_octree_ball_centers_iterator->setDepth(m_octree_ball_centers->getDepth());

        // unsigned int num_neighbors = octree_ball_centers_iterator.getSortedNeighbors(vertex, neighbors);
        m_octree_ball_centers_iterator->getSortedNeighbors(*vertex, neighbors);

    
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

void Mesher::expandOctree(const std::list<Vertex> &vertices)
{
    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Expand the ballcenters octree
    // std::cout << "Vertices size: " << vertices.size() << std::endl;
    for (auto &vertex : vertices)
    {
        BallCenter ball_center(vertex, nullptr);
        m_octree_ball_centers->checkSizeAndexpand(ball_center);
    }
    for (auto &vertex : vertices)
    {
        m_octree_vertices->checkSizeAndexpand(vertex);
    }
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
}

void Mesher::checkAndRemoveCollisionFacets(const std::list<Vertex*> &vertices)
{
    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Collection of collision facets
    Facet_set collision_facets = this->computeCollisionFacets(vertices);
    {
        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // Remove the collision facets
        this->removeFacets<Facet_set, true>(collision_facets); 
        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    }
}

void Mesher::addPointsToOctreeVertices(const std::list<Vertex> &vertices)
{
    for (auto &vertex : vertices)
    {
        Vertex *vertex_ptr = m_octree_vertices->addPoint(vertex);
        if (vertex_ptr != nullptr)
        {
            m_fresh_vertices.push_back(vertex_ptr);
        }
    }
}


void Mesher::clearFreshVertices()
{
    m_fresh_vertices.clear();
}

void Mesher::resetBoundaryEdges()
{
    for (Edge *e : m_border_edges)
    {
        e->setType(Edge::FRONT);
        m_edge_front.push_back(e);
    }
    m_border_edges.clear();
}

void Mesher::batchReconstruct(const std::list<Vertex> &vertices)
{
    // Asumming radius is already set.
    clearBatchFacets();

    // Expand the octree
    this->expandOctree(vertices);

    // Add points to the octree
    this->addPointsToOctreeVertices(vertices);

    // Downsample the octree
    std::list<Vertex*> recruited_points = this->m_iterator_vertices->downSample(m_random_device, m_seed);

    std::cout << "Recruited points size: " << recruited_points.size() << std::endl;

    //this->updateRender();

    // Check and remove collision facets
    this->checkAndRemoveCollisionFacets(recruited_points);

    // Removing collision facets may create fanfan singular
    // Therefore, we need to remove fanfan singular first
    while (true)
    {
        // Remove singular vertices
        bool found_fan_fan_vertex = this->removeFanFanSingular();
        if (!found_fan_fan_vertex)
        {
            break;
        }
    }
    //this->updateRender();
    
    // Reset boundary edges to edge_front
    this->resetBoundaryEdges();

    // Further reconstruction
    this->reconstruct();

    // Fill holes
    this->fillHoles();

    //this->updateRender();

    std::cout << "\033[32mFresh facets size: " << this->m_fresh_facets.size() << "\033[0m" << std::endl;
    
    #ifdef _DEBUG
    std::cout << "Removing fanfan and disk fan singular" << std::endl;
    #endif
    while (true)
    {
        // Remove singular vertices
        bool found_fan_fan_vertex = this->removeFanFanSingular();

        bool found_disk_fan_vertex = this->removeDiskFanSingular();

        if (!found_fan_fan_vertex && !found_disk_fan_vertex)
        {
            break;
        }
    }
    //this->updateRender();

    #ifdef _DEBUG
    std::cout << "Clearing" << std::endl;
    #endif

    // Clear fresh facets
    this->clearFreshFacets();

    // Clear fresh vertices
    this->clearFreshVertices();

    // Clear orphan vertices
    //this->clearOrphanVertices();

#ifdef _DEBUG
    // Clear debug render facets
    this->clearDebugRender();
#endif
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
            #ifdef _DEBUG
            std::cout << "Fan Fan vertices found: " << singular_vertices.size() << std::endl;
            #endif
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
        this->removeFacets(singular_facets);
    }
    return found_fan_fan_vertex;
}

bool Mesher::removeDiskFanSingular()
{

    bool found_disk_fan_vertex = false;

    // Get all boundary vertices
    std::set<Vertex *> boundary_vertices;
    for (Edge *e : m_border_edges)
    {
        boundary_vertices.insert(e->getSource());
        boundary_vertices.insert(e->getTarget());
    }

    // Find all relevant vertices
    std::map<Vertex *, unsigned int> fresh_vertices_counter;
    for (Facet *f : m_fresh_facets)
    {
        for (int i = 0; i < 3; i++)
        {
            Vertex *vertex = f->getVertex(i);
            if (boundary_vertices.find(vertex) == boundary_vertices.end())
            {
                // The fresh facet vertex is not in a boundary vertex. We do not need to count it.
                continue;
            }
            // <<<< Add the vertex to the counter
            if (fresh_vertices_counter.find(vertex) == fresh_vertices_counter.end())
            {
            // the key does not exist, then insert it with 1
                fresh_vertices_counter[vertex] = 1;
            }
            else
            {
                fresh_vertices_counter[vertex]++;
            }
            // >>>>
        }
    }

    // Get all the disk fan vertices
    std::list<VertexDiskFanInfo> disk_fan_vertices;
    for (auto &[v, count] : fresh_vertices_counter)
    {
        if (count > 1)
        {
            VertexDiskFanInfo vertex_disk_fan_info = v->getDiskFan();
            #ifdef _DEBUG
            if (vertex_disk_fan_info.disk_facets.size() > 1)
            {
                std::cerr << "Disk facet set size: " << vertex_disk_fan_info.disk_facets.size() << std::endl;
                std::exit(EXIT_FAILURE);
            }
            #endif
            if (vertex_disk_fan_info.disk_facets.size() > 0)
            {
                disk_fan_vertices.push_back(vertex_disk_fan_info);
            }
        }
    }

    // check any facet to be removed that will deplicate.
    std::set<Facet *> facets_to_be_removed;
    for (VertexDiskFanInfo &vertex_disk_fan_info : disk_fan_vertices)
    {
        for (Facet_set &fan_facets : vertex_disk_fan_info.fan_facets)
        {
            for (Facet *facet : fan_facets)
            {

                facets_to_be_removed.insert(facet);
            }
        }
    }

    for (Facet *facet : facets_to_be_removed)
    {   
        std::cout << "Removing facet in DiskFan: " << facet->getVertex(0)->x() << " " << facet->getVertex(0)->y() << " " << facet->getVertex(0)->z() << std::endl;
        //this->removeRedundantDiskFanFacet(facet);
        this->removeFacet(facet);

    }
    return found_disk_fan_vertex;
}

#ifdef _DEBUG
void Mesher::mesh_integrityCheck()
{
    // make a share object of o3d_mesh
    std::shared_ptr<open3d::geometry::TriangleMesh> o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();

    Visualizer::renderMainMesh(this->getFacets(), o3d_mesh);

    // is it vertex manifold?
    bool is_vertex_manifold = o3d_mesh->IsVertexManifold();
    if (!is_vertex_manifold)
    {
        std::cout << "Mesh is not vertex manifold" << std::endl;
        // Get all the non-manifold vertices
        std::vector<int> non_manifold_vertices = o3d_mesh->GetNonManifoldVertices();

        // Print number of non-manifold vertices
        std::cout << "Number of non-manifold vertices: " << non_manifold_vertices.size() << std::endl;
        for (size_t i = 0; i < non_manifold_vertices.size(); i++)
        {
            Eigen::Vector3d v = o3d_mesh->vertices_[non_manifold_vertices[i]];
            
            // the constructor is Vertex(double x, double y, double z, double nx, double ny, double nz)
            //debug_vertices.push_back(ColorVertex(Vertex(v.x(), v.y(), v.z(), 0, 0, 0), Eigen::Vector3d(1.0, 0.0, 0.0)));
            std::cout << "Non-manifold vertex " << i << ": " << v.x() << " " << v.y() << " " << v.z() << std::endl;
        }
         
        std::exit(EXIT_FAILURE);
    }

    // check o3d_mesh properties
    // is it edge manifold?
    bool allow_boundary_edges = true;
    bool is_edge_manifold = o3d_mesh->IsEdgeManifold(allow_boundary_edges);
    if (!is_edge_manifold)
    {
        // get non-manifold edges
        std::vector<Eigen::Vector2i> non_manifold_edges = o3d_mesh->GetNonManifoldEdges(allow_boundary_edges);
        // print non-manifold edges
        std::cout << "Non-manifold edges: " << non_manifold_edges.size() << std::endl;
        // print top 10 non-manifold edges
        for (size_t i = 0; i < non_manifold_edges.size(); i++)
        {
            std::cout << "Non-manifold edge " << i << ": " << non_manifold_edges[i][0] << " " << non_manifold_edges[i][1] << std::endl;
        }
        std::cerr << "Error: The mesh is not edge manifold" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // is it orientable?
    bool is_orientable = o3d_mesh->IsOrientable();
    if (!is_orientable)
    {
        std::cerr << "Error: The mesh is not orientable" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // Check if mesh has self-intersections
    std::vector<Eigen::Vector2i> intersecting_triangles = o3d_mesh->GetSelfIntersectingTriangles();
    if (intersecting_triangles.size() > 0)
    {
        std::cerr << "Error: The mesh has self-intersections" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return;
}
#endif

void Mesher::createAndAddFacet(Vertex *v1, Vertex *v2, Vertex *v3, const Point &center)
{
    Facet *facet = new Facet(v1, v2, v3, center, m_facet_idx);
    m_facet_idx++;
    addFacet(facet); 
}

void Mesher::createAndAddFacet(Edge *edge, Vertex *vertex, const Point &center)
{
    Facet *facet = new Facet(edge, vertex, center, m_facet_idx);
    m_facet_idx++;
    addFacet(facet);
}

void Mesher::clearBatchFacets()
{
    m_batch_facets_removed.clear();
    m_batch_facets_added.clear();
}

void Mesher::removeFromAddBatchFacet(Facet* facet)
{
    m_batch_facets_added.erase(facet);
}

const std::unordered_set<Facet *> &Mesher::getBatchFacetsAdded() const
{
    return m_batch_facets_added;
}

const std::unordered_set<unsigned int> &Mesher::getBatchFacetsRemoved() const
{
    return m_batch_facets_removed;
}

static std::pair<std::unordered_set<Facet*>, std::unordered_set<Edge*>> extract_edge_connected_elements(const std::unordered_set<Edge*> &bEdges)
{
    // Future optimization:
    // One should use the handness of the mesh since our mesh is always orientable. 
    Edge* edge_start = *bEdges.begin();

    std::unordered_set<Edge*> need_to_be_examinated;
    std::unordered_set<Edge*> examined;
    need_to_be_examinated.insert(edge_start);

    std::unordered_set<Edge*> r_bEdges;
    std::unordered_set<Facet*> r_Facets;
    while (need_to_be_examinated.size() > 0)
    {
        //pop the first edge
        Edge* edge_current = *need_to_be_examinated.begin();
        need_to_be_examinated.erase(edge_current);
        examined.insert(edge_current);
        if (edge_current->getType() == Edge::BORDER)
        {
            r_bEdges.insert(edge_current);
        }

        //Facet 1
        Facet* facet1 = edge_current->getFacet1();
        #ifdef _DEBUG
        if (facet1 == nullptr)
        {
            std::cerr << "Error: The facet1 is nullptr" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        #endif

        r_Facets.insert(facet1); 
        std::set<Edge*> edges_facet1 = facet1->getEdges();
        for (Edge* edge : edges_facet1)
        {
            // if edge is border, then add it to r_bEdges
            if (edge->getType() == Edge::BORDER)
            {
                r_bEdges.insert(edge);
            }
            // if edge is not examined, then add it to need_to_be_examinated
            if (examined.find(edge) == examined.end())
            {
                need_to_be_examinated.insert(edge);
            }
        }

        //Facet 2
        Facet* facet2 = edge_current->getFacet2();
        if (facet2 != nullptr)
        {
            r_Facets.insert(facet2);
            std::set<Edge*> edges_facet2 = facet2->getEdges();
            for (Edge* edge : edges_facet2)
            {
                // if edge is border, then add it to r_bEdges
                if (edge->getType() == Edge::BORDER)
                {
                    r_bEdges.insert(edge);
                }
                // if edge is not examined, then add it to need_to_be_examinated
                if (examined.find(edge) == examined.end())
                {
                    need_to_be_examinated.insert(edge);
                }
            }
        }
    }
    return std::make_pair(r_Facets, r_bEdges);
}

static Region constructRegion(std::unordered_set<Edge*> &r_bEdges, std::unordered_set<Facet*> &r_Facets)
{

    std::vector<Boundary> boundaries;
    std::vector<double> sq_lengths;
    unsigned int max_sq_length_index = 0;
    double max_sq_length = 0;

    // Construct boundaries
    while (r_bEdges.size() > 0)
    {
        Boundary boundary;
        Edge* edge_start = *r_bEdges.begin();
        Edge* edge_current = edge_start;

        while(true)
        {
            boundary.addEdge(edge_current);
            Edge* edge_next = edge_current->findNextBoundaryEdge();
            if (edge_next == edge_start)
            {
                break;
            }
            else
            {
                edge_current = edge_next;
            }
        }
        boundary.setOrdered(true);
        boundary.setSqLength();

        boundaries.push_back(boundary);
        sq_lengths.push_back(boundary.getSqLength());
        
        if (boundary.getSqLength() > max_sq_length)
        {
            max_sq_length = boundary.getSqLength();
            max_sq_length_index = static_cast<unsigned int>(boundaries.size()) - 1;
        }

        for (Edge* edge : boundary.getEdges())
        {
            r_bEdges.erase(edge);
        }
    }
    Boundary max_boundary = boundaries.at(max_sq_length_index); 

    // remove the max_boundary from the boundaries
    boundaries.erase(boundaries.begin() + max_sq_length_index);
    std::list<Boundary> holes(boundaries.begin(), boundaries.end());
    
    return Region(r_Facets, max_boundary, holes); 
}

void Mesher::calculateMainRegion()
{
    std::list<Region> regions;
    std::unordered_set<Edge*> bEdges(m_border_edges.begin(), m_border_edges.end());
    while (bEdges.size() > 0)
    { 
        std::unordered_set<Facet*> r_Facets;
        std::unordered_set<Edge*> r_bEdges;
        std::pair<std::unordered_set<Facet*>, std::unordered_set<Edge*>> result = ::extract_edge_connected_elements(bEdges);
        r_Facets = result.first;
        r_bEdges = result.second; 
        
        // remove the edges that are grouped
        for (Edge* edge : r_bEdges)
        {
            bEdges.erase(edge);
        } 

        Region region = ::constructRegion(r_bEdges, r_Facets);
        regions.push_back(region);
    }
    
    // print total number of regions
    std::cout << "Total number of regions: " << regions.size() << std::endl;
    
    // find the max sq_length region index
    double max_sq_length = 0;
    std::list<Region>::iterator max_sq_length_region = regions.begin();
    for (std::list<Region>::iterator it = regions.begin(); it != regions.end(); it++)
    {
        if (it->getSqLength() > max_sq_length)
        {
            max_sq_length = it->getSqLength();
            max_sq_length_region = it;
        }
    }

    m_main_region = *max_sq_length_region;   
    // print the main region
    std::cout << "Main region: " << m_main_region.getSqLength() << std::endl;
    std::cout << "Main region facets: " << m_main_region.getNumFacets() << std::endl;
}

void Mesher::removeNonMainFacets()
{
    std::unordered_set<Facet*> main_facets = m_main_region.getFacets();
    std::unordered_set<Facet*> facets_to_be_removed(m_facets.begin(), m_facets.end());

    for (Facet* facet : main_facets)
    {
        facets_to_be_removed.erase(facet);
    } 

    for (Facet* facet : facets_to_be_removed)
    {
        removeFacet(facet);
    }

    while (true)
    {
        // Remove singular vertices
        bool found_fan_fan_vertex = this->removeFanFanSingular();
        if (!found_fan_fan_vertex)
        {
            break;
        }
    }
}

static Boundary extract_boundary(Edge* edge_start, std::unordered_set<Edge*> &bEdges)
{
    // Assume orientated mesh with no non-manifold vertices
    Vertex* vertex_start = edge_start->getSource();
    Vertex* vertex_end = edge_start->getTarget(); 
    bEdges.erase(edge_start);

    Boundary boundary;
    boundary.addEdge(edge_start);

    bool loop_found = false;
    while (!loop_found)
    {
        for (Edge* edge : bEdges)
        {
            if (edge->getSource() == vertex_end)
            {
                boundary.addEdge(edge);
                bEdges.erase(edge);

                vertex_end = edge->getTarget(); 
                if (vertex_end == vertex_start)
                {
                    loop_found = true;
                }
                break;
            }
        }
    }
    return boundary; 
}

std::list<Boundary> Mesher::getBoundriesPurelyFromBoarder()
{
    std::list<Boundary> boundaries;
    std::unordered_set<Edge*> bEdges(m_border_edges.begin(), m_border_edges.end());
    while (bEdges.size() > 0)
    {
        Edge* edge_start = *bEdges.begin();
        Boundary boundary = ::extract_boundary(edge_start, bEdges);
        boundaries.push_back(boundary);
    }
    
    // find max sq_length boundary
    double max_sq_length = 0;
    std::list<Boundary>::iterator max_sq_length_boundary = boundaries.begin();
    for (std::list<Boundary>::iterator it = boundaries.begin(); it != boundaries.end(); it++)
    {
        it->setSqLength();
        if (it->getSqLength() > max_sq_length)
        {
            max_sq_length = it->getSqLength();
            max_sq_length_boundary = it;
        }
    }

    // How this works?
    // 1. *max_sq_length_boundary creates a COPY of the Boundary
    // 2. erase() deletes the original Boundary from the list
    // 3. push_front() inserts the COPY at the front
    Boundary boundary_copy = *max_sq_length_boundary;
    boundaries.erase(max_sq_length_boundary);
    boundaries.push_front(boundary_copy);

    return boundaries;
}

#ifdef _DEBUG
void Mesher::checkDegenerateTriangle(Facet* facet) const
{
        const Vertex *v1 = facet->getVertex(0);
        const Vertex *v2 = facet->getVertex(1);
        const Vertex *v3 = facet->getVertex(2);
        
        // Calculate vectors for the triangle edges
        Eigen::Vector3d e1_2(v2->x() - v1->x(), v2->y() - v1->y(), v2->z() - v1->z());
        Eigen::Vector3d e2_3(v3->x() - v2->x(), v3->y() - v2->y(), v3->z() - v2->z());
        Eigen::Vector3d e3_1(v1->x() - v3->x(), v1->y() - v3->y(), v1->z() - v3->z());

        // Calculate angles using dot product (in degrees)
        double angle1 = std::acos(e1_2.dot(-e3_1) / (e1_2.norm() * e3_1.norm())) * 180.0 / M_PI;
        double angle2 = std::acos(e2_3.dot(-e1_2) / (e2_3.norm() * e1_2.norm())) * 180.0 / M_PI;
        double angle3 = std::acos(e3_1.dot(-e2_3) / (e3_1.norm() * e2_3.norm())) * 180.0 / M_PI;

        const double min_angle = 0.0001;
        if (angle1 < min_angle || angle2 < min_angle || angle3 < min_angle) {
            std::cout << "Degenerate triangle found with small angle(s):" << std::endl;
            std::cout << "Angles (radians): " << angle1 << ", " << angle2 << ", " << angle3 << std::endl;
            std::cout << "v1: (" << v1->x() << ", " << v1->y() << ", " << v1->z() << ")" << std::endl;
            std::cout << "v2: (" << v2->x() << ", " << v2->y() << ", " << v2->z() << ")" << std::endl; 
            std::cout << "v3: (" << v3->x() << ", " << v3->y() << ", " << v3->z() << ")" << std::endl;
            std::exit(EXIT_FAILURE);
    }

}
#endif

#ifdef _DEBUG
void Mesher::checkFacetsOrientation() const
{
    for (Facet* facet : m_facets)
    {
        if (!facet->isRightOrientation())
        {
            std::cerr << "Error: Facet is not oriented correctly" << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }
}
#endif

#ifdef _DEBUG
void Mesher::checkOctreeIntegrity()
{
    std::cout << "Checking octree vertices integrity" << std::endl;
    m_octree_vertices->integrityCheck();

    std::cout << "Checking octree ball centers integrity" << std::endl;
    m_octree_ball_centers->integrityCheck();
}
#endif



#ifdef _DEBUG
extern std::condition_variable cv_debug_visualization;
extern bool rendering_in_progress;
extern std::mutex o3d_mesh_mutex;
void Mesher::updateRender()
{
    std::unique_lock<std::mutex> lock(o3d_mesh_mutex);
    rendering_in_progress = true;
    cv_debug_visualization.notify_one();
    std::cout << "\033[33mTask in progress set to true\033[0m" << std::endl;
    std::cout << "\033[33mSignal sent from main\033[0m" << std::endl;
    
    bool& ref = rendering_in_progress;  // Create a local reference
    cv_debug_visualization.wait(lock, [&ref]{ return !ref; });
    std::cout << "\033[33mSignal received at main\033[0m" << std::endl;

    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();
}
#endif

void Mesher::saveMeshAsOpen3DPLY(const std::string &filename) const
{
    std::shared_ptr<open3d::geometry::TriangleMesh> O3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    const std::list<Facet*> &facets = this->getFacets();
    
    // Clear existing mesh data
    O3d_mesh->vertices_.clear();
    O3d_mesh->triangles_.clear();
    O3d_mesh->vertex_colors_.clear();

    // Create a map of vertices to their new indices
    std::unordered_map<Vertex *, int> vertex_to_index; // Changed to int
    int current_index = 0;                             // Changed to int

    for (auto facet : facets)
    {
        for (int i = 0; i < 3; i++)
        {
            Vertex *v = facet->getVertex(i);
            if (vertex_to_index.find(v) == vertex_to_index.end())
            {
                vertex_to_index[v] = current_index++;
                O3d_mesh->vertices_.push_back(Eigen::Vector3d(v->x(), v->y(), v->z()));
                O3d_mesh->vertex_colors_.push_back(Eigen::Vector3d(0.5, 0.5, 0.5));
            }
        }
    }

    // create triangles
    for (auto facet : facets)
    {
        O3d_mesh->triangles_.push_back(Eigen::Vector3i(vertex_to_index[facet->getVertex(0)], vertex_to_index[facet->getVertex(1)], vertex_to_index[facet->getVertex(2)]));
    }

    // Save the mesh to a PLY file
    bool success = open3d::io::WriteTriangleMesh(filename, *O3d_mesh); 
    if (!success)
    {
        std::cout << "Failed to save mesh to " << filename << std::endl;
    }
    else
    {
        std::cout << "Mesh saved to " << filename << std::endl;
    }
}
