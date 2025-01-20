/**
 * @file Vertex.cpp
 * @brief implementation of the vertex methods declared in Vertex.h
 * @author Julie Digne
 * @date 2012-10-08
 * @copyright This program is free software: you can redistribute it and/or
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

#include "Vertex.h"
#include "Edge.h"
#include "Facet.h"
#include "utilities.h"
#include <cstdio>
#include <iostream>

Vertex::Vertex(double x, double y, double z, double nx, double ny, double nz)
    : Point(x, y, z), m_nx(nx), m_ny(ny), m_nz(nz)
{
    // m_nx = nx;
    // m_ny = ny;
    // m_nz = nz;
    m_index = -1;
    setType(Vertex::ORPHAN);
    m_octreeNodeLeaf = nullptr;
}

Vertex::~Vertex()
{
    // m_nx=m_ny=m_nz=0.0;
    m_index = -1;
    m_adjacentEdges.clear();
    m_adjacentFacets.clear();
    setType(Vertex::ORPHAN); // 0
    m_octreeNodeLeaf = nullptr;
}

bool Vertex::addAdjacentEdge(Edge *edge)
{
    pair<set<Edge *>::iterator, bool> insertion_result;
    insertion_result = m_adjacentEdges.insert(edge);
    return insertion_result.second;
}

void Vertex::removeAdjacentEdge(Edge *edge)
{
    m_adjacentEdges.erase(edge);
}

bool Vertex::addAdjacentFacet(Facet *facet)
{
    pair<set<Facet *>::iterator, bool> insertion_result;
    insertion_result = m_adjacentFacets.insert(facet);
    return insertion_result.second;
}

void Vertex::removeAdjacentFacet(Facet *facet)
{
    m_adjacentFacets.erase(facet);
}

int Vertex::index()
{
    return m_index;
}

void Vertex::setIndex(int index)
{
    if (index < -1)
    {
        std::cerr << "Vertex index is less than -1" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    m_index = index;
}

Edge_set &Vertex::adjacentEdges()
{
    return m_adjacentEdges;
}

Edge *Vertex::getLinkingEdge(Vertex *vertex) const
{
    return getCommonElement(m_adjacentEdges, vertex->adjacentEdges());
}

double Vertex::nx() const
{
    return m_nx;
}

double Vertex::ny() const
{
    return m_ny;
}

double Vertex::nz() const
{
    return m_nz;
}

bool Vertex::isCompatibleWith(const Edge &e) const
{
    double ntx, nty, ntz;
    const Vertex &src = *(e.getSource());
    const Vertex &tgt = *(e.getTarget());

    cross_product(x() - src.x(), y() - src.y(), z() - src.z(),
                  tgt.x() - src.x(), tgt.y() - src.y(),
                  tgt.z() - src.z(), ntx, nty, ntz);

    normalize(ntx, nty, ntz);

    if ((ntx * nx() + nty * ny() + ntz * nz() > -1e-16) && (ntx * src.nx() + nty * src.ny() + ntz * src.nz() > -1e-16) && (ntx * tgt.nx() + nty * tgt.ny() + ntz * tgt.nz() > -1e-16))
        return true;

    return false;
}

bool Vertex::isCompatibleWith(const Vertex &v1, const Vertex &v2) const
{

    // By defualt, we assume that the v0, v1, v2 form a postive vector compare to all normals.

    double ntx, nty, ntz;

    // cross_product(x() - v1.x(), y() - v1.y(), z() - v1.z(),
    //               v2.x() - v1.x(), v2.y() - v1.y(), v2.z() - v1.z(),
    //               ntx, nty, ntz);
    cross_product(v1.x() - x(), v1.y() - y(), v1.z() - z(),
                  v2.x() - v1.x(), v2.y() - v1.y(), v2.z() - v1.z(),
                  ntx, nty, ntz);
    normalize(ntx, nty, ntz);

    // Flip the normal such that it aligns with *this vertex.
    if (ntx * nx() + nty * ny() + ntz * nz() < -1e-16)
    {
        ntx = -ntx;
        nty = -nty;
        ntz = -ntz;
        std::cout << "Flipped the normal." << std::endl;
    }
    // ntx, nty, ntz is now the normal of the plane defined by *this, v1 and v2.

    if ((ntx * v1.nx() + nty * v1.ny() + ntz * v1.nz() > -1e-16) && (ntx * v2.nx() + nty * v2.ny() + ntz * v2.nz() > -1e-16))
        return true;

    return false;
}

// added by mauhing
bool Vertex::isCompatibleWithAndHandnessCheck(const Vertex &v1, const Vertex &v2, bool &changeHandness) const
{
    changeHandness = false;
    // With changeHandness as false, we assume that the v0 go to v1, v1 go to v2, v2 go to v0.

    double ntx, nty, ntz;

    cross_product(v1.x() - x(), v1.y() - y(), v1.z() - z(),
                  v2.x() - x(), v2.y() - y(), v2.z() - z(),
                  ntx, nty, ntz);
    normalize(ntx, nty, ntz);

    // Flip the normal such that it aligns with *this vertex.
    if (ntx * nx() + nty * ny() + ntz * nz() < -1e-16)
    {
        ntx = -ntx;
        nty = -nty;
        ntz = -ntz;
        changeHandness = true;
        // With changeHandness as true, v0 go to v2, v2 go to v1, v1 go to v0.
    }
    // ntx, nty, ntz is now the normal of the plane defined by *this, v1 and v2.

    if ((ntx * v1.nx() + nty * v1.ny() + ntz * v1.nz() > -1e-16) && (ntx * v2.nx() + nty * v2.ny() + ntz * v2.nz() > -1e-16))
        return true;

    return false;
}

bool Vertex::isAdjacent(Facet *facet)
{
    if (m_adjacentFacets.find(facet) != m_adjacentFacets.end())
        return true;
    return false;
}

Vertex::VertexType Vertex::getType() const
{
    return m_type;
}

void Vertex::setType(Vertex::VertexType type)
{
    m_type = type;
}

void Vertex::updateType()
{
    /*
    First, check if the vertex is an orphan.
    Second, check if the vertex is on the boundary.
    Third, if not, the vertex is an inner vertex.
    */
    if (m_adjacentEdges.empty())
    {
        // m_type = 0;
#ifdef _DEBUG
        if (adjacentFacets().size() > 0)
        {
            std::cerr << "\033[1;31mError: Vertex is an orphan but has adjacent facets.\033[0m" << std::endl;
            std::exit(EXIT_FAILURE);
        }
#endif
        m_type = Vertex::ORPHAN; // 0
        return;
    }

    if (this->hasDisk())
    {
        m_type = Vertex::INNER;
        return;
    }

    m_type = Vertex::FRONT;
    return;

    //Edge_set::const_iterator ei;
    //for (ei = m_adjacentEdges.begin(); ei != m_adjacentEdges.end(); ++ei)
    //{
    //    const Edge *e = *ei;
    //    if (e->getType() != Edge::INNER) // 2
    //    {
    //        m_type = Vertex::FRONT; // 1
    //        return;
    //    }
    //}
}

ostream &operator<<(ostream &out, const Vertex &v)
{
    out << v.x() << "\t" << v.y() << "\t" << v.z()
        << "\t" << v.nx() << "\t" << v.ny() << "\t" << v.nz();
    return out;
}

//"this" is the source and test is the target
Vertex *Vertex::findBorder(Vertex *test)
{
    Edge *e0 = getLinkingEdge(test);
    Facet *facet = e0->getFacet1();

    Edge_set::iterator ei = m_adjacentEdges.begin();
    Vertex *candidate = nullptr;
    while (ei != m_adjacentEdges.end())
    {
        // if((*ei)->getType() != 0)
        if ((*ei)->getType() != Edge::BORDER)
        {
            // This could be the case when source is disk-fan vertex.
            ++ei;
            continue;
        }

        // Try source first.
        Vertex *v = (*ei)->getSource();

        if (v == this)
        {
            ++ei;
            continue;
        }

        if (facet->hasVertex(v))
        {
            ++ei;
            continue;
        }

        Edge *e = v->getLinkingEdge(test);
        if (e == nullptr)
        {
            ++ei;
            continue;
        }

        // if((e->getType()!=0)||(e->getSource()!=test))
        if ((e->getType() != Edge::BORDER) || (e->getSource() != test))
        {
            ++ei;
            continue;
        }
        return (v);
    }
    // It can return nullptr.
    return candidate;
}

void Vertex::setOctreeNodeLeaf(TOctreeNode<Vertex> *node)
{
    m_octreeNodeLeaf = node;
}

const Facet_set &Vertex::adjacentFacets() const
{
    return m_adjacentFacets;
}

void Vertex::clearAdjacentEdgesAndFacets()
{
    m_adjacentEdges.clear();
    m_adjacentFacets.clear();
}

bool Vertex::hasDisk()
{
    return getDiskFan().disk_facets.size() > 0;
}

VertexDiskFanInfo Vertex::getDiskFan()
{
    Facet_set neighbor_facets = this->adjacentFacets();
    VertexDiskFanInfo vertex_disk_fan_info;
    vertex_disk_fan_info.disk_fan_vertex = this;

    while (!neighbor_facets.empty())
    {
        std::pair<bool, Facet_set> result = extractConnectedFacets(neighbor_facets);
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

std::pair<bool, Facet_set> Vertex::extractConnectedFacets(const Facet_set &facets) const
{

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
        Vertex *next_v = current_facet->nextVertex(this);
        Edge *e = this->getLinkingEdge(next_v);
        Facet *next_facet = e->anotherFacet(current_facet);

#ifdef _DEBUG
        if (next_facet != nullptr && utilities::isNotInSet(next_facet, facets))
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

        connected_facets.insert(next_facet);
        current_facet = next_facet;

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
    }

    if (is_disk_facet_set)
    {
        return std::make_pair(is_disk_facet_set, connected_facets);
    }

    // 2. Opposite orientation
    current_facet = starting_facet;
    while (true)
    {
        Vertex *prev_v = current_facet->previousVertex(this);
        Edge *e = this->getLinkingEdge(prev_v);
        Facet *prev_facet = e->anotherFacet(current_facet);

#ifdef _DEBUG
        if (prev_facet != nullptr && utilities::isNotInSet(prev_facet, facets))
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
        current_facet = prev_facet;

#ifdef _DEBUG
        if (current_facet == starting_facet)
        {
            std::cerr << "\033[1;31mError: Current facet is the same as starting facet\033[0m" << std::endl;
            std::exit(EXIT_FAILURE);
        }
#endif
    }
    return std::make_pair(is_disk_facet_set, connected_facets);
}