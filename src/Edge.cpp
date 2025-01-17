/**
 * @file Edge.cpp
 * @author Julie Digne julie.digne@liris.cnrs.fr
 * @date 2012-10-08
 * @brief implementation of the edge methods declared in Edge.h
 *
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

#include <cstdlib>
#include <cstdio>
#include <iostream>

#include "Edge.h"
#include "Vertex.h"
#include "Facet.h"
#include "utilities.h"

using namespace std;

std::ostream &operator<<(std::ostream &os, const Edge::EdgeType &type)
{
    switch (type)
    {
    case Edge::BORDER:
        os << "BORDER";
        break;
    case Edge::FRONT:
        os << "FRONT";
        break;
    case Edge::INNER:
        os << "INNER";
        break;
    default:
        os << "UNKNOWN";
        break;
    }
    return os;
}

Edge::Edge()
{
    m_src = nullptr;
    m_tgt = nullptr;
    m_facet1 = nullptr;
    m_facet2 = nullptr;
}

Edge::Edge(Vertex *src, Vertex *tgt)
{
    m_src = src;
    m_tgt = tgt;
    src->addAdjacentEdge(this);
    tgt->addAdjacentEdge(this);
    m_facet1 = nullptr;
    m_facet2 = nullptr;
    setType(EdgeType::FRONT);
}

Edge::~Edge()
{
    // std::cout << "Edge destructor called" << std::endl;
    m_src = nullptr;
    m_tgt = nullptr;
    m_facet1 = nullptr;
    m_facet2 = nullptr;
}

Vertex *Edge::getSource() const
{
    return m_src;
}

Vertex *Edge::getTarget() const
{
    return m_tgt;
}

Facet *Edge::getFacet1() const
{
    return m_facet1;
}

Facet *Edge::getFacet2() const
{
    return m_facet2;
}

bool Edge::addAdjacentFacet(Facet *facet)
{
    if ((m_facet1 == facet) || (m_facet2 == facet))
        return false;

    if (m_facet1 == nullptr)
    {
        m_facet1 = facet;
        //updateOrientation(); // TODO: check if this is necessary
        setType(EdgeType::FRONT);
        return true;
    }

    if (m_facet2 == nullptr)
    {
        m_facet2 = facet;
        setType(EdgeType::INNER);
        return true;
    }

    std::cout << "Already two triangles" << endl;
    return false;
}

bool Edge::removeAdjacentFacet(Facet *facet)
{

    if (m_facet1 == facet)
    {
        m_facet1 = m_facet2;
        m_facet2 = nullptr;
        if (m_facet1 != nullptr) {
            alignWithFacet1();
        }
        setType(EdgeType::FRONT);
        return true;
    }

    if (m_facet2 == facet && m_facet1 != nullptr)
    {
        m_facet2 = nullptr;
        setType(EdgeType::FRONT);
        return true;
    }
    
    #ifdef _DEBUG
    std::cerr << "Edge::removeAdjacentFacet() - Facet not found in edge" << std::endl;
    std::exit(EXIT_FAILURE);
    #endif

    return false;
}

void Edge::updateOrientation()
{
    Vertex *opp = getOppositeVertex();
#ifdef _DEBUG
    if (opp == nullptr)
    {
        std::cerr << "\033[1;31mEdge::updateOrientation() - Opposite vertex is null\033[0m" << std::endl;
        std::exit(EXIT_FAILURE);
    }
#endif

    double vx, vy, vz;

    cross_product(m_tgt->x() - m_src->x(), m_tgt->y() - m_src->y(),
                  m_tgt->z() - m_src->z(), opp->x() - m_src->x(),
                  opp->y() - m_src->y(), opp->z() - m_src->z(),
                  vx, vy, vz);
    normalize(vx, vy, vz);

    double nx, ny, nz;
    nx = m_src->nx() + m_tgt->nx() + opp->nx();
    ny = m_src->ny() + m_tgt->ny() + opp->ny();
    nz = m_src->nz() + m_tgt->nz() + opp->nz();
    normalize(nx, ny, nz);

    if (vx * nx + vy * ny + vz * nz < 0)
    {
        Vertex *temp = m_src;
        m_src = m_tgt;
        m_tgt = temp;

#ifdef _DEBUG
        std::cerr << "\033[1;31mEdge::updateOrientation() - Edge flipped\033[0m" << std::endl;
        std::exit(EXIT_FAILURE);
#endif
    }
}

bool Edge::hasVertex(Vertex *vertex) const
{
    if ((vertex == m_src) || (vertex == m_tgt))
        return true;
    return false;
}

Edge::EdgeType Edge::getType() const
{
    return m_type;
}

// void Edge::setType(int type)
//{
//     m_type = type;
// }

void Edge::setType(EdgeType type)
{
    m_type = type;
}

Vertex *Edge::getOppositeVertex() const
{
    if (m_facet1 == nullptr)
        return nullptr;
    Vertex *opp;
    for (int i = 0; i < 3; i++)
    {
        opp = m_facet1->vertex(i);
        if ((opp != m_src) && (opp != m_tgt))
            return opp;
    }
    return nullptr;
}

Facet *Edge::anotherFacet(const Facet *f) const
{
    if (f == m_facet1)
        return m_facet2;
    if (f == m_facet2)
        return m_facet1;
#ifdef _DEBUG
    std::cerr << "Facet not found in edge" << std::endl;
    std::exit(EXIT_FAILURE);
#endif
    return nullptr;
}

#ifdef _DEBUG
unsigned int Edge::debugGetNumAdjacentFacets() const
{
    if (m_facet1 == nullptr && m_facet2 == nullptr)
        return 0;
    if (m_facet1 != nullptr && m_facet2 == nullptr)
        return 1;
    if (m_facet1 != nullptr && m_facet2 != nullptr)
        return 2;
#ifdef _DEBUG
    std::cerr << "m_facet2 is not nullptr and m_facet1 is nullptr" << std::endl;
    std::cerr << "This is violating the function assumption" << std::endl;
    std::exit(EXIT_FAILURE);
#endif
}
#endif

void Edge::flipOrientation()
{
    Vertex *temp = m_src;
    m_src = m_tgt;
    m_tgt = temp;
}

bool Edge::alignWithFacet1()
{
    Facet *facet = m_facet1;
    
    // Compare facet source and edge source to see if they are the same vertex
    Vertex *getEdgeSource = getSource();
    Vertex *getEdgeTarget = getTarget();

    const int facetSourceIndex = facet->getVertexIndex(getEdgeSource);
    const int facetTargetIndex = facet->getVertexIndex(getEdgeTarget);

    if ((facetSourceIndex + 1) % 3 == facetTargetIndex)
    {
        // This is good.
        return false;
    }
    else
    {
        flipOrientation();
        return true;
    } 
}