/**
 * @file Region.cpp
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
 * 
 * @author Mau Hing Yip mauhingyip@hotmail.com
 * @date 2025-06-12
 */

#include "Region.h"

Boundary::Boundary():m_is_ordered(false), m_sq_length(0.0)
{
}

Boundary::~Boundary()
{
}

Boundary::Boundary(const Boundary& other):
    m_edges(other.m_edges),
    m_is_ordered(other.m_is_ordered),
    m_sq_length(other.m_sq_length)
{
}

Boundary& Boundary::operator=(const Boundary& other)
{
    m_edges = other.m_edges;
    m_is_ordered = other.m_is_ordered;
    m_sq_length = other.m_sq_length;
    return *this;
}

void Boundary::addEdge(Edge* edge)
{
    m_edges.insert(edge);
}

void Boundary::setOrdered(bool is_ordered)
{
    m_is_ordered = is_ordered;
}

const std::set<Edge*>& Boundary::getEdges() const
{
    return m_edges;
}

void Boundary::setSqLength()
{
    double sq_length = 0.0;
    for (Edge* edge : m_edges)
    {
        sq_length += edge->getSqLength();
    }
    m_sq_length = sq_length;
}

double Boundary::getSqLength() const
{
    return m_sq_length;
}

std::unordered_set<Facet*> Boundary::getFacets() const
{
    // Future optimization:
    // One should use the handness of the mesh since our mesh is always orientable. 
    Edge* edge_start = *m_edges.begin();

    std::unordered_set<Edge*> need_to_be_examinated;
    std::unordered_set<Edge*> examined;
    need_to_be_examinated.insert(edge_start);

    std::unordered_set<Facet*> r_Facets;
    while (need_to_be_examinated.size() > 0)
    {
        //pop the first edge
        Edge* edge_current = *need_to_be_examinated.begin();
        need_to_be_examinated.erase(edge_current);
        examined.insert(edge_current);

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
                // if edge is not examined, then add it to need_to_be_examinated
                if (examined.find(edge) == examined.end())
                {
                    need_to_be_examinated.insert(edge);
                }
            }
        }
    }
    return r_Facets;
}

Region::Region(const std::unordered_set<Facet*>& land, const Boundary& coast, const std::list<Boundary>& lakes)
{
    m_land = land;
    m_coast = coast;
    m_lakes = lakes;
    m_sq_length = coast.getSqLength();
}

Region::~Region()
{

}

Region::Region(const Region& other)
{
    m_land = other.m_land;
    m_coast = other.m_coast;
    m_lakes = other.m_lakes;
    m_sq_length = other.m_sq_length;
}

Region& Region::operator=(const Region& other)
{
    m_land = other.m_land;
    m_coast = other.m_coast;
    m_lakes = other.m_lakes;
    m_sq_length = other.m_sq_length;
    return *this;
}

double Region::getSqLength() const
{
    return m_sq_length;
}

unsigned int Region::getNumFacets() const
{
    return static_cast<unsigned int>(m_land.size());
}

std::unordered_set<Facet*> Region::getFacets() const
{
    return m_land;
}