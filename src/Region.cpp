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