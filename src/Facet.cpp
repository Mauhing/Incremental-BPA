/** @file  Facet.cpp
 * @brief implementation of the facet methods declared in Facet.h
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

#include "Facet.h"
#include "Edge.h"
#include "Vertex.h"
#include "BallCenter.h"

// Initialize the static member.
std::set<Edge*> Facet::sb_recordedNewBoundaryEdges;

Facet::Facet(Vertex* v0, Vertex* v1, Vertex* v2, const Point &ball_center)
{
    m_vertex[0] = v0;
    m_vertex[1] = v1;
    m_vertex[2] = v2;
    m_ball_center = ball_center;


    Edge *e0 = v0->getLinkingEdge(v1);
    if(e0 == NULL)
    {
        // It is the Facet class own the edge object.
        e0 = new Edge(v0,v1);
    }
    e0->addAdjacentFacet(this);

    Edge *e1 = v1->getLinkingEdge(v2);
    if(e1 == NULL)
    {
        e1 = new Edge(v1,v2);
    }
    e1->addAdjacentFacet(this);

    Edge *e2 = v2->getLinkingEdge(v0);
    if(e2 == NULL)
    {
        e2 = new Edge(v2, v0);
    }
    e2->addAdjacentFacet(this);

    for(unsigned int i = 0; i < 3; i++)
    {
        m_vertex[i]->addAdjacentFacet(this);
        m_vertex[i]->updateType();
    }
}

Facet::Facet(Edge* edge, Vertex* vertex, const Point &ball_center)
{
    Vertex *src = edge->getSource();
    Vertex *tgt = edge->getTarget();

    m_vertex[0] = src;
    m_vertex[1] = vertex;
    m_vertex[2] = tgt;
    m_ball_center = ball_center;

    edge->addAdjacentFacet(this);

    for(int i = 0; i < 2; ++i)
    {
        Edge *e = m_vertex[i]->getLinkingEdge(m_vertex[i+1]);
        if(e == NULL)
        {
            e = new Edge(m_vertex[i], m_vertex[i+1]);
        }
        e->addAdjacentFacet(this);
    }

    for(int i= 0; i < 3; ++i)
    {
        m_vertex[i]->addAdjacentFacet(this);
        m_vertex[i]->updateType();
    }
}

Facet::~Facet()
{ 
    delete m_ball_center_ptr;
    m_ball_center_ptr = nullptr;
    for (int i = 0; i < 3; i++)
    {
        m_vertex[i] = nullptr;
    }
}

void Facet::insertNewBoundaryEdge(Edge* edge)
{
    sb_recordedNewBoundaryEdges.insert(edge);
}

void Facet::removeNewBoundaryEdge(Edge* edge)
{
    sb_recordedNewBoundaryEdges.erase(edge);
}

Vertex* Facet::vertex(unsigned int i) const
{
    unsigned int index = i %3;
    return m_vertex[index];
}

Vertex* Facet::getVertex(unsigned int i)
{
    unsigned int index = i %3;
    return m_vertex[index];
}

Edge* Facet::edge(unsigned int i) const
{
    unsigned int i1 = (i+1) %3;
    unsigned int i2 = (i+2) %3;
    return m_vertex[i1]->getLinkingEdge(m_vertex[i2]);
}

const Point& Facet::getBallCenter() const
{
    return m_ball_center;
}

void Facet::setBallCenter(Point& point)
{
    m_ball_center = point;
}

bool Facet::hasVertex(Vertex* v)
{
    if((m_vertex[0] == v)||(m_vertex[1] == v)||(m_vertex[2] == v))
        return true;
    else
        return false;
}

std::set<Edge*> Facet::getRecordedNewBoundaryEdges()
{
    return sb_recordedNewBoundaryEdges;
}

void Facet::clearNewBoundaryEdges() {
    sb_recordedNewBoundaryEdges.clear();
}

void Facet::setBallCenterPtr(BallCenter* ball_center_ptr) {
    m_ball_center_ptr = ball_center_ptr;
}

Vertex* Facet::nextVertex(const Vertex *v) const
{
    for (int i = 0; i < 3; i++)
    {
        if (m_vertex[i] == v)
            return m_vertex[(i + 1) % 3];
    }
    #ifdef _DEBUG
    std::cerr << "Vertex not found in facet" << std::endl;
    std::exit(EXIT_FAILURE);
    #endif
}

Vertex* Facet::previousVertex(const Vertex *v) const
{
    for (int i = 0; i < 3; i++)
    {
        if (m_vertex[i] == v)
            return m_vertex[(i + 2) % 3];
    }
    #ifdef _DEBUG
    std::cerr << "Vertex not found in facet" << std::endl;
    std::exit(EXIT_FAILURE);
    #endif
}

bool Facet::hasEdge(Edge *e)
{
    for (int i = 0; i < 3; i++)
    {
        if (m_vertex[i]->getLinkingEdge(m_vertex[(i + 1) % 3]) == e)
            return true;
    }
    return false;
}