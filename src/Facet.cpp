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
//std::set<Edge *> Facet::sb_recordedNewBoundaryEdges;

Facet::Facet(Vertex *v0, Vertex *v1, Vertex *v2, const Point &ball_center, unsigned int index)
    : m_ball_center(ball_center), m_ball_center_ptr(nullptr), m_index(index)
{
    m_vertex[0] = v0;
    m_vertex[1] = v1;
    m_vertex[2] = v2;

    Edge *e0 = v0->getLinkingEdge(v1);
    if (e0 == nullptr)
    {
        e0 = new Edge(v0, v1);
    }
    #ifdef _DEBUG
    else {
        std::cerr << "Edge is not nullptr" << std::endl;
        std::cerr << "This function is only meant for creating seed triangles" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    #endif 
    e0->addAdjacentFacet(this);

    Edge *e1 = v1->getLinkingEdge(v2);
    if (e1 == nullptr)
    {
        e1 = new Edge(v1, v2);
    }
    #ifdef _DEBUG
    else {
        std::cerr << "Edge is not nullptr" << std::endl;
        std::cerr << "This function is only meant for creating seed triangles" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    #endif 
    e1->addAdjacentFacet(this);

    Edge *e2 = v2->getLinkingEdge(v0);
    if (e2 == nullptr)
    {
        e2 = new Edge(v2, v0);
    }
    #ifdef _DEBUG
    else {
        std::cerr << "Edge is not nullptr" << std::endl;
        std::cerr << "This function is only meant for creating seed triangles" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    #endif 
    e2->addAdjacentFacet(this);

    for (unsigned int i = 0; i < 3; i++)
    {
        m_vertex[i]->addAdjacentFacet(this);
    }

    for (unsigned int i = 0; i < 3; i++)
    {
        m_vertex[i]->updateType();
    }

#ifdef _DEBUG
    if(!isRightOrientation()) {
        std::cerr << "Facet is not right oriented" << std::endl;
        std::exit(EXIT_FAILURE);
    }
#endif
}

Facet::Facet(Edge *edge, Vertex *vertex, const Point &ball_center, unsigned int index)
    : m_ball_center(ball_center), m_ball_center_ptr(nullptr), m_index(index)
{
    Vertex *src = edge->getSource();
    Vertex *tgt = edge->getTarget();

    m_vertex[0] = src;
    m_vertex[1] = vertex;
    m_vertex[2] = tgt;

    edge->addAdjacentFacet(this);

    for (int i = 0; i < 2; ++i)
    {
        Edge *e = m_vertex[i]->getLinkingEdge(m_vertex[i + 1]);
        if (e == nullptr)
        {
            e = new Edge(m_vertex[i], m_vertex[i + 1]);
        }
        e->addAdjacentFacet(this);
    }

    for (int i = 0; i < 3; ++i)
    {
        m_vertex[i]->addAdjacentFacet(this);
    }

    for (int i = 0; i < 3; ++i)
    {
        m_vertex[i]->updateType();
    }

#ifdef _DEBUG
    if(!isRightOrientation()) {
        std::cerr << "Facet is not right oriented" << std::endl;
        std::exit(EXIT_FAILURE);
    }
#endif
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

Vertex *Facet::vertex(unsigned int i) const
{
    unsigned int index = i % 3;
    return m_vertex[index];
}

Vertex *Facet::getVertex(unsigned int i)
{
    unsigned int index = i % 3;
    return m_vertex[index];
}

const Point &Facet::getBallCenter() const
{
    return m_ball_center;
}

bool Facet::hasVertex(Vertex *v)
{
    if ((m_vertex[0] == v) || (m_vertex[1] == v) || (m_vertex[2] == v))
        return true;
    else
        return false;
}

void Facet::setBallCenterPtr(BallCenter *ball_center_ptr)
{
    m_ball_center_ptr = ball_center_ptr;
}

Vertex *Facet::nextVertex(const Vertex *v) const
{
    for (int i = 0; i < 3; i++)
    {
        if (m_vertex[i] == v)
        {
            return m_vertex[(i + 1) % 3];
        }
    }
#ifdef _DEBUG
    std::cerr << "Vertex not found in facet" << std::endl;
    std::exit(EXIT_FAILURE);
#endif
    return nullptr;
}

Vertex *Facet::previousVertex(const Vertex *v) const
{
    for (int i = 0; i < 3; i++)
    {
        if (m_vertex[i] == v)
        {
            return m_vertex[(i + 2) % 3];
        }
    }
#ifdef _DEBUG
    std::cerr << "Vertex not found in facet" << std::endl;
    std::exit(EXIT_FAILURE);
#endif
    return nullptr;
}

int Facet::getVertexIndex(const Vertex *v) const
{
    for (int i = 0; i < 3; i++)
    {
        if (m_vertex[i] == v)
            return i;
    }
#ifdef _DEBUG
    std::cerr << "Vertex not found in facet" << std::endl;
    std::exit(EXIT_FAILURE);
#endif
    return -1;
}

bool Facet::isRightOrientation() const
{
    // Get adjacent facets through each edge
    for(int i = 0; i < 3; i++) {
        Vertex* v1 = m_vertex[i];
        Vertex* v2 = m_vertex[(i+1)%3];
        Edge* edge = v1->getLinkingEdge(v2);

#ifdef _DEBUG
        if(edge == nullptr) {
            std::cerr << "Edge is nullptr" << std::endl;
            std::exit(EXIT_FAILURE);
        }
#endif

        Facet* adjacent = edge->anotherFacet(this);
        if(adjacent == nullptr) {
            continue;
        }

        // Find indices of these vertices in both facets
        int this_v1_idx = this->getVertexIndex(v1);
        int this_v2_idx = this->getVertexIndex(v2);
        int adj_v1_idx = adjacent->getVertexIndex(v1);
        int adj_v2_idx = adjacent->getVertexIndex(v2);

        // In correctly oriented adjacent facets, vertices should appear in opposite order
        bool is_right_orientation = ((this_v2_idx == (this_v1_idx + 1) % 3) && ((adj_v2_idx + 1)%3 == adj_v1_idx));
        if(!is_right_orientation) {
            return false;
        }
    }
    return true;
    
}

unsigned int Facet::getIndex() const
{
    return m_index;
}

std::set<Edge*> Facet::getEdges()
{
    std::set<Edge*> edges;
    for (int i = 0; i < 3; i++)
    {
        edges.insert(m_vertex[i]->getLinkingEdge(m_vertex[(i + 1) % 3]));
    }
    return edges;
}