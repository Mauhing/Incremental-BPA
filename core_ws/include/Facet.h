/** @file  Facet.h
 * @brief Declaration of a facet object
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
 * 
 * @author Mau Hing Yip mauhingyip@hotmail.com
 * @date 2025-06-12
 * @note This file was further developed based on prior work by Julie Digne.
 */

#ifndef FACET_H
#define FACET_H

#include "Point.h"
#include "Vertex.h"
#include "Edge.h"
#include "BallCenter.h"

/**
 * @class Facet
 * @brief stores a triangular facet
 *
 * A facet contains pointers to its three vertices and its
 * r-circumsphere center
 */
class Facet
{
private:
    /** @brief vertices of the facet*/
    Vertex *m_vertex[3];

    /** @brief center of the ball that generated the facet*/
    const Point m_ball_center;

    BallCenter *m_ball_center_ptr; // Pointer to the ball center. We should remove m_ball_center and only use m_ball_center_ptr later in the code.

    /** @brief unique index for this facet */
    const unsigned int m_index;

public: // constructor+destructor
    /** @brief constructor*/
    Facet() = delete;

    /** @brief constructor from a set of vertices
     * @param v1 first vertex
     * @param v2 second vertex
     * @param v3 third vertex
     * @param ball_center center of the empty interior
     * ball incident to the three vertices
     */
    Facet(Vertex *v1, Vertex *v2, Vertex *v3, const Point &ball_center, unsigned int index);

    /** @brief constructor from an edge and a vertex
     * prerequisite edge has at most one adjacent facet
     * @param edge edge (2 vertices to create the facet)
     * @param vertex third vertex to create the facet
     */
    Facet(Edge *edge, Vertex *vertex) = delete;

    /** @brief constructor from an edge and a vertex
     * prerequisite edge has at most one adjacent facet
     * @param edge edge
     * @param vertex vertex to link to the edge
     * @param ball_center center of the empty interior ball
     * incident to the three vertices
     */
    Facet(Edge *edge, Vertex *vertex, const Point &ball_center, unsigned int index);

    /** @brief destructor*/
    ~Facet();

public: // accessors + modifiers
    /** @brief get facet vertex
     * @param index of the vertex in the facet
     * @return corresponding facet
     */
    Vertex *vertex(unsigned int index) const;

    /** @brief get facet center
     * @return ball center
     */
    const Point &getBallCenter() const;

    /** @brief test if contains vertex
     * @param vertex test vertex
     * @return true if the vertex is a vertex of the facet
     */
    bool hasVertex(Vertex *vertex);

    /** @brief get vertex
     * @param index index of the vertex that mode 3
     * @return vertex
     */
    Vertex *getVertex(unsigned int index);

public:
    /** @brief set the ball center pointer
     * @param ball_center_ptr pointer to the ball center
     */
    void setBallCenterPtr(BallCenter *ball_center_ptr);

    /** @brief get the next vertex
     * it follows the orientation of the facet
     * @param v the vertex
     * @return the next vertex
     */
    Vertex *nextVertex(const Vertex *v) const;

    /** @brief get the next vertex
     * it follows the orientation of the facet, in reverse order.
     * @param v the vertex
     * @return the previous vertex
     */
    Vertex *previousVertex(const Vertex *v) const;

    /** @brief get the index of the vertex
     * @param v the vertex
     * @return the index of the vertex. It is 0, 1 or 2. -1 if the vertex is not found.
     */
    int getVertexIndex(const Vertex *v) const;

    /** @brief get the index of the facet
     * @return the index of the facet
     */
    unsigned int getIndex() const;

    /** @brief get the edges of the facet
     * @return the edges of the facet
     */
    std::set<Edge*> getEdges();

#ifdef _DEBUG
    /** @brief check if the facet is oriented correctly. This is a debug function.
     * @return true if the facet is oriented correctly, false otherwise
     */
    bool isRightOrientation() const;
#endif
};

#endif
