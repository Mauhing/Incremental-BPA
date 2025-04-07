/** @file  Vertex.h
 * @brief Declaration of a vertex object
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

#ifndef VERTEX_H
#define VERTEX_H

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <set>

#include "Point.h"
#include "types.h"
#include "utilities.h"

template <typename T>
class TOctreeNode; // Forward declaration

struct VertexDiskFanInfo
{
    Vertex *disk_fan_vertex;
    std::list<Facet_set> disk_facets;
    std::list<Facet_set> fan_facets;

    VertexDiskFanInfo() : disk_fan_vertex(nullptr) {}
};

/**
 * @class Vertex
 * @brief Input samples to be triangulated
 *
 * Sample point inserted as vertex in the program:
 * to begin with, it is an orphan vertex which will be aggreggated
 * during the triangulation contains topology information
 */
class Vertex : public Point
{
public:
    /**
     * @brief type of vertex
     */
    enum VertexType : unsigned int
    {
        ORPHAN = 0,
        FRONT = 1,
        INNER = 2
    }; // VertexType itself does not form a namespace

    /** @brief overloading operator <<
     * @param out output stream
     * @param v vertex
     * @return output stream
     */
    friend ostream &operator<<(ostream &out, const Vertex &v);

private: // properties
    /** @brief nx, ny, nz normal coordinates*/
    double m_nx, m_ny, m_nz;

    /** @brief set of adjacent edges*/
    Edge_set m_adjacentEdges;

    /** @brief set of adjacent facets*/
    Facet_set m_adjacentFacets;

    /** @brief vertex index of the triangulation, should be 0
     * if the vertex is an orphan*/
    unsigned int m_index;

    /** @brief tag: 0 if orphan, 1 if on front, 2 if inner*/
    VertexType m_type;

    /** @brief pointer to the octree node leaf*/
    TOctreeNode<Vertex> *m_octreeNodeLeaf;

public: // constructor+destructor
    /** @brief default constructor*/
    Vertex();

    /** @brief constructor from coordinates and normal*/
    Vertex(double x, double y, double z, double nx, double ny, double nz);

    /** @brief default destrictor*/
    ~Vertex();

public: // accessors + modifiers
    /** @brief add an edge to the set of edges
     * prerequisite the edge actually contains the vertex
     * @param edge edge the vertex belongs to
     * @return true if the edge was successfully added, false if the edge
     * already was in the set of edges
     */
    bool addAdjacentEdge(Edge *edge);

    /** @brief remove an edge from the set of adjacent edges
     * @param edge edge to remove
     */
    void removeAdjacentEdge(Edge *edge);

    /** @brief add a facet to the set of adjacent facets
     * prerequisite the facet actually contains the vertex
     * @param facet facet adjacent to the vertex
     * @return true if the facet was successfully added, false if the facet
     * already was in the set of facets
     */
    bool addAdjacentFacet(Facet *facet);

    /** @brief remove a facet from the set of adjacent facets
     * @param facet facet to remove
     */
    void removeAdjacentFacet(Facet *facet);

    /** @brief get index of the vertex
     @return index
     */
    unsigned int index() const;

    /** @brief set the vertex index
     * @param index to set
     */
    void setIndex(unsigned int index);

    /** @brief get the set of adjacent edges
     * return the set of adjacent edges
     */
    Edge_set &adjacentEdges();

    /** @brief get edge linking two vertices if any
     * @param vertex test vertex
     * @return edge* if an edge links the two vertices and nullptr otherwise
     */
    Edge *getLinkingEdge(Vertex *vertex) const;

    /** @brief get x normal component
     * @return x normal component nx
     */
    double nx() const;

    /** @brief get y normal component
     * @return y normal component ny
     */
    double ny() const;

    /** @brief get z normal component
     * @return z normal component nz
     * */
    double nz() const;

    /**
     * @brief check if two vertices are compatible with each other and if the normal is compatible with the edge
     * @param v1 first vertex
     * @param v2 second vertex
     * @param changeHandness true if we need to change the order of the vertices, false otherwise
     * @return true if the vertices are compatible, false otherwise
     */
    bool isCompatibleWithAndHandnessCheck(const Vertex &v1, const Vertex &v2, bool &changeHandness) const;

    /** @brief test if a vertex is compatible with an oriented edge
     *@param e test edge
     *@return true if the edge and vertices are compatible
     */
    bool isCompatibleWith(const Edge &e) const;

    /** @brief return vertex types
     * @return type of the vertex (0,1,2)
     */
    VertexType getType() const;

    /** @brief set vertex type
     * @param type (0,1,2)
     */
    void setType(VertexType type);

    /** @brief update the type of a vertex according to its connectivity
     */
    void updateType();

    /** @brief test if a facet is adjacent to a vertex
     *@param facet test facet
     *@return true if the facet is adjacent
     */
    bool isAdjacent(Facet *facet);

    /** @brief test if two vertices are linked by a closed border with
     * three edges.
     * @param test test vertex
     * @return closure vertex or nullptr
     */
    Vertex *findBorder(Vertex *test);

    /** @brief set the octree node leaf
     * @param node octree node leaf
     */
    void setOctreeNodeLeaf(TOctreeNode<Vertex> *node);

public: 
    /** @brief get the set of adjacent facets
     * @return set of adjacent facets
     */
    const Facet_set &adjacentFacets() const;

    /** @brief clear the set of adjacent edges and facets
     */
    void clearAdjacentEdgesAndFacets();

    /** @brief check if the vertex has a disk
     * @return true if the vertex has a disk, false otherwise
     */
    bool hasDisk();

    /** @brief get the disk fan information
     * @return disk fan information
     */
    VertexDiskFanInfo getDiskFan();

    /** @brief remove and delete the vertex
     */
    void remove_and_delete();

private:
    /** @brief extract connected facets
     * @param facets set of facets
     * @return pair of boolean and set of facets
     */
    std::pair<bool, Facet_set> extractConnectedFacets(const Facet_set &facets) const;
};
#endif
