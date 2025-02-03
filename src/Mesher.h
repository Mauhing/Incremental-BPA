/**
 * @file Mesher.h
 * @brief declares methods for building a surface mesh from points stored in an
 * octree
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
#ifndef MESHER_H
#define MESHER_H

#include <cstdlib>
#include <cstdio>

#include "Octree.h"
#include "OctreeIterator.h"

#include "types.h"
#include "Vertex.h"
#include "Vertex.h"
#include "Facet.h"
#include "Edge.h"
#include "utilities.h"
#include "types.h"
#include <open3d/Open3D.h>
#include <functional>

struct ColorVertex
{
    Vertex vertex;
    Eigen::Vector3d color;
    // constructor
    ColorVertex(const Vertex &vertex, const Eigen::Vector3d &color) : vertex(vertex), color(color) {}
    // copy constructor
    ColorVertex(const ColorVertex &other) : vertex(other.vertex), color(other.color) {}
};

struct ColorVertexPtr
{
    Vertex *vertex;
    Eigen::Vector3d color;
    // constructor
    ColorVertexPtr(Vertex *vertex, const Eigen::Vector3d &color) : vertex(vertex), color(color) {}
    // copy constructor
    ColorVertexPtr(const ColorVertexPtr &other) : vertex(other.vertex), color(other.color) {}

    bool operator==(const ColorVertexPtr &other) const
    {
        // Compare relevant members of ColorVertexPtr
        // Example (adjust according to your actual members):
        return color == other.color && vertex == other.vertex;
    }
};

namespace std
{
    template <>
    struct hash<ColorVertexPtr>
    {
        std::size_t operator()(const ColorVertexPtr &cv) const
        {
            return std::hash<Vertex *>()(cv.vertex); // Hash based only on the vertex pointer
        }
    };
}

struct ColorEdgePtr
{
    Edge *edge;
    Eigen::Vector3d color;
    // constructor
    ColorEdgePtr(Edge *edge, const Eigen::Vector3d &color) : edge(edge), color(color) {}
    // copy constructor
    ColorEdgePtr(const ColorEdgePtr &other) : edge(other.edge), color(other.color) {}

    bool operator==(const ColorEdgePtr &other) const
    {
        // Compare relevant members of ColorFacetPtr
        // Example (adjust according to your actual members):
        return color == other.color && edge == other.edge;
    }
};

namespace std
{
    template <>
    struct hash<ColorEdgePtr>
    {
        std::size_t operator()(const ColorEdgePtr &ce) const
        {
            return std::hash<Edge *>()(ce.edge); // Hash based only on the edge pointer
        }
    };
}

struct ColorFacetPtr
{
    Facet *facet;
    Eigen::Vector3d color;
    // constructor
    ColorFacetPtr(Facet *facet, const Eigen::Vector3d &color) : facet(facet), color(color) {}
    // copy constructor
    ColorFacetPtr(const ColorFacetPtr &other) : facet(other.facet), color(other.color) {}

    bool operator==(const ColorFacetPtr &other) const
    {
        // Compare relevant members of ColorFacetPtr
        // Example (adjust according to your actual members):
        return color == other.color && facet == other.facet;
    }
};

namespace std
{
    template <>
    struct hash<ColorFacetPtr>
    {
        std::size_t operator()(const ColorFacetPtr &cf) const
        {
            return std::hash<Facet *>()(cf.facet); // Hash based only on the facet pointer
        }
    };
}

/**
 * @class Mesher
 * @brief Performs the triangulation of the input points
 *
 * This class contains all methods to perform the Ball Pivoting triangulation
 * of the input set of vertices.
 */
class Mesher
{
protected: // class members
    /**
     * @brief OctreeVertices containing the points to mesh
     * */
    OctreeVertices *m_octree_vertices;

    /** @brief iterator over the octree*/
    OctreeIteratorVertices *m_iterator_vertices;

    OctreeBallCenters *m_octree_ball_centers;

    OctreeIteratorBallCenters *m_octree_ball_centers_iterator;

    /** @brief list of active edges (edge front)*/
    Edge_star_list m_edge_front;

    /** @brief list of created triangles*/
    Facet_star_list m_facets;

    /** @brief list of vertices*/
    Vertex_star_list m_vertices;

    /** @brief list of border edges*/
    Edge_star_list m_border_edges;

    /** @brief list of border edges for in node reconstruction*/
    Edge_star_list m_node_border_edges;

    /** @brief radius of the triangulation*/
    double m_ball_radius;

    /** @brief square ball radius*/
    double m_sq_ball_radius;

    /** @brief number of current index vertex*/
    unsigned int m_vertice_idx;

    /** @brief number of current index facet*/
    unsigned int m_facet_idx;


    std::vector<Vertex *> m_fresh_vertices;

public: // constructor-destructor
    /** @brief default constructor*/
    Mesher();

    /** @brief constructor
     * @param octree octree containing the points to mesh
     * @param iterator iterator over the octree
     */
    Mesher(OctreeVertices *octree, OctreeIteratorVertices *iterator,
           OctreeBallCenters *octree_ball_centers, OctreeIteratorBallCenters *octree_ball_centers_iterator);

    /** @brief destructor*/
    ~Mesher();

public: // reconstruction methods
    /** @brief do the triangulation
     * @return true if at least a triangle was created
     */
    void reconstruct();

    /** @brief do the triangulation
     * @param radii a vector of radius
     */
    void reconstruct(const std::list<double> &radii);

    // void cont_reconstruct();
    // void cont_reconstruct(const std::list<double> &radii);

    /** @brief parallel triangulation
     * @param radii a vector of radius
     */
    void parallelReconstruct(std::list<double> &radii);

    /** @brief fill the triangular holes that remain due to wrong
     * normal orientation
     * (post-processing methods)
     */
    void fillHoles();

    // added by mauhing
    /** @brief get the reconstruction type of the edge
     * @param frontEdge the current focusing front edge
     * @param e1 the first candidate edge
     * @param e2 the second candidate edge
     * @param candidate the candidate vertex
     * @return the reconstruction type
     */
    ReconstructionType computeReconstructionType(
        const Edge *e1,
        const Edge *e2,
        const Vertex *candidate) const;

public: // accessors+modifyers
    /** @brief set the ball radius
     * @param r ball radius
     */
    void setBallRadius(double r);

    /** @brief get ball radius
     * @return ball radius
     */
    double getBallRadius() const;

    /** @brief get square ball radius
     * @return square ball radius
     */
    double getSquareBallRadius() const;

    /** @brief get the number of vertices
     * @return number of mesh vertices
     */
    unsigned int nVertices() const;

    /** @brief get the number of facets
     * @return number of mesh facets
     */
    unsigned int nFacets() const;

    /** @brief get number of border edges
     * @return number of border edges
     */
    unsigned int nBorderEdges() const;

    // unsigned int getNumBallCenters() const;

    // const Point_UnOrdSet& getBallCenters() const;

    /** @brief get access to the mesh vertices
     * @return begin iterator of the vertices
     */
    Vertex_star_list::const_iterator vertices_begin() const;

    /** @brief get access to the mesh vertices
     * @return end iterator of the vertices
     */
    Vertex_star_list::const_iterator vertices_end() const;

    /** @brief get access to the mesh facets
     * @return begin iterator of the facets
     */
    Facet_star_list::const_iterator facets_begin() const;

    /** @brief get access to the mesh facets
     * @return end iterator of the facets
     */
    Facet_star_list::const_iterator facets_end() const;

    const std::vector<Vertex *> &getFreshVertices() const;

    void clearFreshVertices();

private: // auxilliary methods for performing the triangulation
    /** @brief find a seed triangle
     * @return true if a seed triangle was found
     */
    bool findSeedTriangle();

    /** @brief find a seed triangle in a given octree node and expand
     * the triangulation from this seed
     * @param node node to search for a seed triangle
     * @param found 1 if a seed triangle was found; false otherwise
     */
    void findSeedTriangle(OctreeNodeV *node, bool &found);

    /** @brief try to find a triangle around a given point
     * @param point candidate point
     * @return true if a seed triangle was found
     */
    bool trySeed(Vertex &v);

    /** @brief try if a facet seed can be built using two given vertices
     * try to find a third vertex to create a facet
     * @param v1 first vertex
     * @param v2 second vertex
     * @param center center of the facet circumsphere if it exists
     * @return third vertex if any was found, nullptr otherwise
     */
    // Vertex* tryTriangleSeed(Vertex *v1,Vertex *v2, Point &center) const;

    /** @brief test if a facet can be built from three vertices
     * test if a facet can be built from three given vertices
     * @param v1 first vertex
     * @param v2 second vertex
     * @param v3 third vertex
     * @param neighbors the set of 2r neighbors of v1
     * @param center center of the facet circumsphere if it exists
     * @return true if the facet is valid, false otherwose
     */
    // bool tryTriangleSeed(Vertex *v1,Vertex *v2, Vertex *v3,
    //                      Neighbor_star_map &neighbors,
    //                      Point &center) const;
    bool tryTriangleSeed(Vertex *v1, Vertex *v2, Vertex *v3,
                         Neighbor_star_map &neighbors,
                         Point &center,
                         bool &flipIndex) const;

    /** @brief expand the triangulation around each of the front edge*/
    void expandTriangulation();

    /** @brief test if three points are in "empty ball configuration"
     * @param v1 first triangle vertex
     * @param v2 second triangle vertex
     * @param v3 third triangle vertex
     * @param[out] center center of the ball (if any)
     */
    bool emptyBallConfiguration(Vertex *v1, Vertex *v2, Vertex *v3,
                                Point &center) const;

    /** @brief test if three points are in "empty ball configuration"
     * given a center
     * @param v1 first triangle vertex
     * @param v2 second triangle vertex
     * @param v3 third triangle vertex
     * @param neighbors a list of all points around v1,v2,v3 (useful to
     * reduce the number of neighbor search
     * @param center center of the r-sphere passing through v1,v2,v3
     */
    bool checkEmptyBallConfiguration(Vertex *v1, Vertex *v2, Vertex *v3,
                                     const Vertex_star_list &neighbors,
                                     const Point &center) const;

    /** @brief compute the center of a ball of radius m_ball_radius
     * and passing through the three points
     * @param v1 first triangle vertex
     * @param v2 second triangle vertex
     * @param v3 third triangle vertex
     * @param[out] center center of the ball
     * @return true if a center was found false otherwise
     * */
    bool computeBallCenter(const Vertex &v1, const Vertex &v2,
                           const Vertex &v3, Point &center) const;

    bool computeBallCenterUsingOrderOfVertices(const Vertex &v1, const Vertex &v2,
                                               const Vertex &v3, Point &center) const;

    /** @brief  compute a normal direction coherent with the
     * three points normals
     * @param v1 first triangle vertex
     * @param v2 second triangle vertex
     * @param v3 third triangle vertex
     * @param[out] nx normal x component
     * @param[out] ny normal y component
     * @param[out] nz normal z component
     */
    void computeNormal(const Vertex &v1, const Vertex &v2, const Vertex &v3,
                       double &nx, double &ny, double &nz) const;

    /** @brief find a candidate vertex for creating a face
     * @return either the candidate vertex if any or nullptr
     */
    Vertex *findCandidateVertex(Edge *edge, Point &center);

    /** @brief add a facet to the list of mesher facets (calls addvertex)
     * @param f input facet
     */
    void addFacet(Facet *f);

    /** @brief add a vertex to the list of mesher vertices if not
     * already added
     * @param v input vertex
     */
    void addVertex(Vertex *v);

    /** @brief change the radius and set the border edges as the edge front
     * @param radius new radius to be tested
     */
    void changeRadius(double radius);

private: //For incremental rendering
    std::vector<unsigned int> m_batch_facets_removed;
    std::vector<Facet*> m_batch_facets_added;
    void clearBatchFacets();
    
public:
    const std::vector<unsigned int> &getBatchFacetsRemoved() const;
    const std::vector<Facet*> &getBatchFacetsAdded() const;

    void removeFromAddBatchFacet(Facet* facet);

public:
    template <typename SetType>
    void removeFacets(SetType &boundary_facets);

    void removeFacet(Facet *facet);

    void removeOrphanAndUpdate(TOctreeNode<Vertex> *node);

    const std::list<Facet *> &getFacets() const;

    void debugPrintStats();

    Facet_set computeCollisionFacets(const std::list<Vertex *> &vertices);

    void clearOrphanVertices();

    void exileVertex(Vertex *vertex);

    void batchReconstruct(const std::list<Vertex> &vertices);

    void addPointsToOctreeVertices(const std::list<Vertex> &vertices);

    void expandOctree(const std::list<Vertex> &vertices);

    void checkAndRemoveCollisionFacets(const std::list<Vertex*> &vertices);

private: // create facets
    void createAndAddFacet(Vertex *v1, Vertex *v2, Vertex *v3, const Point &center);

    void createAndAddFacet(Edge *edge, Vertex *vertex, const Point &center);

private:
    void resetBoundaryEdges();

public: // Boundaries computation
    bool removeFanFanSingular();

private: // For Open3D rendering
    bool m_slow_visualization;
    std::mutex *visualization_mutex;
    bool new_facet_added;

public: // Open3D rendering
    void setVisualizationSync(std::mutex *mutex)
    {
        visualization_mutex = mutex;
    }

    void clearNewFacetFlag() { new_facet_added = false; }
    void setSlowVisualization(bool slow)
    {
        m_slow_visualization = slow;
    }

public: // Detect and remove Disk-Fan singular Fan
    bool removeDiskFanSingular();

    void clearFreshFacets()
    {
        m_fresh_facets.clear();
    }

private: // Check non-manifold vertices


    Facet_UnOrdSet m_fresh_facets;

    void addFreshFacet(Facet *facet)
    {
        m_fresh_facets.insert(facet);
    }

private: // debug rendering
    mutable std::vector<ColorFacetPtr> m_debug_render_facets;
    mutable std::vector<ColorEdgePtr> m_debug_render_edges;
    mutable std::vector<ColorVertexPtr> m_debug_render_vertices;

    void addDebugRenderFacet(Facet *facet, const Eigen::Vector3d &color)
    {
        m_debug_render_facets.push_back(ColorFacetPtr(facet, color));
    }

    void addDebugRenderEdge(Edge *edge, const Eigen::Vector3d &color)
    {
        m_debug_render_edges.push_back(ColorEdgePtr(edge, color));
    }

    void addDebugRenderVertex(Vertex *vertex, const Eigen::Vector3d &color)
    {
        m_debug_render_vertices.push_back(ColorVertexPtr(vertex, color));
    }

    void clearDebugRender()
    {
        m_debug_render_facets.clear();
        m_debug_render_edges.clear();
        m_debug_render_vertices.clear();
    }

public:
    const std::vector<ColorFacetPtr> &getDebugRenderFacets() const
    {
        return m_debug_render_facets;
    }

    const std::vector<ColorEdgePtr> &getDebugRenderEdges() const
    {
        return m_debug_render_edges;
    }

    const std::vector<ColorVertexPtr> &getDebugRenderVertices() const
    {
        return m_debug_render_vertices;
    }

public:
    const Edge_star_list& debugGetBorderEdges() const
    {
        return m_border_edges;
    }

#ifdef _DEBUG
public: // Sanity check
    void debugCheckOrientation() const;

    void mesh_integrityCheck(std::vector<ColorVertex> &debug_vertices) const;

    void mesh_integrityCheck();

    void checkDegenerateTriangle(Facet* facet) const;

    void checkFacetsOrientation() const;  

    void checkOctreeIntegrity();

    

#endif
public:
    void manual_destructor();
};

#endif
