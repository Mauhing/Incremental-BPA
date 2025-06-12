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
 * 
 * @author Mau Hing Yip mauhingyip@hotmail.com
 * @date 2025-06-12
 * @note This file was further developed based on prior work by Julie Digne.
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
#include "Region.h"

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

    /** @brief list of fresh vertices*
     * @details New triangle mesh in the current batch reconstruction
     */
    std::vector<Vertex *> m_fresh_vertices;

    /** @brief main region*/
    Region m_main_region;

    /** @brief random device*/
    bool m_random_device;

    /** @brief seed*/
    int m_seed;

    /** @brief seed triangles every batch*/
    bool m_seed_triangles_every_batch;

public: // constructor-destructor
    /** @brief default constructor*/
    Mesher();

    /** @brief constructor
     * @param octree octree containing the points to mesh
     * @param iterator iterator over the octree
     */
    Mesher(OctreeVertices *octree, OctreeIteratorVertices *iterator,
           OctreeBallCenters *octree_ball_centers, OctreeIteratorBallCenters *octree_ball_centers_iterator,
           const bool &random_device, const int &seed, const bool &seed_triangles_every_batch);

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

    void initialize(const Point &origin, double size, unsigned int depth);

public: // accessors+modifyers
    /** @brief set the ball radius
     * @param r ball radius
     */
    void setBallRadius(double r);

    /** @brief get the number of facets
     * @return number of mesh facets
     */
    unsigned int nFacets() const;

    /** @brief get number of border edges
     * @return number of border edges
     */
    unsigned int nBorderEdges() const;

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

    /** @brief clear the vertices of the current batch reconstruction
     */
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
     * @param changeHandness true if we need to change the handness of the facet
     * @return true if the facet is valid, false otherwise
     */
    bool tryTriangleSeed(Vertex *v1, Vertex *v2, Vertex *v3,
                         Neighbor_star_map &neighbors,
                         Point &center,
                         bool &changeHandness) const;

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

public:
    /** @brief remove facets*/
    template <typename SetType, bool exile = false>
    void removeFacets(SetType &boundary_facets);

    /** @brief remove facet*/
    template <bool exile = false>
    void removeFacet(Facet *facet);

    /** @brief remove orphan and update*/
    void removeOrphanAndUpdate(TOctreeNode<Vertex> *node);

    /** @brief get facets*/
    const std::list<Facet *> &getFacets() const;

    /** @brief debug print stats*/
    void debugPrintStats();

    /** @brief compute collision facets*/
    Facet_set computeCollisionFacets(const std::list<Vertex *> &vertices);

    /** @brief clear orphan vertices*/
    void clearOrphanVertices();

    /** @brief exile vertex*/
    void exileVertex(Vertex *vertex);

    /** @brief batch reconstruct*/
    void batchReconstruct(const std::list<Vertex> &vertices);

    /** @brief add points to octree vertices*/
    void addPointsToOctreeVertices(const std::list<Vertex> &vertices);

    /** @brief expand octree*/
    void expandOctree(const std::list<Vertex> &vertices);

    /** @brief check and remove collision facets*/
    void checkAndRemoveCollisionFacets(const std::list<Vertex*> &vertices);

private: // create facets
    /** @brief create and add facet*/
    void createAndAddFacet(Vertex *v1, Vertex *v2, Vertex *v3, const Point &center);

    /** @brief create and add facet*/
    void createAndAddFacet(Edge *edge, Vertex *vertex, const Point &center);

private:
    /** @brief reset boundary edges*/
    void resetBoundaryEdges();

public: // Disk-Fan singular
    /** @brief remove disk-fan singular*/
    bool removeFanFanSingular();

public: // Detect and remove Disk-Fan singular Fan
    /** @brief remove disk-fan singular*/
    bool removeDiskFanSingular();

    /** @brief clear fresh facets*/
    void clearFreshFacets()
    {
        m_fresh_facets.clear();
    }

private: // Check non-manifold vertices
    /** @brief list of fresh facets*/
    Facet_UnOrdSet m_fresh_facets;

    /** @brief add a fresh facet*/
    void addFreshFacet(Facet *facet)
    {
        m_fresh_facets.insert(facet);
    }

public: // Boundaries computation
    /**
     * @brief Set the Main Region object
     */
    void calculateMainRegion();
    
    /**
     * @brief Remove non-main facets
     */
    void removeNonMainFacets();
    
    /**
     * @brief Get boundaries purely from boarder
     * @return List of boundaries
     */
    std::list<Boundary> getBoundriesPurelyFromBoarder();

// The incremental rendering is not used in the current implementation
// However, I do not want to remove the code in case we want to use it in the future
private: // For incremental rendering
    /** @brief list of facets removed in the current batch reconstruction
     */
    std::unordered_set<unsigned int> m_batch_facets_removed;
    /** @brief list of facets added in the current batch reconstruction
     */
    std::unordered_set<Facet *> m_batch_facets_added;

public: // For incremental rendering
    const std::unordered_set<unsigned int> &getBatchFacetsRemoved() const;
    const std::unordered_set<Facet *> &getBatchFacetsAdded() const;
    void clearBatchFacets();
    void removeFromAddBatchFacet(Facet* facet);

public:
    void saveMeshAsOpen3DPLY(const std::string &filename) const;

// Debugging methods
#ifdef _DEBUG
public:
    void mesh_integrityCheck();
    void checkDegenerateTriangle(Facet* facet) const;
    void checkFacetsOrientation() const;  
    void checkOctreeIntegrity();
    void updateRender();
#endif

#ifdef _DEBUG
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

    const Edge_star_list& debugGetBorderEdges() const
    {
        return m_border_edges;
    }
#endif
};

#endif
