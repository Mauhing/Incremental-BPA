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
#include <Open3D.h>

/**
 * @class Mesher
 * @brief Performs the triangulation of the input points
 * 
 * This class contains all methods to perform the Ball Pivoting triangulation 
 * of the input set of vertices.
 */
class Mesher
{
    protected : //class members

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

        /** @brief number of facets*/
        unsigned int m_nfacets;
        
        std::unordered_set<unsigned int> m_recycle_vertices_idx;

    public : //constructor-destructor

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

    public : //reconstruction methods

        /** @brief do the triangulation
         * @return true if at least a triangle was created
         */
        void reconstruct();

        /** @brief do the triangulation
         * @param radii a vector of radius
         */
        void reconstruct(const std::list<double> &radii);

        //void cont_reconstruct();
        //void cont_reconstruct(const std::list<double> &radii);

        /** @brief parallel triangulation
         * @param radii a vector of radius
         */
        void parallelReconstruct(std::list<double> &radii);

        /** @brief fill the triangular holes that remain due to wrong
         * normal orientation
         * (post-processing methods)
         */
        void fillHoles();

        //added by mauhing
        /** @brief get the reconstruction type of the edge
         * @param frontEdge the current focusing front edge
         * @param e1 the first candidate edge
         * @param e2 the second candidate edge
         * @param candidate the candidate vertex
         * @return the reconstruction type
         */
        ReconstructionType computeReconstructionType(
                                                 const Edge* e1, 
                                                 const Edge* e2,
                                                 const Vertex* candidate) const;

    public : //accessors+modifyers

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

        /** @brief get the number of edges in the advancing front
         * @return number of front edges
         */
        unsigned int nFrontEdges() const;

        /** @brief get number of border edges
         * @return number of border edges
         */
        unsigned int nBorderEdges() const;

        //unsigned int getNumBallCenters() const;

        //const Point_UnOrdSet& getBallCenters() const;

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

    public: // reset octree and octree iterator
            
        /** @brief reset the octree and the octree iterator
        * @param octree octree containing the points to mesh
        * @param iterator iterator over the octree
        */
        void resetOctree(OctreeVertices *octree, OctreeIteratorVertices *iterator);

        /** @brief find all boundary edges and mark it as front edges
         */
        //void allBoundaryEdgesToFrontEdges();

    public:
        std::set<Facet*>& getBoundaryFacets() const;
        

    protected :

        /** @brief reconstruct confined to a band around an inside of a node
         * @param node confinement node
         * @param d band width
         */
        void  reconstructAroundNode(OctreeNodeV *node, double d);


    private : //auxilliary methods for performing the triangulation

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
         * @return third vertex if any was found, NULL otherwise
         */
        Vertex* tryTriangleSeed(Vertex *v1,Vertex *v2, Point &center) const;
        

        /** @brief test if a facet can be built from three vertices
         * test if a facet can be built from three given vertices
         * @param v1 first vertex
         * @param v2 second vertex
         * @param v3 third vertex
         * @param neighbors the set of 2r neighbors of v1
         * @param center center of the facet circumsphere if it exists
         * @return true if the facet is valid, false otherwose
         */
        //bool tryTriangleSeed(Vertex *v1,Vertex *v2, Vertex *v3,
        //                     Neighbor_star_map &neighbors,
        //                     Point &center) const;
        bool tryTriangleSeed(Vertex *v1,Vertex *v2, Vertex *v3,
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
        bool checkEmptyBallConfiguration(Vertex* v1, Vertex* v2, Vertex* v3,
                                         const Vertex_star_list &neighbors,
                                         const Point & center) const;

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

        void computeNormalUsingOrderOfVertices(const Vertex &v1, const Vertex &v2, const Vertex &v3,
                           double &nx, double &ny, double &nz) const;

        /** @brief find a candidate vertex for creating a face
         * @return either the candidate vertex if any or NULL
         */
        Vertex* findCandidateVertex(Edge *edge, Point &center);

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

    private ://parallel methods

        /** @brief find a seed triangle in a given octree node or at least
         * a front edge and expand the triangulation it while keeping the
         * front contained in a dilated cell
         * @param containment_node node in which the expansion front
         * stays confined
         * @param node node to search for a seed triangle
         * @param d band width around the node
         * @param found 1 if a seed triangle was found; false otherwise
         */
         void findSeedTriangle(OctreeNodeV *containment_node,
                               OctreeNodeV *node, double d, bool &found);

        /** @brief try to find a triangle around a given point
         * @param v candidate point
         * @param containment_node ensuring that the seed found is inside
         * a node (or in a small band around the node, see d parameter)
         * @param d band width around the node
         * @return true if a seed triangle was found
         */
        bool trySeed(Vertex& v, OctreeNodeV *containment_node, double d);


        /**expand triangulation in a loose box around the node
         * @param containment_node for restricting the triangulation
         * @param d bandwidth around the node
         */
        void expandTriangulationAroundNode(OctreeNodeV* containment_node,
                                           double d);

        /** @brief get all active edges in a node and add it to the
         * parameter set of active edges
         * @param containment_node for restricting the triangulation
         * @param[out] active_egdes set of active edges
         */
        void collectActiveEdges(OctreeNodeV* containment_node,
                                Edge_set &active_edges);

        /** @brief get all border edges in a node and add it to mesher border
         * @param containment_node for restricting the triangulation
         */
        void collectBorderEdges(OctreeNodeV* containment_node);

        /** @brief get all border edges in a node and add it to
         * the parameter set of border_edges
         * @param containment_node for restricting the triangulation
         * @param[out] border_egdes set of border edges
         */
        void collectBorderEdges(OctreeNodeV* containment_node,
                                Edge_set &border_edges);

    public:
        void removeFacets(std::set<Facet*> &boundary_facets);
        
        void removeFacet(Facet* facet);

        void removeOrphanAndUpdate(TOctreeNode<Vertex>* node);

        const std::list<Facet*>& getFacets() const;
         
        Edge_star_list getBorderEdges() const;

        void print_stats(); 
        
        Facet_set computeCollisionFacets(std::list<Vertex>& vertices);

        void clearOrphanVertices();

        void exileVertex(Vertex* vertex);

        void batchReconstruct(std::list<Vertex>& vertices);

    public: // Open3D rendering
        void renderIntoOpen3D(open3d::geometry::TriangleMesh& mesh);

    public: // Sanity check
        void SanityCheckOrientation() const;
        
        void debug_print_vertices() const;
};

#endif
