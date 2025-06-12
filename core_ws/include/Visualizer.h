/**
 * @file Visualizer.h
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

#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <mutex>
#include <condition_variable>
#include <atomic>
#include "Mesher.h"
#include "types.h"

namespace Visualizer
{
    void visualizationThread(
        Mesher &mesher,
        std::mutex &o3d_mesh_mutex,
        std::atomic<bool> &should_exit,
        std::vector<ColorVertex> &received_vertices,
        std::condition_variable &cv_debug_visualization,
        bool &rendering_in_progress,
        Eigen::Matrix<double, 3, 4> &robot_pose,
        const double &hole_length,
        const bool &show_previous_vertices);

    void renderMainMesh(const Facet_star_list &facets, std::shared_ptr<open3d::geometry::TriangleMesh> &O3d_mesh);

    void renderBoundaries(const std::list<Boundary> &boundaries, std::shared_ptr<open3d::geometry::LineSet> &line, const double &hole_length);

    void renderIncremental(const std::unordered_set<unsigned int> &facets_to_remove, const std::unordered_set<Facet*> &facets_to_add, std::unordered_map<unsigned int, unsigned int> &f_index_2_matrix_row, std::unordered_map<unsigned int, unsigned int> &v_index_2_matrix_row, std::shared_ptr<open3d::geometry::TriangleMesh> &O3d_mesh);

    void renderReceivedVertices(std::vector<ColorVertex> &received_vertices, std::shared_ptr<open3d::geometry::PointCloud> &point);

    void renderFreshVertices(const std::vector<Vertex *> &vertices, std::shared_ptr<open3d::geometry::PointCloud> &O3d_point);

    void renderDebugFacets(const std::vector<ColorFacetPtr> &facets, std::shared_ptr<open3d::geometry::TriangleMesh> &mesh);

    void renderDebugEdges(const std::vector<ColorEdgePtr> &edges, std::shared_ptr<open3d::geometry::LineSet> &line);

    void renderDebugVertices(const std::vector<ColorVertexPtr> &vertices, std::shared_ptr<open3d::geometry::PointCloud> &point);

    void renderDebugVertices(const std::vector<Eigen::Vector3d> &vertices, const Eigen::Vector3d &color, std::shared_ptr<open3d::geometry::PointCloud> &point);


    void renderBorderEdges(const Edge_star_list &border_edges, std::shared_ptr<open3d::geometry::LineSet> &line);

    bool integrityCheck(const std::shared_ptr<open3d::geometry::TriangleMesh> &mesh, std::shared_ptr<open3d::geometry::LineSet> &debug_line);

}

#endif // VISUALIZER_H