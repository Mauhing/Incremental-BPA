#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <mutex>
#include <condition_variable>
#include <atomic>
#include "Mesher.h"

namespace Visualizer
{
    void visualizationThread(
        Mesher &mesher,
        std::mutex &o3d_mesh_mutex,
        std::atomic<bool> &should_exit);

    void renderMainMesh(const Facet_star_list &facets, std::shared_ptr<open3d::geometry::TriangleMesh> &O3d_mesh);
    void renderFreshVertices(const std::vector<Vertex *> &vertices, std::shared_ptr<open3d::geometry::PointCloud> &O3d_point);

    void renderDebugFacets(const std::vector<ColorFacet> &facets, std::shared_ptr<open3d::geometry::TriangleMesh> &mesh);
    void renderDebugEdges(const std::vector<ColorEdge> &edges, std::shared_ptr<open3d::geometry::LineSet> &line);
    void renderDebugVertices(const std::vector<ColorVertex> &vertices, std::shared_ptr<open3d::geometry::PointCloud> &point);

    void renderDebugVertices(const std::vector<Eigen::Vector3d> &vertices, const Eigen::Vector3d &color, std::shared_ptr<open3d::geometry::PointCloud> &point);
}

#endif // VISUALIZER_H