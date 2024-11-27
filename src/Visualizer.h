#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <mutex>
#include <condition_variable>
#include <atomic>
#include "Mesher.h"

namespace Visualizer
{
    void visualizationThread(
        Mesher& mesher,
        std::mutex& o3d_mesh_mutex,
        std::condition_variable& vis_cv,
        std::atomic<bool>& should_exit
    );

    void renderFacets(const std::vector<ColorFacet> &facets, std::shared_ptr<open3d::geometry::TriangleMesh>& mesh);
    void renderEdges(const std::vector<ColorEdge> &edges, std::shared_ptr<open3d::geometry::LineSet>& line);
    void renderVertices(const std::vector<ColorVertex> &vertices, std::shared_ptr<open3d::geometry::PointCloud>& point);
}

#endif // VISUALIZER_H