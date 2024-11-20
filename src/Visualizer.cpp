#include <mutex>
#include <condition_variable>
#include <atomic>
#include <open3d/Open3D.h>
#include "Visualizer.h"

void Visualizer::visualizationThread(
    Mesher& mesher,
    std::mutex& o3d_mesh_mutex,
    std::condition_variable& vis_cv,
    std::atomic<bool>& should_exit
) {
    // Create a visualizer object
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Open3D Mesh Viewer", 1600, 900);
    visualizer.GetRenderOption().mesh_show_back_face_ = true;
    visualizer.GetRenderOption().point_size_ = 5.0;

    // Store the color we want to maintain
    const Eigen::Vector3d golden_color(1.0, 0.7, 0.0);

    // Create a shared pointer to store the mesh
    auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    // Add initial mesh data
    {
        std::lock_guard<std::mutex> lock(o3d_mesh_mutex);
        mesher.renderIntoOpen3D(*o3d_mesh);
        o3d_mesh->vertex_colors_.resize(o3d_mesh->vertices_.size(), Eigen::Vector3d(1.0, 0.7, 0.0));
        //o3d_mesh->ComputeTriangleNormals();
    }
    visualizer.AddGeometry(o3d_mesh);

    // Add a dummy line segment
    auto line_segment = std::make_shared<open3d::geometry::LineSet>();
    line_segment->points_.push_back(Eigen::Vector3d(0, 3, 0));
    line_segment->points_.push_back(Eigen::Vector3d(5, 3, 0));
    line_segment->lines_.push_back(Eigen::Vector2i(0, 1));
    line_segment->colors_.push_back(Eigen::Vector3d(1.0, 0.0, 0.0)); // Red color
    visualizer.GetRenderOption().line_width_ = 500.0; // Make lines thicker
    visualizer.AddGeometry(line_segment);

    // Set default viewpoint
    visualizer.GetViewControl().SetFront({0, 0, -1});
    visualizer.GetViewControl().SetLookat({0, 0, 0});
    visualizer.GetViewControl().SetUp({0, 1, 0});
    visualizer.GetViewControl().SetZoom(0.7);

    //bool first_frame = true;

    // add coordinate axes
    auto coordinate_axes = open3d::geometry::TriangleMesh::CreateCoordinateFrame(5.0);
    visualizer.AddGeometry(coordinate_axes);

    // Visualization loop
    while (!should_exit) {
        if (!visualizer.PollEvents()) {  // Window was closed
            should_exit = true;  // Signal main thread to exit
            break;
        }

        // Wait for new facet with timeout
        {
            std::unique_lock<std::mutex> lock(o3d_mesh_mutex);
            vis_cv.wait_for(lock, 
                std::chrono::milliseconds(16),  // Short timeout for responsiveness
                [&mesher]{ return mesher.hasNewFacet(); });

            if (mesher.hasNewFacet()) {
                mesher.renderIntoOpen3D(*o3d_mesh);
                o3d_mesh->vertex_colors_.clear();
                o3d_mesh->vertex_colors_.resize(o3d_mesh->vertices_.size(), golden_color);
                o3d_mesh->ComputeVertexNormals();
                o3d_mesh->ComputeTriangleNormals();
                visualizer.UpdateGeometry(o3d_mesh);
                mesher.clearNewFacetFlag();

                // Optional: add small delay to make visualization more visible
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        
        visualizer.UpdateRender();
    }

    visualizer.DestroyVisualizerWindow();
}