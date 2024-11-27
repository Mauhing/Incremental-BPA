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
) 
{
    // Create a visualizer object
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Open3D Mesh Viewer", 1600, 900);
    visualizer.GetRenderOption().mesh_show_back_face_ = true;
    visualizer.GetRenderOption().point_size_ = 5.0;

    // Store the color we want to maintain
    const Eigen::Vector3d golden_color(1.0, 0.7, 0.0);

    // Create a shared pointer to store the mesh
    auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    auto o3d_line = std::make_shared<open3d::geometry::LineSet>();
    auto o3d_point = std::make_shared<open3d::geometry::PointCloud>();
    // Add initial mesh data
    {
        std::lock_guard<std::mutex> lock(o3d_mesh_mutex);

        std::vector<ColorFacet> color_facets = mesher.getFacetsToRender();
        renderFacets(color_facets, o3d_mesh);
        //o3d_mesh->vertex_colors_.resize(o3d_mesh->vertices_.size(), Eigen::Vector3d(1.0, 0.7, 0.0));
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

    // add debug facets, line, point
    //auto debug_facets = std::make_shared<open3d::geometry::TriangleMesh>();
    //auto debug_line = std::make_shared<open3d::geometry::LineSet>();
    //auto debug_point = std::make_shared<open3d::geometry::PointCloud>();

    // Visualization loop
    while (!should_exit) {
        if (!visualizer.PollEvents()) {  // Window was closed
            should_exit = true;  // Signal main thread to exit
            break;
        }

        // Wait for new facet with timeout
        {
            //std::unique_lock<std::mutex> lock(o3d_mesh_mutex);
            //vis_cv.wait_for(lock, 
            //    std::chrono::milliseconds(16),  // Short timeout for responsiveness
            //    [&mesher]{ return mesher.hasNewFacet(); });

            //if (mesher.hasNewFacet()) {
            //    mesher.renderIntoOpen3D(*o3d_mesh);
            //    o3d_mesh->vertex_colors_.clear();
            //    o3d_mesh->vertex_colors_.resize(o3d_mesh->vertices_.size(), golden_color);
            //    o3d_mesh->ComputeVertexNormals();
            //    o3d_mesh->ComputeTriangleNormals();
            //    visualizer.UpdateGeometry(o3d_mesh);
            //    mesher.clearNewFacetFlag();
            //    std::this_thread::sleep_for(std::chrono::milliseconds(1));
            //}
            std::unique_lock<std::mutex> lock(o3d_mesh_mutex, std::defer_lock);
            if (lock.try_lock()) {  // Only proceed if we got the lock
                if (mesher.hasNewFacet()) 
                {
                    //std::cout << "Update mesh: Thread " << std::this_thread::get_id() <<  std::endl;
                    // print system time 
                    //auto now = std::clock();
                    //std::cout << "CPU time: " << static_cast<double>(now) / CLOCKS_PER_SEC << " seconds" << std::endl;
                    //std::vector<ColorFacet> color_facets = mesher.getFacetsToRender();
                    std::vector<ColorFacet> color_facets = mesher.getFacetsToRender();
                    renderFacets(color_facets, o3d_mesh);
                    o3d_mesh->ComputeVertexNormals();
                    o3d_mesh->ComputeTriangleNormals();
                    visualizer.UpdateGeometry(o3d_mesh);
                    mesher.clearNewFacetFlag();
                }
            }
        }
        
        visualizer.UpdateRender();
    }

    visualizer.DestroyVisualizerWindow();
}

void Visualizer::renderFacets(const std::vector<ColorFacet> &color_facets, std::shared_ptr<open3d::geometry::TriangleMesh>& O3d_mesh)
 {
    //auto O3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();

    // Clear existing mesh data
    O3d_mesh->vertices_.clear();
    O3d_mesh->triangles_.clear();

    // Create a map of vertices to their new indices
    std::unordered_map<Vertex *, int> vertex_to_index; // Changed to int
    int current_index = 0;                             // Changed to int

    // First pass: collect unique vertices and assign new indices
    // color is golden
    const Eigen::Vector3d golden_color(1.0, 0.7, 0.0);
    for (auto color_facet : color_facets)
    {
        for (int i = 0; i < 3; i++)
        {
            Vertex *v = color_facet.facet->getVertex(i);
            if (vertex_to_index.find(v) == vertex_to_index.end())
            {
                vertex_to_index[v] = current_index++;
                O3d_mesh->vertices_.push_back(
                    Eigen::Vector3d(v->x(), v->y(), v->z()));
                O3d_mesh->vertex_colors_.push_back(color_facet.color);
            }
        }
    }

    // Second pass: create triangles using the new indices
    for (auto color_facet : color_facets)
    {
        O3d_mesh->triangles_.push_back(
            Eigen::Vector3i(
                vertex_to_index[color_facet.facet->getVertex(0)],
                vertex_to_index[color_facet.facet->getVertex(1)],
                vertex_to_index[color_facet.facet->getVertex(2)]));
    }

}

void Visualizer::renderEdges(const std::vector<ColorEdge> &color_edges, std::shared_ptr<open3d::geometry::LineSet>& O3d_line)
{
    O3d_line->points_.clear();
    O3d_line->lines_.clear();
    O3d_line->colors_.clear();
    
    std::unordered_map<Vertex *, int> vertex_to_index; // Changed to int 
    int current_index = 0;
    for (auto color_edge : color_edges)
    {
        if (vertex_to_index.find(color_edge.edge->getSource()) == vertex_to_index.end())
        {
            vertex_to_index[color_edge.edge->getSource()] = current_index++;
        }
        if (vertex_to_index.find(color_edge.edge->getTarget()) == vertex_to_index.end())
        {
            vertex_to_index[color_edge.edge->getTarget()] = current_index++;
        }
    }

    // add lines
    for (auto color_edge : color_edges)
    {
        O3d_line->lines_.push_back(Eigen::Vector2i(vertex_to_index[color_edge.edge->getSource()], vertex_to_index[color_edge.edge->getTarget()]));
        O3d_line->colors_.push_back(color_edge.color);
    }
}

void Visualizer::renderVertices(const std::vector<ColorVertex> &color_vertices, std::shared_ptr<open3d::geometry::PointCloud>& O3d_point)
{
    //auto O3d_point = std::make_shared<open3d::geometry::PointCloud>();
    O3d_point->points_.clear();
    O3d_point->colors_.clear();

    for (auto color_vertex : color_vertices)
    {
        O3d_point->points_.push_back(Eigen::Vector3d(color_vertex.vertex->x(), color_vertex.vertex->y(), color_vertex.vertex->z()));
        O3d_point->colors_.push_back(color_vertex.color);
    }
}