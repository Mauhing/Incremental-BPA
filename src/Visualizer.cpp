#include <mutex>
#include <condition_variable>
#include <atomic>
#include <open3d/Open3D.h>
#include "Visualizer.h"

void Visualizer::visualizationThread(
    Mesher &mesher,
    std::mutex &o3d_mesh_mutex,
    std::atomic<bool> &should_exit)
{
    // Create a visualizer object
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Open3D Mesh Viewer", 800, 450);

    auto device = open3d::core::Device("CUDA:0"); // Use first CUDA device
    // Check if the requested device is available
    if (device.IsAvailable())
    {
        std::cout << "\033[32mUsing GPU device: " << device.ToString() << "\033[0m" << std::endl;
    }
    else
    {
        std::cout << "\033[32mGPU not available, falling back to CPU\033[0m" << std::endl;
        device = open3d::core::Device("CPU:0");
    }

    // visualizer.GetRenderOption().mesh_show_back_face_ = true;
    // visualizer.GetRenderOption().point_size_ = 5.0;

    // Enable GPU rendering options
    auto &render_option = visualizer.GetRenderOption();
    render_option.mesh_show_back_face_ = true;
    render_option.point_size_ = 5.0;
    render_option.light_on_ = true;
    render_option.mesh_shade_option_ = open3d::visualization::RenderOption::MeshShadeOption::FlatShade;

    // Store the color we want to maintain
    const Eigen::Vector3d golden_color(1.0, 0.7, 0.0);

    // Create a shared pointer to store the mesh
    auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    // Add initial mesh data
    {
        std::lock_guard<std::mutex> lock(o3d_mesh_mutex);
        renderMainMesh(mesher.getFacets(), o3d_mesh);
    }
    visualizer.AddGeometry(o3d_mesh);

    // add debug vertices, debug facets, debug edges
    auto debug_facet = std::make_shared<open3d::geometry::TriangleMesh>();
    auto debug_edge = std::make_shared<open3d::geometry::LineSet>();
    auto debug_point = std::make_shared<open3d::geometry::PointCloud>();
    visualizer.AddGeometry(debug_facet);
    visualizer.AddGeometry(debug_edge);
    visualizer.AddGeometry(debug_point);

    // Set default viewpoint
    // visualizer.GetViewControl().SetFront({0, 0, -1});
    // visualizer.GetViewControl().SetLookat({0, 0, 0});
    // visualizer.GetViewControl().SetUp({0, 1, 0});
    // visualizer.GetViewControl().SetZoom(0.7);

    // bool first_frame = true;

    // add coordinate axes
    auto coordinate_axes = open3d::geometry::TriangleMesh::CreateCoordinateFrame(1.0);
    visualizer.AddGeometry(coordinate_axes);

    // Visualization loop
    while (!should_exit)
    {
        if (!visualizer.PollEvents())
        {                       // Window was closed
            should_exit = true; // Signal main thread to exit
            break;
        }

        // Wait for new facet with timeout
        {
            std::unique_lock<std::mutex> lock(o3d_mesh_mutex, std::defer_lock);
            if (lock.try_lock())
            { // Only proceed if we got the lock
                // if (mesher.hasNewFacet())
                if (true)
                {
                    renderMainMesh(mesher.getFacets(), o3d_mesh);
                    renderFreshVertices(mesher.getFreshVertices(), debug_point);

                    // Shader
                    // o3d_mesh->ComputeVertexNormals();
                    // o3d_mesh->ComputeTriangleNormals();

                    // is it vertex manifold?
                    //bool is_vertex_manifold = o3d_mesh->IsVertexManifold();
                    //if (!is_vertex_manifold) {
                    //    // get the non-manifold vertices
                    //    debug_point->points_.clear();
                    //    debug_point->colors_.clear();
                    //    std::vector<int> non_manifold_vertices = o3d_mesh->GetNonManifoldVertices();
                    //    // put the non-manifold vertices in debug_point
                    //    for (size_t i = 0; i < non_manifold_vertices.size(); i++) {
                    //        debug_point->points_.push_back(o3d_mesh->vertices_[non_manifold_vertices[i]]);
                    //        debug_point->colors_.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
                    //    }
                    //    std::cout << "Non-manifold vertices: " << non_manifold_vertices.size() << std::endl;
                    //}
                    visualizer.UpdateGeometry(o3d_mesh);
                    visualizer.UpdateGeometry(debug_point);
                    mesher.clearNewFacetFlag();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }

        visualizer.UpdateRender();
    }

    visualizer.DestroyVisualizerWindow();
}

void Visualizer::renderMainMesh(const Facet_star_list &facets, std::shared_ptr<open3d::geometry::TriangleMesh> &O3d_mesh)
{
    // Clear existing mesh data
    O3d_mesh->vertices_.clear();
    O3d_mesh->triangles_.clear();
    O3d_mesh->vertex_colors_.clear();

    // Create a map of vertices to their new indices
    std::unordered_map<Vertex *, int> vertex_to_index; // Changed to int
    int current_index = 0;                             // Changed to int

    for (auto facet : facets)
    {
        for (int i = 0; i < 3; i++)
        {
            Vertex *v = facet->getVertex(i);
            if (vertex_to_index.find(v) == vertex_to_index.end())
            {
                vertex_to_index[v] = current_index++;
                O3d_mesh->vertices_.push_back(Eigen::Vector3d(v->x(), v->y(), v->z()));
                O3d_mesh->vertex_colors_.push_back(Eigen::Vector3d(0.5, 0.5, 0.5));
            }
        }
    }

    // create triangles
    for (auto facet : facets)
    {
        O3d_mesh->triangles_.push_back(Eigen::Vector3i(vertex_to_index[facet->getVertex(0)], vertex_to_index[facet->getVertex(1)], vertex_to_index[facet->getVertex(2)]));
    }
}

void Visualizer::renderDebugFacets(const std::vector<ColorFacetPtr> &color_facets, std::shared_ptr<open3d::geometry::TriangleMesh> &O3d_mesh)
{
    // Clear existing mesh data
    O3d_mesh->vertices_.clear();
    O3d_mesh->triangles_.clear();
    O3d_mesh->vertex_colors_.clear();

    // Create a map of vertices to their new indices
    std::unordered_map<Vertex *, int> vertex_to_index; // Changed to int
    int current_index = 0;                             // Changed to int

    // First pass: collect unique vertices and assign new indices
    // color is golden
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
            else
            {
                O3d_mesh->vertex_colors_[vertex_to_index[v]] = color_facet.color;
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

void Visualizer::renderDebugEdges(const std::vector<ColorEdgePtr> &color_edges, std::shared_ptr<open3d::geometry::LineSet> &O3d_line)
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

void Visualizer::renderDebugVertices(const std::vector<ColorVertexPtr> &color_vertices, std::shared_ptr<open3d::geometry::PointCloud> &O3d_point)
{
    O3d_point->points_.clear();
    O3d_point->colors_.clear();

    for (auto color_vertex : color_vertices)
    {
        O3d_point->points_.push_back(Eigen::Vector3d(color_vertex.vertex->x(), color_vertex.vertex->y(), color_vertex.vertex->z()));
        O3d_point->colors_.push_back(color_vertex.color);
    }
}

void Visualizer::renderFreshVertices(const std::vector<Vertex *> &vertices, std::shared_ptr<open3d::geometry::PointCloud> &O3d_point)
{
    O3d_point->points_.clear();
    O3d_point->colors_.clear();

    for (auto vertex : vertices)
    {
        O3d_point->points_.push_back(Eigen::Vector3d(vertex->x(), vertex->y(), vertex->z()));
        O3d_point->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    }
}

void Visualizer::renderDebugVertices(const std::vector<Eigen::Vector3d> &vertices, const Eigen::Vector3d &color, std::shared_ptr<open3d::geometry::PointCloud> &point)
{
    for (auto vertex : vertices)
    {
        point->points_.push_back(vertex);
        point->colors_.push_back(color);
    }
}