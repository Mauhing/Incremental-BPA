#include <mutex>
#include <condition_variable>
#include <atomic>
#include <open3d/Open3D.h>
#include "Visualizer.h"
#include "types.h"
#include <GLFW/glfw3.h>

void Visualizer::visualizationThread(
    Mesher &mesher,
    std::mutex &o3d_mesh_mutex,
    std::atomic<bool> &should_exit,
    std::vector<ColorVertex> &received_vertices,
    std::condition_variable &cv_debug_visualization,
    bool &rendering_in_progress,
    Eigen::Matrix<double, 3, 4> &robot_pose)
{
    // Create a visualizer object
    open3d::visualization::VisualizerWithKeyCallback visualizer;
    visualizer.CreateVisualizerWindow("Open3D Mesh Viewer", 800, 450);

    // Create a shared pointer to store the mesh
    auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    // Add initial mesh data
    {
        std::lock_guard<std::mutex> lock(o3d_mesh_mutex);
        renderMainMesh(mesher.getFacets(), o3d_mesh);
    }
    visualizer.AddGeometry(o3d_mesh);

    //// Register the key callback after o3d_mesh is created
    //visualizer.RegisterKeyCallback(GLFW_KEY_T,
    //    [&o3d_mesh](open3d::visualization::Visualizer* vis) -> bool {
    //        bool saved = open3d::io::WriteTriangleMeshToPLY("temp_mesh.ply", *o3d_mesh, false, false, false, false, false, true);
    //        if (saved) {
    //            std::cout << "Mesh saved to temp_mesh.ply" << std::endl;
    //        } else {
    //            std::cout << "Failed to save mesh" << std::endl;
    //        }
    //        return true;
    //    });

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
    render_option.mesh_show_back_face_ = false;
    render_option.point_size_ = 5.0;
    render_option.light_on_ = true;
    render_option.mesh_shade_option_ = open3d::visualization::RenderOption::MeshShadeOption::FlatShade;

    // Store the color we want to maintain
    const Eigen::Vector3d golden_color(1.0, 0.7, 0.0);

    // add debug vertices, debug facets, debug edges
    auto debug_facet = std::make_shared<open3d::geometry::TriangleMesh>();
    auto debug_edge = std::make_shared<open3d::geometry::LineSet>();
    auto debug_point = std::make_shared<open3d::geometry::PointCloud>();
    visualizer.AddGeometry(debug_facet);
    visualizer.AddGeometry(debug_edge);
    visualizer.AddGeometry(debug_point);

    // Set default viewpoint
     //visualizer.GetViewControl().SetFront({0, 0, -1});
     //visualizer.GetViewControl().SetLookat({0, 0, 0});
     //visualizer.GetViewControl().SetUp({0, 1, 0});
     //visualizer.GetViewControl().SetZoom(0.7);

    //std::this_thread::sleep_for(std::chrono::seconds(5));

    // add coordinate axes
    auto coordinate_axes = open3d::geometry::TriangleMesh::CreateCoordinateFrame(1.0);
    visualizer.AddGeometry(coordinate_axes);

    auto robot_pose_mesh = open3d::geometry::TriangleMesh::CreateCoordinateFrame(1.0);
    Eigen::Matrix4d previous_pose = Eigen::Matrix4d::Identity();
    visualizer.AddGeometry(robot_pose_mesh);

    std::unordered_map<unsigned int, unsigned int> f_index_2_matrix_row;
    std::unordered_map<unsigned int, unsigned int> v_index_2_matrix_row;

    bool need_2_update = false;

    bool key_operation_performed = false;

    while (!should_exit)
    {
        if (!visualizer.PollEvents())
        {                       // Window was closed
            should_exit = true; // Signal main thread to exit
            break;
        }

        if (rendering_in_progress) {            
            visualizer.RegisterKeyCallback(GLFW_KEY_T,
                [&mesher, &rendering_in_progress, &key_operation_performed](open3d::visualization::Visualizer* vis) -> bool {
                    if (rendering_in_progress) {
                        mesher.calculateMainRegion();
                        mesher.removeNonMainFacets();
                        key_operation_performed = true;
                        return true;
                    }
                    return false;
                });

            // Process any pending key operations
            visualizer.PollEvents();
            if (key_operation_performed) {
                key_operation_performed = false;
            }

            renderMainMesh(mesher.getFacets(), o3d_mesh);
            //renderIncremental(mesher.getBatchFacetsRemoved(), mesher.getBatchFacetsAdded(), f_index_2_matrix_row, v_index_2_matrix_row, o3d_mesh);

            renderReceivedVertices(received_vertices, debug_point);

            // the first boundary is the boundary with the largest square length
            std::list<Boundary> boundaries = mesher.getBoundriesPurelyFromBoarder();
            renderBoundaries(boundaries, debug_edge);

            // rotation
            Eigen::Matrix3d rotation_trans;
            rotation_trans = previous_pose.block<3, 3>(0, 0).transpose();

            // translation
            Eigen::Vector3d translation_trans;
            translation_trans = -rotation_trans * previous_pose.block<3, 1>(0, 3);

            // back to origin
            Eigen::Matrix4d back_to_origin = Eigen::Matrix4d::Identity();
            back_to_origin.block<3, 3>(0, 0) = rotation_trans;
            back_to_origin.block<3, 1>(0, 3) = translation_trans;
            robot_pose_mesh->Transform(back_to_origin);
            
            // the robot pose is 3x4 matrix

            // the robot pose is 3x4 matrix
            // the transform is 4x4 matrix
            // so we need to convert the robot pose to 4x4 matrix
            Eigen::Matrix4d robot_pose_matrix = Eigen::Matrix4d::Identity();
            robot_pose_matrix.block<3, 3>(0, 0) = robot_pose.block<3, 3>(0, 0);
            robot_pose_matrix.block<3, 1>(0, 3) = robot_pose.block<3, 1>(0, 3); 
 
            //print the robot pose matrix
            //std::cout << "Robot pose matrix:" << std::endl;
            //std::cout << robot_pose_matrix << std::endl;

            robot_pose_mesh->Transform(robot_pose_matrix);
            previous_pose = robot_pose_matrix;
            
            rendering_in_progress = false;
            need_2_update = true;
            cv_debug_visualization.notify_one(); 
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (need_2_update) {
            //Shader
            o3d_mesh->ComputeVertexNormals();
            o3d_mesh->ComputeTriangleNormals(); 

            visualizer.UpdateGeometry(o3d_mesh);
            visualizer.UpdateGeometry(debug_edge);
            visualizer.UpdateGeometry(debug_point);
            visualizer.UpdateGeometry(robot_pose_mesh);
            need_2_update = false;
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

void Visualizer::renderReceivedVertices(std::vector<ColorVertex> &received_vertices, std::shared_ptr<open3d::geometry::PointCloud> &point)
{
    point->points_.clear();
    point->colors_.clear();
    for (auto color_vertex : received_vertices)
    {
        point->points_.push_back(Eigen::Vector3d(color_vertex.vertex.x(), color_vertex.vertex.y(), color_vertex.vertex.z()));
        point->colors_.push_back(color_vertex.color);
    }
}

void Visualizer::renderBorderEdges(const Edge_star_list &border_edges, std::shared_ptr<open3d::geometry::LineSet> &line)
{
    std::unordered_map<Vertex*, int> vertex_to_index;
    int current_index = 0;
    
    // First pass - build vertex index map
    for (auto edge : border_edges) {
        if (vertex_to_index.find(edge->getSource()) == vertex_to_index.end()) {
            vertex_to_index[edge->getSource()] = current_index++;
        }
        if (vertex_to_index.find(edge->getTarget()) == vertex_to_index.end()) {
            vertex_to_index[edge->getTarget()] = current_index++;
        }
    }

    // Second pass - create lines using indices
    line->points_.clear();
    line->colors_.clear();
    line->lines_.clear();

    // Add vertices
    line->points_.resize(vertex_to_index.size());
    for (const auto& [vertex, index] : vertex_to_index) {
        line->points_[index] = Eigen::Vector3d(vertex->x(), vertex->y(), vertex->z());
    }

    // Add lines with red color
    for (auto edge : border_edges) {
        line->lines_.push_back(Eigen::Vector2i(
            vertex_to_index[edge->getSource()], 
            vertex_to_index[edge->getTarget()]));
        line->colors_.push_back(Eigen::Vector3d(1.0, 0.0, 0.0)); // Red color
    }
}

bool Visualizer::integrityCheck(const std::shared_ptr<open3d::geometry::TriangleMesh> &mesh, std::shared_ptr<open3d::geometry::LineSet> &line) {
    // Check if mesh has self-intersections


    bool has_self_intersections = mesh->IsSelfIntersecting();
    if (has_self_intersections) {
        // Clear existing line data
        line->points_.clear();
        line->lines_.clear();
        line->colors_.clear();

        // Get self-intersecting triangles
        std::vector<Eigen::Vector2i> intersecting_triangles = mesh->GetSelfIntersectingTriangles();
        
        // Print vertices of intersecting triangles
        std::cout << "Intersecting triangles vertices:" << std::endl;
        for (const auto& pair : intersecting_triangles) {
            // Print first triangle vertices
            Eigen::Vector3i tri1 = mesh->triangles_[pair[0]];
            std::cout << "Triangle " << pair[0] << ":" << std::endl;
            for (int i = 0; i < 3; i++) {
                Eigen::Vector3d v = mesh->vertices_[tri1[i]];
                std::cout << "  v" << i << " [" << tri1[i] << "]: (" << v.x() << ", " << v.y() << ", " << v.z() << ")" << std::endl;
            }
            
            // Print second triangle vertices  
            Eigen::Vector3i tri2 = mesh->triangles_[pair[1]];
            std::cout << "Triangle " << pair[1] << ":" << std::endl;
            for (int i = 0; i < 3; i++) {
                Eigen::Vector3d v = mesh->vertices_[tri2[i]];
                std::cout << "  v" << i << " [" << tri2[i] << "]: (" << v.x() << ", " << v.y() << ", " << v.z() << ")" << std::endl;
            }
            std::cout << "---" << std::endl;
        }

        
        
        // Add vertices from intersecting triangles
        std::unordered_map<int, int> vertex_map;
        size_t current_index = 0; // Start from 0 since we cleared

        // For each intersecting triangle pair
        for (const auto& pair : intersecting_triangles) {
            // Get vertices for both triangles
            for (int tri_idx : {pair[0], pair[1]}) {
                // Get the three vertex indices for this triangle
                Eigen::Vector3i triangle = mesh->triangles_[tri_idx];
                
                std::cout << "Triangle " << tri_idx << " vertices:" << std::endl;
                
                // Add each vertex if not already added
                for (int i = 0; i < 3; i++) {
                    Eigen::Vector3d vertex = mesh->vertices_[triangle[i]];
                    std::cout << "  v" << i << ": (" << vertex.x() << ", " << vertex.y() << ", " << vertex.z() << ")" << std::endl;
                    
                    if (vertex_map.find(triangle[i]) == vertex_map.end()) {
                        vertex_map[triangle[i]] = static_cast<int>(current_index++);
                        line->points_.push_back(vertex);
                    }
                }

                // Add lines forming the triangle
                for (int i = 0; i < 3; i++) {
                    line->lines_.push_back(Eigen::Vector2i(
                        vertex_map[triangle[i]], 
                        vertex_map[triangle[(i + 1) % 3]]));
                    line->colors_.push_back(Eigen::Vector3d(0.0, 0.0, 1.0)); // Blue color
                }
            }
        }

        std::cout << "Found " << intersecting_triangles.size() << " self-intersecting triangle pairs" << std::endl;
        return false;
    }
    else {
        std::cout << "\033[32mNo self-intersecting triangles found\033[0m" << std::endl;
        return true;
    }

}

void Visualizer::renderIncremental(const std::unordered_set<unsigned int> &facets_to_remove, const std::unordered_set<Facet*> &facets_to_add, std::unordered_map<unsigned int, unsigned int> &f_index_2_matrix_row, std::unordered_map<unsigned int, unsigned int> &v_index_2_matrix_row, std::shared_ptr<open3d::geometry::TriangleMesh> &O3d_mesh) {
    // Some assumptions:
    #ifdef _DEBUG
    for (unsigned int f_index : facets_to_remove) {
        for (Facet* facet_add : facets_to_add) {
            if (f_index == facet_add->getIndex()) {
                std::cerr << "Facets to remove and add has same index" << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
    }
    #endif

    // Add facets
    std::vector<Eigen::Vector3i> &matrix_triangles = O3d_mesh->triangles_;
    std::vector<Eigen::Vector3d> &matrix_vertices = O3d_mesh->vertices_;

    for (auto facet : facets_to_add) {
        // First register the point in matrix_v
        for (int i = 0; i < 3; i++) {
            Vertex *v = facet->getVertex(i);
            unsigned int v_index = v->index();
            if (v_index_2_matrix_row.find(v_index) == v_index_2_matrix_row.end()) {
                v_index_2_matrix_row[v_index] = static_cast<unsigned int>(matrix_vertices.size());
                matrix_vertices.push_back(Eigen::Vector3d(v->x(), v->y(), v->z()));
            }
        }

        // Then register the triangle in matrix_t
        unsigned int f_index = facet->getIndex();
        f_index_2_matrix_row[f_index] = static_cast<unsigned int>(matrix_triangles.size());
        matrix_triangles.push_back(Eigen::Vector3i(v_index_2_matrix_row[facet->getVertex(0)->index()],
                                                   v_index_2_matrix_row[facet->getVertex(1)->index()], 
                                                   v_index_2_matrix_row[facet->getVertex(2)->index()]));
    }
 
    // Record the row shifts
    std::unordered_map<unsigned int, unsigned int> row_shifts;
    for (unsigned int row = 0; row < matrix_triangles.size(); ++row) {
        unsigned int shift = 0;
        // How many facets are removed before this row?
        for (unsigned int f_index : facets_to_remove) {
            if (f_index_2_matrix_row.find(f_index) != f_index_2_matrix_row.end() && f_index_2_matrix_row[f_index] < row) {
                shift++;
            }
        }
        row_shifts[row] = shift;
    }

    // Create new matrix with removed rows
    std::vector<Eigen::Vector3i> new_matrix;
    new_matrix.reserve(matrix_triangles.size() - facets_to_remove.size());
    
    for (size_t row = 0; row < matrix_triangles.size(); ++row) {
        // Check if this row should be kept
        bool keep_row = true;
        for (unsigned int f_index : facets_to_remove) {
            if (f_index_2_matrix_row.find(f_index) != f_index_2_matrix_row.end() && f_index_2_matrix_row[f_index] == row) {
                keep_row = false;
                break;
            }
        }
        
        if (keep_row) {
            new_matrix.push_back(matrix_triangles[row]);
        }
    }

    O3d_mesh->triangles_ = new_matrix;

    // Update facet_index_2_matrix_row map
    for (auto it = f_index_2_matrix_row.begin(); it != f_index_2_matrix_row.end();) {
        if (std::find(facets_to_remove.begin(), facets_to_remove.end(), it->first) != facets_to_remove.end()) {
            it = f_index_2_matrix_row.erase(it);
        } else {
            it->second -= row_shifts[it->second];
            ++it;
        }
    }
}

void Visualizer::renderBoundaries(const std::list<Boundary> &boundaries, std::shared_ptr<open3d::geometry::LineSet> &line) {

    line->points_.clear();
    line->colors_.clear();
    line->lines_.clear();

    std::unordered_map<Vertex*, int> vertex_to_index;
    int current_index = 0;
    
    // First pass - build vertex index map
    for (auto boundary : boundaries) {
        for (auto edge : boundary.getEdges()) {
            if (vertex_to_index.find(edge->getSource()) == vertex_to_index.end()) {
                vertex_to_index[edge->getSource()] = current_index++;
            }
            if (vertex_to_index.find(edge->getTarget()) == vertex_to_index.end()) {
                vertex_to_index[edge->getTarget()] = current_index++;
            }
        }
    }

    // Add vertices
    line->points_.resize(vertex_to_index.size());
    for (const auto& [vertex, index] : vertex_to_index) {
        line->points_[index] = Eigen::Vector3d(vertex->x(), vertex->y(), vertex->z());
    }

    // Add lines. First boundary blue, rest red
    bool is_first_boundary = true;
    for (const auto& boundary : boundaries) {
        for (const auto& edge : boundary.getEdges()) {
            line->lines_.push_back(Eigen::Vector2i(vertex_to_index[edge->getSource()], vertex_to_index[edge->getTarget()]));
            if (is_first_boundary) {
                line->colors_.push_back(Eigen::Vector3d(0.0, 1.0, 0.0)); // Green for first boundary
            } 
            else if (boundary.getSqLength() < 10) {
                line->colors_.push_back(Eigen::Vector3d(0.0, 0.0, 1.0)); // Blue for rest
            }
            else {
                line->colors_.push_back(Eigen::Vector3d(1.0, 0.0, 0.0)); // Red for rest
            }
        }
        is_first_boundary = false;
    }
}