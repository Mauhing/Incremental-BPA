#include "Reconstructor.h"
#include "Visualizer.h"
#include <unistd.h>

Reconstructor::Reconstructor(const ProgramOptions& options):
m_octree_vertices(options.max_orphan_per_voxel),
m_iterator_vertices(&m_octree_vertices),
m_octree_ball_centers(-1),
m_iterator_ball_centers(&m_octree_ball_centers),
m_mesher(&m_octree_vertices,
        &m_iterator_vertices, 
        &m_octree_ball_centers,
        &m_iterator_ball_centers, 
        options.random_device, 
        options.seed,
        options.seed_triangles_every_batch),
m_radius(options.radius),
m_is_initialized(false),
m_should_exit(false),
m_hole_length(options.hole_length),
m_show_previous_vertices(options.show_previous_vertices),
m_save_path(options.output_file)
{    
}

Reconstructor::~Reconstructor()
{
    if (m_vis_thread.joinable()) {
        m_should_exit = true;  // Signal thread to exit
        m_vis_thread.join();   // Wait for thread to finish
    }
}

void Reconstructor::reconstruct(const Eigen::Matrix<double, 3, 4>& pose, const std::list<Vertex>& vertices )
{
    if (!m_is_initialized)
    {
        initializeOctree(vertices);
        initializeVisualization();
        m_is_initialized = true;
    }

    m_received_vertices.clear();
    for (const auto& v : vertices) {
        m_received_vertices.emplace_back(v, Eigen::Vector3d(0.0, 1.0, 0.0)); // Green
    }
    
    m_mesher.batchReconstruct(vertices);
    m_robot_pose = pose;
    
    #ifdef _DEBUG
    //std::cout << "Checking mesh integrity" << std::endl;
    //m_mesher.mesh_integrityCheck();
    #endif

    {
        std::unique_lock<std::mutex> lock(m_o3d_mesh_mutex);
        m_rendering_in_progress = true;
        m_cv_debug_visualization.notify_one();

        bool& ref = m_rendering_in_progress;  // Create a local reference
    
        auto timeout = std::chrono::milliseconds(500); // Set a timeout duration
        while (true) {
            if (m_cv_debug_visualization.wait_for(lock, timeout, [&ref]{ return !ref; })) {
                break;
            }

            if (m_should_exit) {
                break;
            }
            // Optionally, add a small sleep here to prevent busy-waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void Reconstructor::initializeOctree(const std::list<Vertex>& vertices)
{
    // Initialize the octree
    Point origin;
    double size;
    unsigned int depth;
    
    std::tie(origin, size, depth) = FileIO::originAndDepth(vertices, m_radius);
    m_mesher.initialize(origin, size, depth);
}

void Reconstructor::initializeVisualization()
{
    // Initialize the visualization
    m_vis_thread = std::thread(Visualizer::visualizationThread,
                           std::ref(m_mesher),
                           std::ref(m_o3d_mesh_mutex),
                           std::ref(m_should_exit),
                           std::ref(m_received_vertices),
                           std::ref(m_cv_debug_visualization),
                           std::ref(m_rendering_in_progress),
                           std::ref(m_robot_pose),
                           std::ref(m_hole_length),
                           std::ref(m_show_previous_vertices));
}

void Reconstructor::saveMesh()
{
    m_mesher.saveMeshAsOpen3DPLY(m_save_path);
}