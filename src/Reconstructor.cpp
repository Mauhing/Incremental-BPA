#include "Reconstructor.h"
#include "Visualizer.h"

Reconstructor::Reconstructor(const ProgramOptions& options):
m_octree_vertices(options.max_orphan_per_voxel),
m_iterator_vertices(&m_octree_vertices),
m_octree_ball_centers(-1),
m_iterator_ball_centers(&m_octree_ball_centers),
m_mesher(&m_octree_vertices, &m_iterator_vertices, &m_octree_ball_centers, &m_iterator_ball_centers),
m_radius(options.radius),
m_is_initialized(false)
{
    unsigned int reading_per_batch = options.reading_per_batch;
    //const bool one_mesh_policy = options.policy_main_mesh.enabled;
    //const unsigned int activation_batch_number = options.policy_main_mesh_activation_batch_number;
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

    {
        std::unique_lock<std::mutex> lock(m_o3d_mesh_mutex);
        m_rendering_in_progress = true;
        m_cv_debug_visualization.notify_one();
        std::cout << "\033[33mTask in progress set to true\033[0m" << std::endl;
        std::cout << "\033[33mSignal sent from main\033[0m" << std::endl;        
        bool& ref = m_rendering_in_progress;  // Create a local reference
        m_cv_debug_visualization.wait(lock, [&ref]{ return !ref; });
        std::cout << "\033[33mSignal received at main\033[0m" << std::endl;
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
                           std::ref(m_robot_pose));
}