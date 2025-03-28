#include "Reconstructor.h"

Reconstructor::Reconstructor(const ProgramOptions& options):
m_octree_vertices(options.max_orphan_per_voxel),
m_iterator_vertices(&m_octree_vertices),
m_octree_ball_centers(-1),
m_iterator_ball_centers(&m_octree_ball_centers),
m_mesher(&m_octree_vertices, &m_iterator_vertices, &m_octree_ball_centers, &m_iterator_ball_centers),
m_is_initialized(false),
m_radius(options.radius)
{
    unsigned int max_orphan_per_voxel = options.max_orphan_per_voxel;
    unsigned int reading_per_batch = options.reading_per_batch;
    //const bool one_mesh_policy = options.policy_main_mesh.enabled;
    //const unsigned int activation_batch_number = options.policy_main_mesh_activation_batch_number;
}

Reconstructor::~Reconstructor()
{
}

void Reconstructor::reconstruct(const std::list<Vertex>& vertices)
{
    if (!m_is_initialized)
    {
        initialize(vertices);
        m_is_initialized = true;
    }
    
    m_mesher.batchReconstruct(vertices);
}

void Reconstructor::initialize(const std::list<Vertex>& vertices)
{
    // Initialize the octree
    Point origin;
    double size;
    unsigned int depth;
    std::tie(origin, size, depth) = FileIO::originAndDepth(vertices, m_radius);
    m_mesher.initialize(origin, size, depth);
}