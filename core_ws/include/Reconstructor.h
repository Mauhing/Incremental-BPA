#ifndef RECONSTRUCTOR_H
#define RECONSTRUCTOR_H

#include "Mesher.h"
#include "Octree.h"
#include "OctreeIterator.h"
#include "FileIO.h"
#include "ProgramOptions.hpp"
#include "Visualizer.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

class Reconstructor
{
    private:
    // Reconstruction
    OctreeVertices m_octree_vertices;
    OctreeIteratorVertices m_iterator_vertices;
    OctreeBallCenters m_octree_ball_centers;
    OctreeIteratorBallCenters m_iterator_ball_centers;
    Mesher m_mesher;
    double m_radius;
    bool m_is_initialized;
    
    // Visualization related members
    std::condition_variable m_cv_debug_visualization;
    bool m_rendering_in_progress;
    std::mutex m_o3d_mesh_mutex;
    std::atomic<bool> m_should_exit;
    Eigen::Matrix<double, 3, 4> m_robot_pose;
    std::vector<ColorVertex> m_received_vertices;
    std::thread m_vis_thread;
    double m_hole_length;
    bool m_show_previous_vertices;

    void initializeOctree(const std::list<Vertex>& vertices);
    void initializeVisualization();

    std::string m_save_path;

    public:
    // delete the default constructor
    Reconstructor() = delete;

    Reconstructor(const ProgramOptions& options);
    ~Reconstructor();

    bool shouldExit() const { return m_should_exit; }
    
    void reconstruct(const Eigen::Matrix<double, 3, 4>& pose, const std::list<Vertex>& vertices );

    void saveMesh();
};

#endif
