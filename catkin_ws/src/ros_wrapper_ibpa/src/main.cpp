#include "ros_wrapper_ibpa/rosIbpaReconstructor.h"
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <open3d/Open3D.h>

#include "Octree.h"
#include "OctreeIterator.h"
#include "utilities.h"
#include "Vertex.h"
#include "Mesher.h"
#include "FileIO.h"
#include "types.h"
#include "ProgramOptions.hpp"
#include "Visualizer.h"
#include "Reconstructor.h"

std::condition_variable cv_debug_visualization;
bool rendering_in_progress(false);
std::mutex o3d_mesh_mutex;
std::atomic<bool> should_exit(false);
Eigen::Matrix<double, 3, 4> robot_pose;

int main(int argc, char** argv) {
    ros::init(argc, argv, "ibpa_node");
    ros::NodeHandle nh("~");

    std::string config_path;
    if (!nh.getParam("config_path", config_path)) {
        ROS_ERROR("Failed to get config_path parameter");
        return 1;
    }
    
    ROS_INFO("Got config path: %s", config_path.c_str()); 

    ProgramOptions program_options = parseConfigFile(config_path);

    ros::Duration(1.0).sleep();
    rosIBPA ibpa(program_options);
    
    // Spin to receive callbacks
    ros::spin();

    //ros::Rate rate(100); // 100Hz or adjust as needed
    //while (ros::ok() && !ibpa.shouldExit()) {
    //    ros::spinOnce();
    //    rate.sleep();
    //}

    ros::shutdown();
    return 0;
} 