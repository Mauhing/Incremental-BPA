/**
 * @copyright This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * @author Mau Hing Yip mauhingyip@hotmail.com
 * @date 2025-06-12
 */

#include "ros_wrapper_ibpa/rosIbpaReconstructor.h"
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <open3d/Open3D.h>
#include <ros/ros.h>

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
    //ros::spin();

    ros::Rate rate(100); // 100Hz or adjust as needed
    while (ros::ok() && !ibpa.shouldExit()) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Shutdown requested");
    ibpa.saveMesh();
 
    ros::shutdown(); // Cleanly shut down ROS
    ROS_INFO("ROS shutdown complete.");

    return 0;
} 