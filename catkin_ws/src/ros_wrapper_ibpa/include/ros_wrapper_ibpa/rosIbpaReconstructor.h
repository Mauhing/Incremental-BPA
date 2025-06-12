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

#ifndef ROS_IBPA_RECONSTRUCTOR_H
#define ROS_IBPA_RECONSTRUCTOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <mutex>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <algorithm>
#include <queue>
#include <thread>
#include "ProgramOptions.hpp"
#include "Eigen/Dense"
#include <vector>
#include "Vertex.h"
#include "Reconstructor.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

struct rosPointCloudPose {
    sensor_msgs::PointCloud2 points;    // 256 points
    nav_msgs::Odometry pose;            // Associated pose
};

struct PointCloudPose {
    Eigen::Matrix<double, 3, 4> pose;
    std::vector<Vertex> points;
};

class rosIBPA {
private:
    ros::NodeHandle nh_;
    ros::Subscriber points_sub_;
    ros::Subscriber odom_sub_;
    std::queue<rosPointCloudPose> data_queue_;
    std::thread reconstruction_thread_;
    bool has_get_points_;
    bool has_get_pose_;
    bool should_exit_;

    // Add containers for batches
    std::vector<sensor_msgs::PointCloud2> point_buffer_;
    std::vector<nav_msgs::Odometry> pose_buffer_;
    std::mutex batch_mutex_;  // Add mutex for thread safety

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;    

    ProgramOptions program_options_;
    size_t batch_size_; // This should be set up by a config file later.
    bool down_sample_in_ros_;
    int down_sample_in_ros_max_points_;

    Eigen::Matrix<double, 4, 4> b2s_pose_;

    Reconstructor ibpa_reconstructor_;

public:
    rosIBPA(const ProgramOptions& program_options);

    ~rosIBPA();

    void processPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void processOdometry(const nav_msgs::Odometry::ConstPtr& msg);

    bool shouldExit() const;

    void saveMesh();

private:

    void reconstruction_loop();

    void reconstructSurface();

    std::vector<rosPointCloudPose> processBatch();

    sensor_msgs::PointCloud2 filterPointCloud(const sensor_msgs::PointCloud2& input_cloud);
};

#endif // ROS_IBPA_RECONSTRUCTOR_H