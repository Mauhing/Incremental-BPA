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

struct PointCloudPose {
    sensor_msgs::PointCloud2 points;    // 256 points
    nav_msgs::Odometry pose;            // Associated pose
};

class rosIBPA {
private:
    ros::NodeHandle nh_;
    ros::Subscriber points_sub_;
    ros::Subscriber odom_sub_;
    std::queue<PointCloudPose> data_queue_;
    std::mutex queue_mutex_;
    std::thread reconstruction_thread_;
    bool has_get_points_;
    bool has_get_pose_;
    bool should_exit_;

    // Add containers for batches
    std::vector<sensor_msgs::PointCloud2> point_batch_;
    std::vector<nav_msgs::Odometry> pose_batch_;
    const size_t BATCH_SIZE = 20; // This should be set up by a config file later.
    std::mutex batch_mutex_;  // Add mutex for thread safety

public:
    rosIBPA();

    void processPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void processOdometry(const nav_msgs::Odometry::ConstPtr& msg);

    /*
    * @brief Process a bag file offline
    * @param bag_path The path to the bag file
    */
    void processBag(const std::string& bag_path);

private:

    void reconstruction_loop();

    void reconstructSurface();

    std::vector<PointCloudPose> processBatch();

    sensor_msgs::PointCloud2 filterPointCloud(const sensor_msgs::PointCloud2& input_cloud);
};