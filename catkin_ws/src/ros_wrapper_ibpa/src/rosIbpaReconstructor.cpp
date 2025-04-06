#include "ros_wrapper_ibpa/rosIbpaReconstructor.h"

rosIBPA::rosIBPA(const ProgramOptions& program_options) : has_get_points_(false), has_get_pose_(true), should_exit_(false) {
    program_options_ = program_options;
    batch_size_ = program_options_.reading_per_batch;

    down_sample_in_ros_ = program_options_.down_sample_in_ros;
    down_sample_in_ros_max_points_ = program_options_.down_sample_in_ros_max_points;

    // Subscribe to topics
    points_sub_ = nh_.subscribe("/depth_registered/points", 1, &rosIBPA::processPointCloud, this);
    odom_sub_ = nh_.subscribe("/rovio/odometry", 1, &rosIBPA::processOdometry, this);
 
    reconstruction_thread_ = std::thread(&rosIBPA::reconstruction_loop, this);
}

void rosIBPA::processPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(batch_mutex_);
    if (!has_get_points_ && has_get_pose_) {
        ROS_INFO("Received point cloud data");
        // Store original point cloud without filtering
        point_batch_.push_back(*msg);
        has_get_points_ = true;
        has_get_pose_ = false;
    }
}

void rosIBPA::processOdometry(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(batch_mutex_);  // Lock during modification
    if (has_get_points_ && !has_get_pose_) {
        ROS_INFO("Received odometry data");
        pose_batch_.push_back(*msg);
        has_get_pose_ = true;
        has_get_points_ = false;

        if (point_batch_.size() == batch_size_ && pose_batch_.size() == batch_size_) {
            processBatch();
            point_batch_.clear();
            pose_batch_.clear();
        }
    }
}


void rosIBPA::reconstruction_loop() {
    while (!should_exit_) {
        std::vector<rosPointCloudPose> batch;
        
        // Get batch when ready
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (data_queue_.size() >= batch_size_) {
                // Get batch from queue...
            }
        }

        if (!batch.empty()) {
            reconstructSurface();
        }

        // Sleep to prevent busy waiting
        ros::Duration(0.01).sleep();
    }
}

void rosIBPA::reconstructSurface() {
    // Your surface reconstruction code
}

std::vector<rosPointCloudPose> rosIBPA::processBatch() {
    ROS_INFO("=== Starting processBatch ===");
    std::vector<rosPointCloudPose> processed_pairs;
    processed_pairs.reserve(batch_size_);  // Reserve space for efficiency

    for (size_t i = 0; i < batch_size_; ++i) {
        ROS_INFO("Processing pair %zu", i);
        
        // Create a new pair
        rosPointCloudPose pair;
        
        // Filter point cloud to 256 points
        pair.points = filterPointCloud(point_batch_[i]);
        pair.pose = pose_batch_[i];
        
        processed_pairs.push_back(pair);
        
        ROS_INFO("Points in filtered cloud: %d", 
                    pair.points.width * pair.points.height);
    }

    ROS_INFO("=== Finished processBatch ===");
    return processed_pairs;
}

sensor_msgs::PointCloud2 rosIBPA::filterPointCloud(const sensor_msgs::PointCloud2& input_cloud) {
    ROS_INFO("Starting filterPointCloud");
    sensor_msgs::PointCloud2 cloud = input_cloud;  // Create mutable copy
    
    try {
        // Create iterators for x, y, z
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        ROS_INFO("Created iterators");
        std::vector<uint8_t> filtered_data;
        size_t point_count = 0;

        // Collect all non-zero points first
        std::vector<size_t> valid_indices;
        size_t point_idx = 0;
        
        ROS_INFO("Starting point collection");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_idx) {
            if (*iter_x == 0.0f && *iter_y == 0.0f && *iter_z == 0.0f) {
                continue;
            }
            valid_indices.push_back(point_idx);
        }
        ROS_INFO("Found %zu valid points", valid_indices.size());

        // Randomly sample exactly 256 points
        if (valid_indices.size() > 256) {
            std::random_shuffle(valid_indices.begin(), valid_indices.end());
            valid_indices.resize(256);
        } else if (valid_indices.size() < 256) {
            ROS_WARN("Not enough valid points: %zu", valid_indices.size());
        }

        // Create new point cloud with exactly these points
        sensor_msgs::PointCloud2 filtered_cloud;  // Create new empty point cloud
        filtered_cloud.data.clear();             // Clear any existing data (safety)

        // Copy metadata from original cloud
        filtered_cloud.header = cloud.header;    // Copy timestamp and frame_id
        filtered_cloud.height = 1;               // Unorganized cloud (1 row)
        filtered_cloud.width = valid_indices.size();  // Number of points (256 or less)
        filtered_cloud.fields = cloud.fields;    // Copy point structure (x,y,z,etc.)
        filtered_cloud.is_bigendian = cloud.is_bigendian;  // Copy byte order
        filtered_cloud.point_step = cloud.point_step;      // Bytes per point (e.g., 32 bytes)
        filtered_cloud.row_step = filtered_cloud.width * filtered_cloud.point_step;  // Bytes per row
        filtered_cloud.is_dense = false;         // Might have invalid points

        // Pre-allocate memory for efficiency
        filtered_cloud.data.reserve(filtered_cloud.width * filtered_cloud.point_step);

        // Add the selected points
        for (size_t idx : valid_indices) {
            filtered_cloud.data.insert(filtered_cloud.data.end(),
                                    cloud.data.begin() + idx * cloud.point_step,
                                    cloud.data.begin() + (idx + 1) * cloud.point_step);
        }

        point_count = valid_indices.size();

        filtered_cloud.data = filtered_data;
        filtered_cloud.width = point_count;
        filtered_cloud.row_step = filtered_cloud.width * filtered_cloud.point_step;

        return filtered_cloud;
    } catch (const std::exception& e) {
        ROS_ERROR("Error in filterPointCloud: %s", e.what());
        return input_cloud;  // Return original cloud on error
    }
}

void rosIBPA::processBag(const std::string& bag_path) {
    rosbag::Bag bag;
    try {
        bag.open(bag_path, rosbag::bagmode::Read);
    } catch(rosbag::BagException& e) {
        ROS_ERROR("Error opening bag file: %s", e.what());
        return;
    }

    std::vector<std::string> topics;
    topics.push_back("/depth_registered/points");
    topics.push_back("/rovio/odometry");

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(rosbag::MessageInstance const& m : view) {
        if (m.getTopic() == "/depth_registered/points") {
            sensor_msgs::PointCloud2::ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
            if (pc != nullptr) {
                processPointCloud(pc);
            }
        }
        else if (m.getTopic() == "/rovio/odometry") {
            nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();
            if (odom != nullptr) {
                processOdometry(odom);
            }
        }

        if (has_get_points_ && has_get_pose_) {
            ROS_INFO("Got both point cloud and odometry data, stopping...");
            break;
        }
    }

    bag.close();
}