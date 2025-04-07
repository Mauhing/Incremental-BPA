#include "ros_wrapper_ibpa/rosIbpaReconstructor.h"
#include "Point.h"
#include "utilities.h"
#include <cmath>

rosIBPA::rosIBPA(const ProgramOptions &program_options) : 
has_get_points_(false), 
has_get_pose_(true), 
should_exit_(false),
ibpa_reconstructor_(program_options)
{
    program_options_ = program_options;
    batch_size_ = program_options_.reading_per_batch;

    down_sample_in_ros_ = program_options_.down_sample_in_ros;
    down_sample_in_ros_max_points_ = program_options_.down_sample_in_ros_max_points;

    // Subscribe to topics
    points_sub_ = nh_.subscribe("/depth_registered/points", 1, &rosIBPA::processPointCloud, this);
    odom_sub_ = nh_.subscribe("/rovio/odometry", 1, &rosIBPA::processOdometry, this);

    reconstruction_thread_ = std::thread(&rosIBPA::reconstruction_loop, this);

    point_buffer_.clear();
    pose_buffer_.clear();

    // Camera to IMU transform
    // qCM = [0.5027695156557828, 0.5002145141495467, 0.49690290362200085, -0.5000957491655045]  # x,y,z,w
    // MrMC = [0.04839478648122993, 0.04799781122813299, -0.009770663739786307]  # x,y,z
    // I need to ask Mohit to know how I can actually get the number from ROS, and not hard code it.
    Eigen::Quaterniond rot_q_s2b(-0.5000957491655045, 0.5027695156557828, 0.5002145141495467, 0.49690290362200085);
    Eigen::Matrix3d rot_s2b = rot_q_s2b.toRotationMatrix();

    Eigen::Vector3d tran_s2b(0.04839478648122993, 0.04799781122813299, -0.009770663739786307);

    Eigen::Matrix3d rot_b2s = rot_s2b.transpose();
    Eigen::Vector3d tran_b2s = -rot_b2s * tran_s2b;

    b2s_pose_.block<3, 3>(0, 0) = rot_b2s;
    b2s_pose_.block<3, 1>(0, 3) = tran_b2s; 
}

void rosIBPA::processPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(batch_mutex_);
    if (!has_get_points_ && has_get_pose_)
    {
        ROS_INFO("Received point cloud data");
        // Store original point cloud without filtering
        point_buffer_.push_back(*msg);
        has_get_points_ = true;
        has_get_pose_ = false;
    }
}

void rosIBPA::processOdometry(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(batch_mutex_); // Lock during modification
    if (has_get_points_ && !has_get_pose_)
    {
        ROS_INFO("Received odometry data");
        pose_buffer_.push_back(*msg);
        has_get_pose_ = true;
        has_get_points_ = false;
    }
}

static Eigen::Matrix<double, 3, 4> getTransformFromOdometry(const nav_msgs::Odometry& odom) {
    Eigen::Matrix<double, 3, 4> transform;

    // Extract rotation quaternion
    Eigen::Quaterniond rot_q(odom.pose.pose.orientation.w,
                           odom.pose.pose.orientation.x, 
                           odom.pose.pose.orientation.y,
                           odom.pose.pose.orientation.z);

    // Convert quaternion to rotation matrix
    transform.block<3,3>(0,0) = rot_q.toRotationMatrix();

    // Extract translation
    transform.block<3,1>(0,3) = Eigen::Vector3d(odom.pose.pose.position.x,
                                               odom.pose.pose.position.y,
                                               odom.pose.pose.position.z);

    return transform;
}

static void transformPointCloud(const Eigen::Matrix<double, 3, 4>& transform,
                              double& x, double& y, double& z) {
    // Extract rotation and translation
    Eigen::Matrix3d rotation = transform.block<3,3>(0,0);
    Eigen::Vector3d translation = transform.block<3,1>(0,3);

    // Create point vector
    Eigen::Vector3d point(x, y, z);

    // Transform point: R*p + t
    point = rotation * point + translation;

    // Store transformed coordinates back
    x = point.x();
    y = point.y(); 
    z = point.z();
}


static PointCloudPose processPointCloudPose(const std::vector<sensor_msgs::PointCloud2> &point_batch,
                                            const std::vector<nav_msgs::Odometry> &pose_batch,
                                            bool down_sample,
                                            int max_points,
                                            const Eigen::Matrix<double, 3, 4>& b2s_pose,
                                            const bool &random_device,
                                            const int &seed)
{
    // Store points
    std::vector<Vertex> points;
    for (size_t i = 0; i < point_batch.size(); ++i)
    {
        sensor_msgs::PointCloud2ConstIterator<float> const_iter_x(point_batch[i], "x");
        sensor_msgs::PointCloud2ConstIterator<float> const_iter_y(point_batch[i], "y");
        sensor_msgs::PointCloud2ConstIterator<float> const_iter_z(point_batch[i], "z");

        //const double position_x = pose_batch[i].pose.pose.position.x;
        //const double position_y = pose_batch[i].pose.pose.position.y;
        //const double position_z = pose_batch[i].pose.pose.position.z;

        Eigen::Matrix<double, 3, 4> current_pose;
    
        Eigen::Quaterniond orientation(pose_batch[i].pose.pose.orientation.w, 
                                       pose_batch[i].pose.pose.orientation.x, 
                                       pose_batch[i].pose.pose.orientation.y, 
                                       pose_batch[i].pose.pose.orientation.z);

        current_pose.block<3, 3>(0, 0) = orientation.toRotationMatrix();
        current_pose.block<3, 1>(0, 3) = Eigen::Vector3d(pose_batch[i].pose.pose.position.x, 
                                                         pose_batch[i].pose.pose.position.y, 
                                                         pose_batch[i].pose.pose.position.z);


        for (; const_iter_x != const_iter_x.end(); ++const_iter_x, ++const_iter_y, ++const_iter_z)
        {
            if (*const_iter_x == 0.0f || *const_iter_y == 0.0f || *const_iter_z == 0.0f)
            {
                continue;
            }

            double x = *const_iter_x;
            double y = *const_iter_y;
            double z = *const_iter_z;

            transformPointCloud(b2s_pose, x, y, z);
            transformPointCloud(current_pose, x, y, z);
            
            // Todo: I have to transform the point cloud later.
            double normal_x = pose_batch[i].pose.pose.position.x - x;
            double normal_y = pose_batch[i].pose.pose.position.y - y;
            double normal_z = pose_batch[i].pose.pose.position.z - z;

            normalize(normal_x, normal_y, normal_z);
            points.push_back(Vertex(x, y, z, normal_x, normal_y, normal_z));
        }
    } 

    if (down_sample && points.size() > max_points)
    {
        // Random shuffle
        if (random_device) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::shuffle(points.begin(), points.end(), gen); 
        } else {
            std::mt19937 gen(seed);
            std::shuffle(points.begin(), points.end(), gen);
        }
        points.resize(max_points);
    }

    size_t middle_point_idx = pose_batch.size()/2; 
    Eigen::Matrix<double, 3, 4> Eigen_pose;
    
    nav_msgs::Odometry middle_pose = pose_batch[middle_point_idx];
    Eigen::Quaterniond orientation(middle_pose.pose.pose.orientation.w, 
                                   middle_pose.pose.pose.orientation.x, 
                                   middle_pose.pose.pose.orientation.y, 
                                   middle_pose.pose.pose.orientation.z);

    Eigen_pose.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    Eigen_pose.block<3, 1>(0, 3) = Eigen::Vector3d(middle_pose.pose.pose.position.x, 
                                                   middle_pose.pose.pose.position.y, 
                                                   middle_pose.pose.pose.position.z);
    
    PointCloudPose point_cloud_pose;
    point_cloud_pose.points = points;
    point_cloud_pose.pose = Eigen_pose;
    return point_cloud_pose;
}

void rosIBPA::reconstruction_loop()
{
    while (!should_exit_)
    {
        std::vector<sensor_msgs::PointCloud2> point_batch;
        std::vector<nav_msgs::Odometry> pose_batch;

        {
            std::lock_guard<std::mutex> lock(batch_mutex_);
            if (point_buffer_.size() >= batch_size_ && pose_buffer_.size() >= batch_size_)
            {
                // Pop the first batch_size_ points and poses
                point_batch = std::vector<sensor_msgs::PointCloud2>(point_buffer_.begin(), point_buffer_.begin() + batch_size_);
                pose_batch = std::vector<nav_msgs::Odometry>(pose_buffer_.begin(), pose_buffer_.begin() + batch_size_);

                // Remove the processed points and poses
                point_buffer_.erase(point_buffer_.begin(), point_buffer_.begin() + batch_size_);
                pose_buffer_.erase(pose_buffer_.begin(), pose_buffer_.begin() + batch_size_);
            }
        }

        PointCloudPose point_cloud_pose;
        if (!point_batch.empty() && !pose_batch.empty())
        {
            point_cloud_pose = processPointCloudPose(point_batch, 
                                                pose_batch,
                                                down_sample_in_ros_, 
                                                down_sample_in_ros_max_points_,
                                                b2s_pose_,
                                                program_options_.random_device,
                                                program_options_.seed);
            point_batch.clear();
            pose_batch.clear();

            const std::list<Vertex> vertices(point_cloud_pose.points.begin(), point_cloud_pose.points.end());
            const Eigen::Matrix<double, 3, 4>& pose = point_cloud_pose.pose;

            ibpa_reconstructor_.reconstruct(pose, vertices);
        }
        
        // Sleep to prevent busy waiting
        ros::Duration(0.01).sleep();
    }
}

void rosIBPA::reconstructSurface()
{
    // Your surface reconstruction code
}

std::vector<rosPointCloudPose> rosIBPA::processBatch()
{
    ROS_INFO("=== Starting processBatch ===");
    std::vector<rosPointCloudPose> processed_pairs;
    processed_pairs.reserve(batch_size_); // Reserve space for efficiency

    for (size_t i = 0; i < batch_size_; ++i)
    {
        ROS_INFO("Processing pair %zu", i);

        // Create a new pair
        rosPointCloudPose pair;

        // Filter point cloud to 256 points
        pair.points = filterPointCloud(point_buffer_[i]);
        pair.pose = pose_buffer_[i];

        processed_pairs.push_back(pair);

        ROS_INFO("Points in filtered cloud: %d",
                 pair.points.width * pair.points.height);
    }

    ROS_INFO("=== Finished processBatch ===");
    return processed_pairs;
}

sensor_msgs::PointCloud2 rosIBPA::filterPointCloud(const sensor_msgs::PointCloud2 &input_cloud)
{
    ROS_INFO("Starting filterPointCloud");
    sensor_msgs::PointCloud2 cloud = input_cloud; // Create mutable copy

    try
    {
        // Create iterators for x, y, z
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

        ROS_INFO("Created iterators");
        std::vector<uint8_t> filtered_data;
        size_t point_count = 0;

        // Collect all non-zero points first
        std::vector<size_t> valid_indices;
        size_t point_idx = 0;

        ROS_INFO("Starting point collection");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_idx)
        {
            if (*iter_x == 0.0f && *iter_y == 0.0f && *iter_z == 0.0f)
            {
                continue;
            }
            valid_indices.push_back(point_idx);
        }
        ROS_INFO("Found %zu valid points", valid_indices.size());

        // Randomly sample exactly 256 points
        if (valid_indices.size() > 256)
        {
            std::random_shuffle(valid_indices.begin(), valid_indices.end());
            valid_indices.resize(256);
        }
        else if (valid_indices.size() < 256)
        {
            ROS_WARN("Not enough valid points: %zu", valid_indices.size());
        }

        // Create new point cloud with exactly these points
        sensor_msgs::PointCloud2 filtered_cloud; // Create new empty point cloud
        filtered_cloud.data.clear();             // Clear any existing data (safety)

        // Copy metadata from original cloud
        filtered_cloud.header = cloud.header;                                       // Copy timestamp and frame_id
        filtered_cloud.height = 1;                                                  // Unorganized cloud (1 row)
        filtered_cloud.width = valid_indices.size();                                // Number of points (256 or less)
        filtered_cloud.fields = cloud.fields;                                       // Copy point structure (x,y,z,etc.)
        filtered_cloud.is_bigendian = cloud.is_bigendian;                           // Copy byte order
        filtered_cloud.point_step = cloud.point_step;                               // Bytes per point (e.g., 32 bytes)
        filtered_cloud.row_step = filtered_cloud.width * filtered_cloud.point_step; // Bytes per row
        filtered_cloud.is_dense = false;                                            // Might have invalid points

        // Pre-allocate memory for efficiency
        filtered_cloud.data.reserve(filtered_cloud.width * filtered_cloud.point_step);

        // Add the selected points
        for (size_t idx : valid_indices)
        {
            filtered_cloud.data.insert(filtered_cloud.data.end(),
                                       cloud.data.begin() + idx * cloud.point_step,
                                       cloud.data.begin() + (idx + 1) * cloud.point_step);
        }

        point_count = valid_indices.size();

        filtered_cloud.data = filtered_data;
        filtered_cloud.width = point_count;
        filtered_cloud.row_step = filtered_cloud.width * filtered_cloud.point_step;

        return filtered_cloud;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Error in filterPointCloud: %s", e.what());
        return input_cloud; // Return original cloud on error
    }
}

// void rosIBPA::processBag(const std::string& bag_path) {
//     rosbag::Bag bag;
//     try {
//         bag.open(bag_path, rosbag::bagmode::Read);
//     } catch(rosbag::BagException& e) {
//         ROS_ERROR("Error opening bag file: %s", e.what());
//         return;
//     }
//
//     std::vector<std::string> topics;
//     topics.push_back("/depth_registered/points");
//     topics.push_back("/rovio/odometry");
//
//     rosbag::View view(bag, rosbag::TopicQuery(topics));
//
//     for(rosbag::MessageInstance const& m : view) {
//         if (m.getTopic() == "/depth_registered/points") {
//             sensor_msgs::PointCloud2::ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
//             if (pc != nullptr) {
//                 processPointCloud(pc);
//             }
//         }
//         else if (m.getTopic() == "/rovio/odometry") {
//             nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();
//             if (odom != nullptr) {
//                 processOdometry(odom);
//             }
//         }
//
//         if (has_get_points_ && has_get_pose_) {
//             ROS_INFO("Got both point cloud and odometry data, stopping...");
//             break;
//         }
//     }
//
//     bag.close();
// }