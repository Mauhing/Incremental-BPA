#include "ros_wrapper_ibpa/rosIbpaReconstructor.h"
#include "Point.h"
#include "utilities.h"
#include <cmath>
#include <chrono>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

rosIBPA::rosIBPA(const ProgramOptions &program_options) : 
has_get_points_(false), 
has_get_pose_(true), 
should_exit_(false),
ibpa_reconstructor_(program_options),
tf_buffer_(),
tf_listener_(tf_buffer_)
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

    // Try to get transform between camera frame and IMU frame
    Eigen::Matrix4d b2s_pose_matrix = Eigen::Matrix4d::Identity();
    
    try {
        // Wait for transform to be available (timeout after 5 seconds)
        geometry_msgs::TransformStamped transform_stamped = 
            tf_buffer_.lookupTransform("imu_frame", "camera_frame", ros::Time(0), ros::Duration(5.0));
        
        // Convert to Eigen
        Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform_stamped);
        b2s_pose_matrix = eigen_transform.matrix();
        
        ROS_INFO("Successfully loaded camera-to-IMU transform from TF");
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Could not get transform from TF: %s. Using hardcoded values.", ex.what());
        
        // Fall back to your hardcoded values
        Eigen::Quaterniond rot_q_s2b(-0.5000957491655045, 0.5027695156557828, 0.5002145141495467, 0.49690290362200085);
        Eigen::Matrix3d rot_s2b = rot_q_s2b.toRotationMatrix();
        Eigen::Vector3d tran_s2b(0.04839478648122993, 0.04799781122813299, -0.009770663739786307);
        Eigen::Matrix3d rot_b2s = rot_s2b.transpose();
        Eigen::Vector3d tran_b2s = -rot_b2s * tran_s2b;
        
        b2s_pose_matrix.block<3, 3>(0, 0) = rot_b2s;
        b2s_pose_matrix.block<3, 1>(0, 3) = tran_b2s;
    }
    
    // Use the matrix (either from TF or hardcoded)
    b2s_pose_ = b2s_pose_matrix;
}

rosIBPA::~rosIBPA()
{
    ROS_INFO("rosIBPA destructor called");
    should_exit_ = true;
    if (reconstruction_thread_.joinable()) {
        reconstruction_thread_.join();
    }
    ROS_INFO("rosIBPA destructor finished");
}

void rosIBPA::processPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(batch_mutex_);
    if (!has_get_points_ && has_get_pose_)
    {
        //ROS_INFO("Received point cloud data");
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
        //ROS_INFO("Received odometry data");
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

static void transformPointCloud(const Eigen::Matrix<double, 4, 4>& transform,
                              double& x, double& y, double& z) {
    // Extract rotation and translation
    //Eigen::Matrix3d rotation = transform.block<3,3>(0,0);
    //Eigen::Vector3d translation = transform.block<3,1>(0,3);

    // Create point vector
    Eigen::Vector4d point(x, y, z, 1.0);

    // Transform point: R*p + t
    //point = rotation * point + translation;
    
    point = transform * point;

    // Store transformed coordinates back
    x = point[0];
    y = point[1]; 
    z = point[2];
}

static Eigen::Matrix4Xd pointCloudToEigenMatrix(const sensor_msgs::PointCloud2& cloud) {
    // Determine the number of points
    size_t num_points = cloud.width * cloud.height;
     
    // Initialize the Eigen matrix
    Eigen::Matrix4Xd eigen_matrix(4, num_points); // It is column major by default

    // Create iterators for x, y, z
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    // Iterate over the point cloud and populate the matrix
    int column_reduce = 0;
    size_t j = 0;
    for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
        if (*iter_x == 0.0 && *iter_y == 0.0 && *iter_z == 0.0)
        {
            column_reduce++;
            continue;
        }
        eigen_matrix(0, j) = static_cast<double>(*iter_x);
        eigen_matrix(1, j) = static_cast<double>(*iter_y);
        eigen_matrix(2, j) = static_cast<double>(*iter_z);
        eigen_matrix(3, j) = 1.0;
        j++;
    }
    eigen_matrix.conservativeResize(4, num_points - column_reduce);
    return eigen_matrix;
} 

static Eigen::Matrix4d odometryToMatrix4d(const nav_msgs::Odometry& odom) {
    // Extract position
    Eigen::Vector3d position(odom.pose.pose.position.x,
                             odom.pose.pose.position.y,
                             odom.pose.pose.position.z);

    // Extract orientation (quaternion)
    Eigen::Quaterniond orientation(odom.pose.pose.orientation.w,
                                   odom.pose.pose.orientation.x,
                                   odom.pose.pose.orientation.y,
                                   odom.pose.pose.orientation.z);

    // Normalize the quaternion to ensure it's a valid rotation
    orientation.normalize();

    // Create the 4x4 transformation matrix
    Eigen::Matrix4d transformation_matrix;

    // Set the rotation part (top-left 3x3)
    transformation_matrix.block<3, 3>(0, 0) = orientation.toRotationMatrix();

    // Set the translation part (top-right 3x1)
    transformation_matrix.block<3, 1>(0, 3) = position;

    transformation_matrix.block<1, 4>(3, 0) << 0, 0, 0, 1;

    return transformation_matrix;
}

static void transformPointCloud(const Eigen::Matrix4d &transform, Eigen::Matrix4Xd &point_cloud) {
    point_cloud = transform * point_cloud;
}

static Eigen::Matrix3Xd randomSelectPoints(const Eigen::Matrix3Xd &point_cloud,
                                           const int &max_points_per_batch,
                                           const bool &random_device,
                                           const int &seed) {
    std::vector<int> indices(point_cloud.cols());
    std::iota(indices.begin(), indices.end(), 0);
    std::mt19937 gen(random_device ? std::random_device{}() : seed);
    std::shuffle(indices.begin(), indices.end(), gen);
    indices.resize(max_points_per_batch);

    Eigen::Matrix3Xd selected_points(3, max_points_per_batch);
    for (int j = 0; j < max_points_per_batch; ++j) {
        selected_points.col(j) = point_cloud.col(indices[j]);
    }
    return selected_points;
}

static PointCloudPose processPointCloudPose(const std::vector<sensor_msgs::PointCloud2> &point_batch,
                                                const std::vector<nav_msgs::Odometry> &pose_batch,
                                                bool down_sample,
                                                int max_points,
                                                const Eigen::Matrix<double, 4, 4>& b2s_pose,
                                                const bool &random_device,
                                                const int &seed)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    ROS_INFO("Starting point cloud processing...");

    std::list<Eigen::Matrix3Xd> all_points;
    std::list<Eigen::Matrix3Xd> all_normals;
    int batch_size = static_cast<int>(point_batch.size());
    int max_points_per_batch = static_cast<int>(max_points / batch_size);

    for (size_t i = 0; i < point_batch.size(); ++i)
    {
        Eigen::Matrix4Xd point_cloud_homogeneous = pointCloudToEigenMatrix(point_batch[i]);
        Eigen::Matrix4d world2body = odometryToMatrix4d(pose_batch[i]);

        transformPointCloud(b2s_pose,   point_cloud_homogeneous);
        transformPointCloud(world2body, point_cloud_homogeneous);

        // Dehomogenize the point cloud
        Eigen::Matrix3Xd point_cloud = point_cloud_homogeneous.topRows(3);

        if (down_sample) {
            point_cloud = randomSelectPoints(point_cloud, max_points_per_batch, random_device, seed);
        }

        Eigen::Matrix3Xd point_cloud_normal(3, point_cloud.cols());
        for (size_t j = 0; j < point_cloud.cols(); ++j)
        {
            Eigen::Vector3d normal  = world2body.block<3, 1>(0, 3) - point_cloud.col(j);
            normal.normalize();
            point_cloud_normal.col(j) = normal;
        }
        
        all_points.push_back(point_cloud);
        all_normals.push_back(point_cloud_normal);
    }

    std::vector<Vertex> flat_points;
    for (auto points_it = all_points.begin(), normals_it = all_normals.begin(); 
     points_it != all_points.end(); 
     ++points_it, ++normals_it)
    {
        for(int j = 0; j < points_it->cols(); ++j)
        {
            flat_points.emplace_back(points_it->col(j)[0], points_it->col(j)[1], points_it->col(j)[2],
                                normals_it->col(j)[0], normals_it->col(j)[1], normals_it->col(j)[2]);
        }
    }

    size_t middle_point_idx = pose_batch.size() / 2 + 1;
    Eigen::Matrix<double, 3, 4> Eigen_pose;
    
    nav_msgs::Odometry middle_pose = pose_batch[middle_point_idx];
    Eigen::Quaterniond middle_orientation(middle_pose.pose.pose.orientation.w, 
                                          middle_pose.pose.pose.orientation.x, 
                                          middle_pose.pose.pose.orientation.y, 
                                          middle_pose.pose.pose.orientation.z);

    Eigen_pose.block<3, 3>(0, 0) = middle_orientation.toRotationMatrix();
    Eigen_pose.block<3, 1>(0, 3) = Eigen::Vector3d(middle_pose.pose.pose.position.x, 
                                                   middle_pose.pose.pose.position.y, 
                                                   middle_pose.pose.pose.position.z);

    PointCloudPose point_cloud_pose;
    point_cloud_pose.points = std::move(flat_points);
    point_cloud_pose.pose = Eigen_pose;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    ROS_INFO("Total time taken to process point cloud: %ld ms", duration.count());

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
                ROS_INFO("Processing batch %zu", point_buffer_.size());
                ROS_INFO("We need batch size %zu", batch_size_);
                auto start_time = std::chrono::high_resolution_clock::now();
                // Pop the first batch_size_ points and poses
                point_batch = std::vector<sensor_msgs::PointCloud2>(point_buffer_.begin(), point_buffer_.begin() + batch_size_);
                pose_batch = std::vector<nav_msgs::Odometry>(pose_buffer_.begin(), pose_buffer_.begin() + batch_size_);

                // Remove the processed points and poses
                point_buffer_.erase(point_buffer_.begin(), point_buffer_.begin() + batch_size_);
                pose_buffer_.erase(pose_buffer_.begin(), pose_buffer_.begin() + batch_size_);
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                ROS_INFO("Time taken to save and remove batch: %ld ms", duration.count());
            }
        }

        PointCloudPose point_cloud_pose;
        if (!point_batch.empty() && !pose_batch.empty())
        {
            // time start
            point_cloud_pose = processPointCloudPose(point_batch, 
                                                pose_batch,
                                                down_sample_in_ros_, 
                                                down_sample_in_ros_max_points_,
                                                b2s_pose_,
                                                program_options_.random_device,
                                                program_options_.seed);

            const std::list<Vertex> vertices(point_cloud_pose.points.begin(), point_cloud_pose.points.end());
            const Eigen::Matrix<double, 3, 4>& pose = point_cloud_pose.pose;

            auto start_time = std::chrono::high_resolution_clock::now();
            ibpa_reconstructor_.reconstruct(pose, vertices);
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            ROS_INFO("Time taken to reconstruct: %ld ms", duration.count());
        }

        if (ibpa_reconstructor_.shouldExit()) {
            should_exit_ = true;
            break;
        } 
        ros::Duration(0.01).sleep();
    }
    ROS_INFO("Reconstruction loop exiting.");
}

bool rosIBPA::shouldExit() const
{
    return should_exit_;
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

void rosIBPA::saveMesh()
{
    ibpa_reconstructor_.saveMesh();
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