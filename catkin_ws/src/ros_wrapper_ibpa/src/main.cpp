#include "ros_wrapper_ibpa/rosIbpaReconstructor.h"
#include "ProgramOptions.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ibpa_node");
    ros::NodeHandle nh("~");

    std::string config_path;
    if (!nh.getParam("config_path", config_path)) {
        ROS_ERROR("Failed to get config_path parameter");
        return 1;
    }
    
    ROS_INFO("Got config path: %s", config_path.c_str());    //Sleep for 1 second

    ProgramOptions program_options = parseConfigFile(config_path);

    ros::Duration(1.0).sleep();
    rosIBPA ibpa;
    
    // Spin to receive callbacks
    ros::spin();

    return 0;
} 