#include "ros_wrapper_ibpa/rosIbpaReconstructor.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ibpa_node");
    
    rosIBPA ibpa;
    
    // Spin to receive callbacks
    ros::spin();

    return 0;
} 