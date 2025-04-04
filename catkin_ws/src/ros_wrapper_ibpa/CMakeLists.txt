cmake_minimum_required(VERSION 3.0.2)
project(ros_wrapper_ibpa)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add this to find your core project headers
set(CORE_WS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../core_ws")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  rosbag
)

# Declare catkin package information
# This is used by other packages that want to use this package
catkin_package(
    INCLUDE_DIRS include     # Public header files directory
    LIBRARIES ${PROJECT_NAME} # Libraries this package provides
    CATKIN_DEPENDS roscpp    # ROS runtime dependencies
)

# Specify include directories for this package
# Both for building this package and for other packages using this package
# Add core_ws include directories
include_directories(
    include                          # ROS package headers
    ${catkin_INCLUDE_DIRS}          # ROS headers
    ${CORE_WS_PATH}/include         # Core project headers
)

# Create the main library of this package
# Built from the source files in src/
add_library(${PROJECT_NAME}
    src/rosIbpaReconstructor.cpp
)

# Create the ROS node executable
# The '_node' suffix is a common ROS naming convention
add_executable(${PROJECT_NAME}_node src/main.cpp)

# Link the node executable with required libraries
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}          # Link against our own library
    ${catkin_LIBRARIES}      # Link against ROS libraries
)