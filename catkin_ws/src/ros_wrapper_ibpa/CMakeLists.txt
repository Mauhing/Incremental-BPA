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

# First compile yaml-cpp
set(YAML_CPP_DIR "${CORE_WS_PATH}/third_party/yaml-cpp-0.8.0")
set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "yaml-cpp build shared library" FORCE)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "yaml-cpp build tests" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "yaml-cpp build tools" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "yaml-cpp build contrib" FORCE)

# Build yaml-cpp with ABI=0
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
add_subdirectory(${YAML_CPP_DIR} EXCLUDE_FROM_ALL)

# Update the message to show the correct library name based on build type
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "YAML library location: ${YAML_CPP_DIR}/lib$<$<CONFIG:Debug>:yaml-cppd>$<$<NOT:$<CONFIG:Debug>>:yaml-cpp>.a")

if(BUILD_DEBUG)
    set(CMAKE_BUILD_TYPE Debug)
    # Add debug symbols
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
    # Disable optimizations in Debug mode
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
    # Add _DEBUG definition for Debug mode
    add_definitions(-D_DEBUG)
    message(STATUS "Building in Debug mode")
else()
    set(CMAKE_BUILD_TYPE Release)
    # Enable optimizations in Release mode
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
    message(STATUS "Building in Release mode")
endif()

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
    yaml-cpp
)