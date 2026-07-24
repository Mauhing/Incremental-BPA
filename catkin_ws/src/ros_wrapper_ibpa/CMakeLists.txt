cmake_minimum_required(VERSION 3.0.2)
project(ros_wrapper_ibpa)

# ─────────────────────────────────────────────────────────────
# 🔧 Build configuration
# ─────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 🐞 Debug/Release toggle
option(BUILD_DEBUG "Build in Debug mode" ON)
if (BUILD_DEBUG)
  # Add debug symbols
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
  # Disable optimizations in Debug mode
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

  set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
  add_definitions(-D_DEBUG)
  # print CMAKE_CXX_FLAGS_DEBUG
  message(STATUS "CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
else()
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Apply global debug postfix
if (NOT DEFINED CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

# ─────────────────────────────────────────────────────────────
# 🤖 ROS configuration
# ─────────────────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  rosbag
  tf2_ros
  tf2_eigen
  tf2
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs rosbag tf2_ros tf2_eigen tf2
)

# ─────────────────────────────────────────────────────────────
# 🧩 Third-party dependencies
# ─────────────────────────────────────────────────────────────
set(CORE_WS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../core_ws")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Include and build the core_ws as a subdirectory
add_subdirectory(${CORE_WS_PATH} ${CMAKE_BINARY_DIR}/core_ws_build)

# Open3D imported targets are directory-scoped in CMake. Resolve Open3D again
# in this package scope so transitive target properties can be evaluated.
find_package(Open3D QUIET CONFIG
  PATHS
    "${CMAKE_BINARY_DIR}/_deps/open3d-src/lib/cmake/Open3D"
    "${CORE_WS_PATH}/build/_deps/open3d-src/lib/cmake/Open3D"
  NO_DEFAULT_PATH
)
if(NOT Open3D_FOUND)
  find_package(Open3D REQUIRED CONFIG)
endif()

# ─────────────────────────────────────────────────────────────
# 🧱 Targets
# ─────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME}
  src/rosIbpaReconstructor.cpp
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

# ─────────────────────────────────────────────────────────────
# 🔗 Linking
# ─────────────────────────────────────────────────────────────
target_link_libraries(${PROJECT_NAME}
  core_ibpa_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
