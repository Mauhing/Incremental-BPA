#cmake_minimum_required(VERSION 2.8)
cmake_minimum_required(VERSION 3.10)
project(ballpivoting)

# Set C++11 standard
#set(CMAKE_CXX_STANDARD 11)

# Set the path to Open3D
set(Open3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/open3d-devel-linux-x86_64-pre-cxx11-abi-0.18.0/lib/cmake/Open3D")

# Find Open3D package
find_package(Open3D REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option to choose between Debug and Release
option(BUILD_DEBUG "Build in Debug mode" ON)

if(BUILD_DEBUG)
    set(CMAKE_BUILD_TYPE Debug)
    # Add debug symbols
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
    # Disable optimizations in Debug mode
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
    message(STATUS "Building in Debug mode")
else()
    set(CMAKE_BUILD_TYPE Release)
    # Enable optimizations in Release mode
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
    message(STATUS "Building in Release mode")
endif()

# Enable all warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Option to use Clang
option(USE_CLANG "build application with clang" OFF)

if(USE_CLANG)
    set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
    add_definitions(-DUSE_CLANG)
else()
    set(CMAKE_CXX_COMPILER "/usr/bin/g++")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ansi -fopenmp")
endif()

message(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Compiler flags: ${CMAKE_CXX_FLAGS}")

# Add your source files
add_executable(ballpivoting main.cpp
			src/Facet.cpp
			src/Edge.cpp
			src/Vertex.cpp
			src/Point.cpp
			src/Mesher.cpp
			src/FileIO.cpp)

# Include Open3D headers
target_include_directories(ballpivoting PRIVATE ${Open3D_INCLUDE_DIRS})

# Link against Open3D libraries
target_link_libraries(ballpivoting PRIVATE ${Open3D_LIBRARIES})

# Set rpath for the executable
set_target_properties(ballpivoting PROPERTIES
    INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/open3d-devel-linux-x86_64-pre-cxx11-abi-0.18.0/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)
