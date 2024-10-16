cmake_minimum_required(VERSION 3.10)
project(ballpivoting)

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
    # Add _DEBUG definition for Debug mode
    add_definitions(-D_DEBUG)
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

# Set the path to Open3D
set(Open3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/open3d-devel-linux-x86_64-pre-cxx11-abi-0.18.0/lib/cmake/Open3D")

# Find Open3D package
find_package(Open3D REQUIRED)

# Print Open3D include directories for debugging
message(STATUS "Open3D include dirs: ${Open3D_INCLUDE_DIRS}")

# Add your source files
add_executable(ballpivoting 
    main.cpp
    src/Edge.cpp
    src/Facet.cpp
    src/FileIO.cpp
    src/Mesher.cpp
    src/Point.cpp
    src/Vertex.cpp
)

# Add compile definitions for debug mode
target_compile_definitions(ballpivoting PRIVATE $<$<CONFIG:Debug>:_DEBUG>)

# Add Open3D include directories
target_include_directories(ballpivoting PRIVATE ${Open3D_INCLUDE_DIRS})

# Link against Open3D libraries
target_link_libraries(ballpivoting PRIVATE ${Open3D_LIBRARIES})

# Add compile definitions required by Open3D
#target_compile_definitions(ballpivoting PRIVATE ${Open3D_DEFINITIONS})

# Set rpath for the executable
set_target_properties(ballpivoting PROPERTIES
    INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/open3d-devel-linux-x86_64-pre-cxx11-abi-0.18.0/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()
add_subdirectory(src)
add_subdirectory(tests)