cmake_minimum_required(VERSION 3.10)
project(core_ibpa)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add option for debug build first
option(BUILD_DEBUG "Build in Debug mode" ON)

# Set initial build type based on BUILD_DEBUG
if(BUILD_DEBUG)
    set(CMAKE_BUILD_TYPE Debug)
else()
    set(CMAKE_BUILD_TYPE Release)
endif()

# First compile yaml-cpp
set(YAML_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp-0.8.0")
set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "yaml-cpp build shared library" FORCE)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "yaml-cpp build tests" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "yaml-cpp build tools" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "yaml-cpp build contrib" FORCE)

# Build yaml-cpp with ABI=0
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
add_subdirectory(${YAML_CPP_DIR} EXCLUDE_FROM_ALL)

# Update the message to show the correct library name based on build type
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "YAML library location: ${CMAKE_BINARY_DIR}/third_party/yaml-cpp-0.8.0/lib$<$<CONFIG:Debug>:yaml-cppd>$<$<NOT:$<CONFIG:Debug>>:yaml-cpp>.a")

# Now continue with main project in Debug mode
option(BUILD_DEBUG "Build in Debug mode" ON)

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# Enable all warnings and checks for unused code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wconversion -Werror=conversion -Wunused -Wunused-function -Wunused-variable")

# Option to use Clang
option(USE_CLANG "build application with clang" OFF)

if(USE_CLANG)
    set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
    add_definitions(-DUSE_CLANG)
else()
    set(CMAKE_CXX_COMPILER "/usr/bin/g++")
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
file(GLOB_RECURSE SRC_FILES "src/*.cpp")

add_executable(core_ibpa 
    main.cpp
    ${SRC_FILES}
)

# Add compile definitions for debug mode
target_compile_definitions(core_ibpa PRIVATE $<$<CONFIG:Debug>:_DEBUG>)


# Update include directories to include yaml-cpp headers
target_include_directories(core_ibpa PRIVATE 
    ${Open3D_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${YAML_CPP_DIR}/include
)

target_link_libraries(core_ibpa PRIVATE 
    yaml-cpp
)

# Find pthread
find_package(Threads REQUIRED)

# Link against Open3D libraries and pthread
target_link_libraries(core_ibpa PRIVATE 
    ${Open3D_LIBRARIES}
    Threads::Threads
)

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()
