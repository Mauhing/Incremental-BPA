/**
 * @file main.cpp
 * @brief main program file for the ball pivoting method
 * @author Julie Digne julie.digne@liris.cnrs.fr
 * @date 2012/11/14
 *
 * @copyright This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <sstream>
#include <ctime>
#include <getopt.h>
#include <vector>

#include "include/Octree.h"
#include "include/OctreeIterator.h"
#include "include/utilities.h"
#include "include/Vertex.h"
#include "include/Mesher.h"
#include "include/FileIO.h"
#include "include/types.h"
#include "include/ProgramOptions.hpp"
#include "include/Visualizer.h"
#include "include/Reconstructor.h"

#include <open3d/Open3D.h>
#include <thread>
#include <mutex>
#include <atomic>

std::condition_variable cv_debug_visualization;
bool rendering_in_progress(false);
std::mutex o3d_mesh_mutex;
std::atomic<bool> should_exit(false);
Eigen::Matrix<double, 3, 4> robot_pose;

/**
 * @brief main function for the ball pivoting reconstruction
 * @param argc
 * @param argv
 * @return 1 if the program exited successfully
 */

int main(int argc, char **argv)
{
    // Parse the command line
    ProgramOptions options = parseCommandLine(argc, argv);

    //std::exit(EXIT_SUCCESS);

    // Use the parsed options
    std::cout << "Input file: " << options.input_file << std::endl;
    std::cout << "Output file: " << options.output_file << std::endl;
    std::cout << "Radius: " << options.radius << std::endl;
    std::cout << "Max orphan points per voxel: " << options.max_orphan_per_voxel << std::endl;
    std::cout << "Reading per batch: " << options.reading_per_batch << std::endl;
    std::cout << "Policy main mesh: " << (options.policy_main_mesh ? "true" : "false") << std::endl;
    std::cout << "Policy main mesh activation batch number: " << options.policy_main_mesh_activation_batch_number << std::endl;
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Set the base output filename
    FileIO::setBaseOutputFilename(options.output_file.erase(options.output_file.find(".ply"), 4));
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    // Get the input filename from the program options 
    std::string infile = options.input_file;

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Read the input file
    // Turn the whole data into batch data
    std::vector<SensorFrame> sensor_frames = FileIO::readIntoFileBatch(infile.c_str());

    if (sensor_frames.empty())
    {
        std::cerr << "Error: No data read from file" << std::endl;
        return EXIT_FAILURE;
    }
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Initialize the reconstructor
    Reconstructor reconstructor(options);
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    int first_one_third_batch = static_cast<int>(sensor_frames.size() / 3);
    bool first_one_third_batch_done = false;
    int second_one_third_batch = static_cast<int>(sensor_frames.size() * 2 / 3);
    bool second_one_third_batch_done = false;

    //print the first_one_third_batch and second_one_third_batch
    std::cout << "First one third batch: " << first_one_third_batch << std::endl;
    std::cout << "Second one third batch: " << second_one_third_batch << std::endl;
    
    //sleep for 10 seconds
    std::this_thread::sleep_for(std::chrono::seconds(10));

    auto start_time = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < sensor_frames.size(); i += options.reading_per_batch) {
        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // Load batch of point cloud
        std::pair<std::list<Vertex>, Eigen::Matrix<double, 3, 4>> vertices_and_pose = FileIO::readFromBatchToList(sensor_frames, i, options.reading_per_batch);
        std::list<Vertex> vertices = vertices_and_pose.first;
        robot_pose = vertices_and_pose.second;
        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        
        //Todo: this part should be optimized. We should use vector instead of list.
        reconstructor.reconstruct(robot_pose, vertices);

        if (reconstructor.shouldExit()) {
            std::cout << "\033[33mExiting\033[0m" << std::endl;
            break;
        }
        
        //if (i >= first_one_third_batch && !first_one_third_batch_done) {
        //    std::cout << "\033[33mFirst one third batch done\033[0m" << std::endl;
        //    std::this_thread::sleep_for(std::chrono::seconds(10));
        //    first_one_third_batch_done = true;
        //}
        //if (i >= second_one_third_batch && !second_one_third_batch_done) {
        //    std::cout << "\033[33mSecond one third batch done\033[0m" << std::endl;
        //    std::this_thread::sleep_for(std::chrono::seconds(10));
        //    second_one_third_batch_done = true;
        //}
        
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    std::cout << "Time taken: " << duration.count() << " seconds" << std::endl;

    std::cout << "\033[33mPress Enter to exit\033[0m" << std::endl;
    std::cin.get();

    return EXIT_SUCCESS;
}
