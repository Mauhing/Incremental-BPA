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

#include "src/Octree.h"
#include "src/OctreeIterator.h"
#include "src/utilities.h"
#include "src/Vertex.h"
#include "src/Mesher.h"
#include "src/FileIO.h"
#include "src/types.h"
#include "src/ProgramOptions.hpp"
#include "src/Visualizer.h"

#include <open3d/Open3D.h>
#include <thread>
#include <mutex>
#include <atomic>

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

    // Use the parsed options
    std::cout << "Input files: " << options.input_infiles << std::endl;
    std::cout << "Output file: " << options.outfile << std::endl;
    std::cout << "Radii: ";
    for (const auto &radius : options.radii)
    {
        std::cout << radius << " ";
    }
    std::cout << std::endl;
    std::cout << "Parallel flag: " << (options.parallel_flag ? "true" : "false") << std::endl;

    double radius = -1;
    if (options.radii.size() > 0)
    {
        options.radii.sort();
        radius = options.radii.front();
    }

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Set the base output filename
    FileIO::setBaseOutputFilename(options.outfile.erase(options.outfile.find(".ply"), 4));
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Read the input file
    std::string infile = options.input_infiles;

    // Turn the whole data into batch data
    // std::vector<string> batch_data = FileIO::readIntoFileBatch_PointOnly(infile.c_str());

    //batch_data = FileIO::readIntoFileBatch_PointOnly(infile.c_str());
    std::vector<SensorFrame> sensor_frames = FileIO::readIntoFileBatch(infile.c_str());
    if (sensor_frames.empty())
    {
        std::cerr << "Error: No data read from file" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "sensor_frames size: " << sensor_frames.size() << std::endl;
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // Octree creation

    Point origin;
    double size;
    unsigned int depth;
    //std::tie(origin, size, depth) = FileIO::originAndDepth_depricated(batch_data[0], radius); 
    std::tie(origin, size, depth) = FileIO::originAndDepth(sensor_frames, radius); 

    const int max_orphan_points_per_node = 1;

    OctreeVertices octree_vertices(max_orphan_points_per_node);
    octree_vertices.setDepth(depth);
    octree_vertices.initialize(origin, size);

    OctreeIteratorVertices iterator_vertices(&octree_vertices);
    iterator_vertices.setR(radius);
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // Create the ball centers octree and its iterator
    OctreeBallCenters octree_ball_centers(-1);
    octree_ball_centers.setDepth(octree_vertices.getDepth());
    octree_ball_centers.initialize(octree_vertices.getOrigin(), octree_vertices.getSize());

    OctreeIteratorBallCenters octree_ball_centers_iterator(&octree_ball_centers);
    octree_ball_centers_iterator.setR(radius);
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // Define the mesher
    Mesher mesher(&octree_vertices, &iterator_vertices,
                  &octree_ball_centers,
                  &octree_ball_centers_iterator);

    // Create shared data structures and synchronization primitives
    std::cout << "Main Thread ID: " << std::this_thread::get_id() << std::endl;

    std::mutex o3d_mesh_mutex;
    std::atomic<bool> should_exit(false);

    std::condition_variable cv_debug_visualization;
    bool task_in_progress(false);

    std::vector<ColorVertex> received_vertices;

    std::thread vis_thread(Visualizer::visualizationThread,
                           std::ref(mesher),
                           std::ref(o3d_mesh_mutex),
                           std::ref(should_exit),
                           std::ref(received_vertices),
                           std::ref(cv_debug_visualization),
                           std::ref(task_in_progress));

    std::cout << "Visualization Thread ID: " << vis_thread.get_id() << std::endl;

    // enter looping phase
    // time it
    time_t start_batch, end_batch;
    std::time(&start_batch);
    size_t max_batch_index  = sensor_frames.size();
    size_t batch_size = 10;
    for (size_t batch_index = 0; batch_index < max_batch_index; batch_index += batch_size)
    {
        if (should_exit)
        { // Check if visualization window was closed
            break;
        }

        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Processing batch " << batch_index << std::endl;

        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // Load batch of point cloud
        std::list<Vertex> vertices = FileIO::readFromBatchToList(sensor_frames, batch_index, batch_size);
        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        received_vertices.clear();
        for (const auto& v : vertices) {
            received_vertices.emplace_back(v, Eigen::Vector3d(0.0, 1.0, 0.0)); // Green
        }

        mesher.batchReconstruct(vertices);

        if (batch_index == 50) {
            mesher.setMainRegion();
            mesher.removeNonMainFacets();
        }
        #ifdef _DEBUG
        //mesher.mesh_integrityCheck();
        #endif
        
        {
            std::unique_lock<std::mutex> lock(o3d_mesh_mutex);
            task_in_progress = true;
            cv_debug_visualization.notify_one();
            std::cout << "\033[33mTask in progress set to true\033[0m" << std::endl;
            std::cout << "\033[33mSignal sent from main\033[0m" << std::endl;
            
            cv_debug_visualization.wait(lock, [&task_in_progress]{ return !task_in_progress; });
            std::cout << "\033[33mSignal received at main\033[0m" << std::endl;
        }
        std::cout << "Press Enter to exit..." << std::endl;
        std::cin.get();
        
    }

    std::time(&end_batch);
    double seconds_batch = std::difftime(end_batch, start_batch);
    int minutes_batch = static_cast<int>(seconds_batch) / 60;
    seconds_batch = std::fmod(seconds_batch, 60.0);
    std::cout << "Time taken: " << minutes_batch << " minutes " << seconds_batch << " seconds" << std::endl;

    // wait for terminal input
    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();

    should_exit = true;
    vis_thread.join();
    #ifdef _DEBUG
    if (!FileIO::saveMeshDebug("_final.txt", mesher))
    {
        std::cerr << "Pb saving the mesh; exiting." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Mesh saved in" << "_final.txt" << std::endl;
    #endif

    // Open3D save ply
    // time it
    time_t start_o3d, end_o3d;
    std::time(&start_o3d);
    bool o3d_save_ply = true;
    if (o3d_save_ply)
    {
        std::shared_ptr<open3d::geometry::TriangleMesh> o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        Visualizer::renderMainMesh(mesher.getFacets(), o3d_mesh);
        open3d::io::WriteTriangleMeshToPLY("Final_mesh.ply", *o3d_mesh, false, false, false, false, false, true);
        std::cout << "Mesh saved in" << "new_final.ply" << std::endl;
    }
    else
    {
        std::cout << "O3d ply file not saved" << std::endl;
    }
    std::time(&end_o3d);
    double seconds_o3d = std::difftime(end_o3d, start_o3d);
    int minutes_o3d = static_cast<int>(seconds_o3d) / 60;
    seconds_o3d = std::fmod(seconds_o3d, 60.0);
    std::cout << "Time taken: " << minutes_o3d << " minutes " << seconds_o3d << " seconds" << std::endl;

    return EXIT_SUCCESS;
}
