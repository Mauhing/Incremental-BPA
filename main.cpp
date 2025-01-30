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
    bool ok;
    std::string infile = options.input_infiles;

    // Turn the whole data into batch data
    // std::vector<string> batch_data = FileIO::readIntoFileBatch(infile.c_str());
    std::vector<string> batch_data;
    try
    {
        batch_data = FileIO::readIntoFileBatch(infile.c_str());
        if (batch_data.empty())
        {
            std::cerr << "Error: No data read from file" << std::endl;
            return EXIT_FAILURE;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error reading file: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "batch_data size: " << batch_data.size() << std::endl;
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // Octree creation
    time_t start, end;
    std::time(&start);

    Point origin;
    double size;
    unsigned int depth;
    std::tie(origin, size, depth) = FileIO::originAndDepth(batch_data[0], radius);


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
    mesher.setVisualizationSync(&o3d_mesh_mutex);

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
    size_t max_batch_index  = batch_data.size();
    for (size_t batch_index = 0; batch_index < max_batch_index; batch_index++)
    {
        if (should_exit)
        { // Check if visualization window was closed
            break;
        }

        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Processing batch " << batch_index << std::endl;

        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // Load batch of point cloud
        std::cout << "Remove overlape vertices in next batch" << std::endl;
        std::list<Vertex> vertices = FileIO::readFromBatchToList(batch_data[batch_index]);
        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        {
            std::lock_guard<std::mutex> lock(o3d_mesh_mutex);
            // Add the vertices to the received_vertices
            received_vertices.clear();
            for (const auto& v : vertices) {
                received_vertices.emplace_back(v, Eigen::Vector3d(0.0, 1.0, 0.0)); // Green
            }
        }

        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // Set slow visualization flag when at batch 8
        // if (batch_index == 8) {
        //    mesher.setSlowVisualization(true);  // Add this method to Mesher
        //} else {
        //    mesher.setSlowVisualization(false);
        //}
        mesher.setSlowVisualization(false);
        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        mesher.batchReconstruct(vertices);

        //mesher.mesh_integrityCheck();
        
#ifdef _DEBUG
        {
            std::unique_lock<std::mutex> lock(o3d_mesh_mutex);
            task_in_progress = true;
            cv_debug_visualization.notify_one();
            std::cout << "\033[33mTask in progress set to true\033[0m" << std::endl;
            std::cout << "\033[33mSignal sent from main\033[0m" << std::endl;
            
            cv_debug_visualization.wait(lock, [&task_in_progress]{ return !task_in_progress; });
            std::cout << "\033[33mSignal received at main\033[0m" << std::endl;
        }
#endif
    }

    // wait for terminal input
    //std::cout << "Press Enter to exit..." << std::endl;
    //std::cin.get();

    should_exit = true;
    vis_thread.join();

    if (!FileIO::saveMeshDebug("_final.txt", mesher))
    {
        std::cerr << "Pb saving the mesh; exiting." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Mesh saved in" << "_final.txt" << std::endl;

    // Open3D save ply
    bool o3d_save_ply = true;
    if (o3d_save_ply)
    {
        std::shared_ptr<open3d::geometry::TriangleMesh> o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        Visualizer::renderMainMesh(mesher.getFacets(), o3d_mesh);
        open3d::io::WriteTriangleMeshToPLY("new_final.ply", *o3d_mesh, false, false, false, false, false, true);
        std::cout << "Mesh saved in" << "new_final.ply" << std::endl;
    }
    else
    {
        std::cout << "O3d ply file not saved" << std::endl;
    }
    return EXIT_SUCCESS;
}
