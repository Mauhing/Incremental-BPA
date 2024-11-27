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

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Octree creation
    time_t start, end;
    std::time(&start);

    OctreeVertices octree_vertices;
    if (radius > 0)
    {
        // ok = FileIO::readAndSortPoints(infile.c_str(),octree,radius);
        ok = FileIO::readFromBatchAndSortPoints(batch_data[0], octree_vertices, radius);
    }
    if (!ok)
    {
        std::cerr << "Pb opening the file; exiting." << std::endl;
        return EXIT_FAILURE;
    }
    std::time(&end);

    octree_vertices.printOctreeStat();
    std::cout << "Reading and sorting points in this octree took "
              << difftime(end, start) << " s." << std::endl;
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Set the vertices iterator
    std::cout << "****** Reconstructing with radii " << std::flush;
    std::list<double>::const_iterator ri = options.radii.begin();
    while (ri != options.radii.end())
    {
        std::cout << *ri << "; ";
        ++ri;
    }
    std::cout << "******" << std::endl;

    OctreeIteratorVertices iterator_vertices(&octree_vertices);

    if (radius > 0)
        iterator_vertices.setR(radius);
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Create the ball centers octree and its iterator
    // OctreeBallCenters octree_ball_centers = octree_vertices.copy_skeleton<BallCenter>();
    OctreeBallCenters octree_ball_centers;
    octree_ball_centers.setDepth(octree_vertices.getDepth());
    Point origin = octree_vertices.getOrigin();
    octree_ball_centers.initialize(origin, octree_vertices.getSize());

    OctreeIteratorBallCenters octree_ball_centers_iterator(&octree_ball_centers);

    if (radius > 0)
        octree_ball_centers_iterator.setR(radius);
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Bootstrapping reconstruction
    std::time(&start);
    Mesher mesher(&octree_vertices, &iterator_vertices,
                  &octree_ball_centers,
                  &octree_ball_centers_iterator);

    std::cout << "Reconstructing the mesh" << std::endl;
    mesher.reconstruct(options.radii);
    std::time(&end);
    std::cout << "Finish reconstruction" << std::endl;
    std::cout << "Reconstructing the mesh took " << difftime(end, start)
              << "s." << std::endl;
    mesher.print_stats();
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Fill holes
    std::cout << "Filling holes..." << std::endl;
    std::time(&start);
    mesher.fillHoles();
    std::time(&end);
    std::cout << "Filling holes took " << difftime(end, start) << "s." << std::endl;
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Create shared data structures and synchronization primitives
    std::cout << "Main Thread ID: " << std::this_thread::get_id() << std::endl;

    std::mutex o3d_mesh_mutex;
    std::atomic<bool> should_exit(false);
    std::condition_variable vis_cv;
    mesher.setVisualizationSync(&o3d_mesh_mutex, &vis_cv);

    // Create visualization thread
    std::thread vis_thread(Visualizer::visualizationThread, 
        std::ref(mesher), 
        std::ref(o3d_mesh_mutex), 
        std::ref(vis_cv),
        std::ref(should_exit)
    );

    std::cout << "Visualization Thread ID: " << vis_thread.get_id() << std::endl;
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Sanity check
    std::cout << "Sanity checking all nodes points" << std::endl;
    std::cout << "Ball centers" << std::endl;
    std::cout << "Number of ball centers: " << octree_ball_centers_iterator.debug_checkTotalNumberOfElements() << std::endl;
    octree_ball_centers_iterator.debug_checkAllNodesPoints();
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

            // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Remove singular vertices
    mesher.removeFanFanSingular();
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        
    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Remove disk singular vertices
    mesher.removeDiskFanSingular();
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        
    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Clear fresh facets
    mesher.clearFreshFacets();
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Clear orphan vertices
    mesher.clearOrphanVertices();
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // #ifdef _DEBUG
    // std::cout << "Sanity check: check orientation" << std::endl;
    // mesher.SanityCheckOrientation();
    // #endif

    //if (!FileIO::saveMeshDebug("_cumulative0.txt", mesher))
    //{
    //    std::cerr << "Pb saving the mesh; exiting." << std::endl;
    //    return EXIT_FAILURE;
    //}



    // Sanity check: save border edges
    // #ifdef _DEBUG
    // std::cout << "Sanity check: save border edges after trimming" << std::endl;
    // Edge_star_list border_edges = mesher.getBorderEdges();
    // FileIO::saveLinesetDebug("_border_edges.txt", border_edges);
    // #endif

    for (size_t batch_index = 1; batch_index < batch_data.size(); batch_index++)
    {
        if (should_exit) {  // Check if visualization window was closed
            break;
        }

        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Processing batch " << batch_index << std::endl;

        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // Get the next batch
        std::cout << "Remove overlape vertices in next batch" << std::endl;
        std::list<Vertex> vertices = FileIO::readFromBatchToList(batch_data[batch_index]);
        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // Set slow visualization flag when at batch 8
        if (batch_index == 8) {
            mesher.setSlowVisualization(true);  // Add this method to Mesher
        } else {
            mesher.setSlowVisualization(false);
        }
        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // Expand the ballcenters octree
        // std::cout << "Vertices size: " << vertices.size() << std::endl;
        std::cout << "Expanding octree ball centers" << std::endl;
        for (auto &vertex : vertices)
        {
            BallCenter ball_center(vertex, nullptr);
            octree_ball_centers.checkSizeAndexpand(ball_center);
        }
        std::cout << "Expanding octree vertices" << std::endl;
        for (auto &vertex : vertices)
        {
            octree_vertices.checkSizeAndexpand(vertex);
        }
        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // Reconstruct the mesh
        mesher.batchReconstruct(vertices);
        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        mesher.print_stats();
        
        //std::cout << "Press enter to continue" << std::endl;
        //std::cin.get();

        //FileIO::saveMeshDebug(("_cumulative" + std::to_string(batch_index) + ".txt").c_str(), mesher);
        
    }

    // wait for terminal input
    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();

    should_exit = true;
    vis_thread.join();

    if (!FileIO::saveMeshDebug("_final.txt", mesher))
    {
        std::cerr << "Pb saving the mesh; exiting." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Mesh saved in" << "_final.txt" << std::endl;

    
    // Open3D save ply
    bool o3d_save_ply = false;

    if (o3d_save_ply) {
        auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        mesher.renderIntoOpen3D(*o3d_mesh);
        open3d::io::WriteTriangleMeshToPLY("new_final.ply", *o3d_mesh, false, false, false, false, false, true);
        std::cout << "Mesh saved in" << "new_final.ply" << std::endl;
    }
    else {
        std::cout << "O3d ply file not saved" << std::endl;
    }   

    return EXIT_SUCCESS;
}
