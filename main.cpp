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

//#include <open3d/Open3D.h>

/**
 * @brief main function for the ball pivoting reconstruction
 * @param argc
 * @param argv
 * @return 1 if the program exited successfully
 */ 

int main(int argc, char **argv)
{
    //handling command line options
    int c;
    stringstream f;
    string input_infiles, outfile, input_radii;
    std::list<string> infile_list;
    double radius = -1;
    int radius_flag = -1;
    int infile_flag = -1;
    int outfile_flag = -1;
    std::list<double> radii;

    // parse command line options
    while( (c = getopt(argc,argv, "i:o:d:r:p")) != -1)
    {
        switch(c)
        {
            case 'i':
            {
                string infile;
                input_infiles=optarg;
                infile_flag = 1;
                break;

                //f.clear();
                //f<<optarg;
                //f>>infile;
                //infile_flag = 1;
                //break;
            }
            case 'o':
            {
                f.clear();
                f<<optarg;
                f>>outfile;
                outfile_flag = 1;
                break;
            }
            case 'p':
            {
                //parallel_flag = 1;
                break;
            }
            case 'r': 
            {
                input_radii=optarg;
                istringstream iss(input_radii, istringstream::in);
                while (iss>>radius)
                {
                    radii.push_back(radius);
                }
                radius_flag = 1;
                break;
            }
        }    
    }

    if(infile_flag == -1)
    {
        std::cerr<<"No input file given (use the -i option)"<<std::endl;
        return EXIT_FAILURE;
    }

    if(outfile_flag == -1)
    {
        std::cerr<<"No output file given (use the -o option)"<<std::endl;
        return EXIT_FAILURE;
    }

    if(radius_flag == 1)
    {
        radii.sort();
        radius = radii.front();
    }

    // Set the base output filename
    FileIO::setBaseOutputFilename(outfile.erase(outfile.find(".ply"), 4));

    time_t start,end;
    OctreeVertices octree;

    std::time(&start);
    bool ok;
    std::string infile = input_infiles;

    // Turn the whole data into batch data
    std::vector<string> batch_data = FileIO::readIntoFileBatch(input_infiles.c_str());
    std::cout << "batch_data size: " << batch_data.size() << std::endl;


    if(radius >0)
    {
        //ok = FileIO::readAndSortPoints(infile.c_str(),octree,radius); 
        ok = FileIO::readFromBatchAndSortPoints(batch_data[0], octree, radius);
    } 
    if( !ok )
    {
        std::cerr<<"Pb opening the file; exiting."<<std::endl;
        return EXIT_FAILURE;
    }
    std::time(&end);

    std::cout<<"OctreeVertices with depth "<<octree.getDepth()<<" created."<<std::endl;
    std::cout<<"OctreeVertices contains "<<octree.getNpoints()
            <<" points. The bounding box size is "
            <<octree.getSize()<<std::endl;
    std::cout<<"Reading and sorting points in this octree took "
            <<difftime(end,start)<<" s."<<std::endl;
    std::cout<<"OctreeVertices statistics"<<std::endl;
    octree.printOctreeStat();

    std::cout << "OctreeVertices root depth: " << octree.getRoot()->getDepth()<< std::endl;
    std::cout << "OctreeVertices root size: " << octree.getRoot()->getSize()<< std::endl;
    //octree.debugPrint();

    std::cout<<"****** Reconstructing with radii "<<std::flush;
    std::list<double>::const_iterator ri = radii.begin();
    while(ri != radii.end())
    {
        std::cout<< *ri <<"; ";
        ++ri;
    }
    std::cout<<"******"<<std::endl;

    OctreeIteratorVertices iterator(&octree);

    if(radius>0)
        iterator.setR(radius);

    // Copy the octree skeleton to a new octree
    OctreeBallCenters octree_ball_centers = octree.copy_skeleton<BallCenter>();

    // Copy the octree iterator
    OctreeIteratorBallCenters octree_ball_centers_iterator(&octree_ball_centers);

    if (radius > 0)
        octree_ball_centers_iterator.setR(radius);

    std::time(&start);
    Mesher mesher(&octree, &iterator, &octree_ball_centers, &octree_ball_centers_iterator);
    mesher.reconstruct(radii);
    std::time(&end);

    std::cout<<"Reconstructed mesh: "<<mesher.nVertices()
             <<" vertices; "<<mesher.nFacets()<<" facets. "<<std::endl;
    std::cout<<mesher.nBorderEdges()<<" border edges"<<std::endl;
    std::cout<<"Reconstructing the mesh took "<<difftime(end,start)
             <<"s."<<std::endl;

    std::cout << "Filling holes..." << std::endl;
    std::time(&start);
    mesher.fillHoles();
    std::time(&end);
    std::cout << "Filling holes took " << difftime(end,start) << "s." << std::endl;

    std::cout << "Sanity check: check orientation" << std::endl;
    mesher.SanityCheckOrientation();

    //std::cout << "Saving the debug mesh to " << outfile << std::endl;
    //std::string debug_outfile = outfile;
    //debug_outfile.erase(debug_outfile.find(".ply"), 4);
    //debug_outfile += "_cumulative1.txt";
    if(! FileIO::saveMeshDebug("_cumulative1.txt", mesher))
    {
        std::cerr<<"Pb saving the mesh; exiting."<<std::endl;
        return EXIT_FAILURE;
    }

    //std::set<Facet*>& boundary_facets = mesher.getBoundaryFacets();

    // Save the boundary facets to a file
    //std::cout << "Saving trimmed facets to a file" << std::endl;
    //FileIO::saveMeshDebug("_trimmed.txt", std::list<Facet*>(boundary_facets.begin(), boundary_facets.end()));
        
    //std::cout << "Address of boundary_facets: " << &boundary_facets << std::endl;
    //mesher.removeFacets(boundary_facets);

    //std::cout << "Saving trimmed facets to a file" << std::endl;
    //FileIO::saveMeshDebug("_after_trimmed.txt", mesher);

    std::cout<<"Reconstructed mesh after trimming boundary facets: "<<mesher.nVertices()
             <<" vertices; "<<mesher.nFacets()<<" facets. "<<std::endl;
    std::cout<<mesher.nBorderEdges()<<" border edges"<<std::endl;
    std::cout<<"Reconstructing the mesh took "<<difftime(end,start)
             <<"s."<<std::endl;

    // Sanity check: save border edges
    #ifdef _DEBUG
    std::cout << "Sanity check: save border edges after trimming" << std::endl;
    Edge_star_list border_edges = mesher.getBorderEdges();
    FileIO::saveLinesetDebug("_border_edges.txt", border_edges);
    // Result: It is correct.
    #endif

    std::cout << "Setting all facets to old" << std::endl;
    mesher.setAllFacetsToOld();
    
    std::cout << "Set empty ball " << std::endl;
    mesher.putBallCentersInOctree();

    // Sanity check: save ball centers
    #ifdef _DEBUG
    std::cout << "Sanity check: save ball centers" << std::endl;
    FileIO::saveBallCenters("_ball_centers.txt", mesher.getBallCenters(), radius);
    #endif

    // Next batch
    std::cout << "Remove overlape vertices in next batch" << std::endl;
    std::list<Vertex> vertices = FileIO::readFromBatchToList(batch_data[1]);
    
    std::cout << "Vertices size: " << vertices.size() << std::endl;
    for (auto& vertex : vertices) {
        BallCenter ball_center(vertex, nullptr);
        octree_ball_centers.checkSizeAndexpand(ball_center);
    }

    Facet_set collision_facets;
    std::unordered_set<Vertex*> vertices_inside_ball_set;
    octree_ball_centers_iterator.setDepth(octree.getDepth());
    double min_squared_distance = radius * radius;
    for (auto& vertex : vertices) {
        //Check if the vertex is in side the any ball
        //Point point = Point(vertex.x(), vertex.y(), vertex.z());
        std::map<double, BallCenter*> neighbors; // neighbor.first is the squared distance

        //unsigned int num_neighbors = octree_ball_centers_iterator.getSortedNeighbors(vertex, neighbors);
        octree_ball_centers_iterator.getSortedNeighbors(vertex, neighbors);

        // if any squared distance is less than the squared radius, the vertex is inside a ball
        for (auto& neighbor : neighbors) {
            if (neighbor.first < min_squared_distance && neighbor.second != nullptr) {
                BallCenter* ball_center = neighbor.second;
                Facet* facet = ball_center->getFacet();
                if (facet != nullptr) {
                    collision_facets.insert(facet);
                }
                else {
                    std::cerr << "Error: The facet is nullptr" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                delete ball_center; // TODO: need to update the octree node
                neighbor.second = nullptr;
            }
        }
    }

    // Remove the collision facets
    mesher.removeFacets(collision_facets);
    
    // Add the new vertices to the octree
    for (auto& vertex : vertices) {
        octree.checkSizeAndaddPoint(vertex);
    }


    std::cout << "Further reconstructing" << std::endl;
    mesher.furtherReconstruct();
    
    mesher.fillHoles();
    
    std::cout<<"Reconstructed mesh: "<<mesher.nVertices()
             <<" vertices; "<<mesher.nFacets()<<" facets. "<<std::endl;
    std::cout<<mesher.nBorderEdges()<<" border edges"<<std::endl;
    std::cout<<"Reconstructing the mesh took "<<difftime(end,start)
             <<"s."<<std::endl;
    
    std::cout << "Saving mesh to " << outfile << std::endl;
    if(! FileIO::saveMesh("_final.ply", mesher))
    {
        std::cerr<<"Pb saving the mesh; exiting."<<std::endl;
        return EXIT_FAILURE;
    }
    std::cout<<"Mesh saved in "<<"_final.ply"<<std::endl;

    std::cout << "Saving the debug mesh to " << outfile << std::endl;
    if(! FileIO::saveMeshDebug("_cumulative2.txt", mesher))
    {
        std::cerr<<"Pb saving the mesh; exiting."<<std::endl;
        return EXIT_FAILURE;
    }
    std::cout<<"Mesh saved in "<<"_cumulative2.txt"<<std::endl;

    return EXIT_SUCCESS;
}
