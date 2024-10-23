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
    //double x = -0.1;
    //unsigned int result = (unsigned int)std::ceil(x);
    //std::cout << "ceil(" << x << ") = " << result << std::endl;
    //return 0;


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
    //int parallel_flag = -1;

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

    time_t start,end;

    OctreeVertices octree;
    //Mesher mesher;

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
    OctreePoints octree_ball_centers = octree.copy_skeleton<Point>();

    // Copy the octree iterator
    OctreeIteratorPoints octree_ball_centers_iterator(&octree_ball_centers);
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

    std::cout << "Saving the debug mesh to " << outfile << std::endl;
    std::string debug_outfile = outfile;
    debug_outfile.erase(debug_outfile.find(".ply"), 4);
    debug_outfile += "_cumulative1.txt";
    if(! FileIO::saveMeshDebug(debug_outfile.c_str(), mesher))
    {
        std::cerr<<"Pb saving the mesh; exiting."<<std::endl;
        return EXIT_FAILURE;
    }

    std::set<Facet*>& boundary_facets = mesher.getBoundaryFacets();

    // Save the boundary facets to a file
    std::cout << "Saving trimmed facets to a file" << std::endl;
    std::string debug_boundary_outfile = outfile;
    debug_boundary_outfile.erase(debug_boundary_outfile.find(".ply"), 4);
    debug_boundary_outfile += "_trimmed.txt";
    FileIO::saveMeshDebug(debug_boundary_outfile.c_str(), std::list<Facet*>(boundary_facets.begin(), boundary_facets.end()));
        
    std::cout << "Address of boundary_facets: " << &boundary_facets << std::endl;
    mesher.trimBoundaryFacets(boundary_facets);

    std::cout << "Saving trimmed facets to a file" << std::endl;
    debug_boundary_outfile = outfile;
    debug_boundary_outfile.erase(debug_boundary_outfile.find(".ply"), 4);
    debug_boundary_outfile += "_after_trimmed.txt";
    FileIO::saveMeshDebug(debug_boundary_outfile.c_str(), mesher);

    std::cout<<"Reconstructed mesh after trimming boundary facets: "<<mesher.nVertices()
             <<" vertices; "<<mesher.nFacets()<<" facets. "<<std::endl;
    std::cout<<mesher.nBorderEdges()<<" border edges"<<std::endl;
    std::cout<<"Reconstructing the mesh took "<<difftime(end,start)
             <<"s."<<std::endl;

    // Sanity check 
    #ifdef _DEBUG
    std::cout << "Sanity check: check facets" << std::endl;
    std::list<Facet*> facets = mesher.getFacets();
    for (auto& facet : facets) {
        // Get all three edges of the facet and check their facets
        for (int i = 0; i < 3; ++i) {
            Vertex* vertex1 = facet->getVertex(i);
            Vertex* vertex2 = facet->getVertex((i+1)%3);
            Edge* edge = vertex1->getLinkingEdge(vertex2);
            if (edge->getFacet2() != nullptr && edge->getFacet1() == nullptr) {
                std::cerr << "Facet " << i+1 << " does not exist" << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
    }
    #endif    
    
    // Sanity check: save border edges
    #ifdef _DEBUG
    std::cout << "Sanity check: save border edges after trimming" << std::endl;
    Edge_star_list border_edges = mesher.getBorderEdges();
    std::string debug_border_edges_outfile = outfile;
    debug_border_edges_outfile.erase(debug_border_edges_outfile.find(".ply"), 4);
    debug_border_edges_outfile += "_border_edges.txt";
    FileIO::saveLinesetDebug(debug_border_edges_outfile.c_str(), border_edges);
    // Result: It is correct.
    #endif

    std::cout << "Setting all facets to old" << std::endl;
    mesher.setAllFacetsToOld();
    
    std::cout << "Set empty ball " << std::endl;
    mesher.putBallCentersInOctree();

    // Sanity check: save ball centers
    #ifdef _DEBUG
    std::cout << "Sanity check: save ball centers" << std::endl;
    std::string debug_ball_centers_outfile = outfile;
    debug_ball_centers_outfile.erase(debug_ball_centers_outfile.find(".ply"), 4);
    debug_ball_centers_outfile += "_ball_centers.txt";
    FileIO::saveBallCenters(debug_ball_centers_outfile.c_str(), mesher.getBallCenters(), radius);
    #endif

    // Next batch
    std::cout << "Remove overlape vertices in next batch" << std::endl;
    std::list<Vertex> vertices = FileIO::readFromBatchToList(batch_data[1]);
    

    std::cout << "Vertices size: " << vertices.size() << std::endl;
    for (auto& vertex : vertices) {
       octree_ball_centers.checkSizeAndexpand(vertex);
    }

    std::unordered_set<Vertex*> vertices_set;
    octree_ball_centers_iterator.setDepth(octree.getDepth());
    for (auto& vertex : vertices) {
        //Check if the vertex is in side the any ball
        //Point point = Point(vertex.x(), vertex.y(), vertex.z());
        std::map<double, Point*> neighbors;
        unsigned int num_neighbors = octree_ball_centers_iterator.getSortedNeighbors(vertex, neighbors);
        if (num_neighbors == 0) {
            //Vertex* vertex_ptr = new Vertex(vertex);
            Vertex* vertex_prt = octree.checkSizeAndaddPoint(vertex);
            vertices_set.insert(vertex_prt);
        }
        else {
            //std::cout << "Vertex " << vertex.x() << " " << vertex.y() << " " << vertex.z() << " is inside a ball" << std::endl;
            // How many neighbors are there?
            //std::cout << "Number of neighbors: " << num_neighbors << std::endl;
        }        
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
    if(! FileIO::saveMesh(outfile.c_str(), mesher))
    {
        std::cerr<<"Pb saving the mesh; exiting."<<std::endl;
        return EXIT_FAILURE;
    }
    std::cout<<"Mesh saved in "<<outfile<<std::endl;

    std::cout << "Saving the debug mesh to " << outfile << std::endl;
    debug_outfile = outfile;
    debug_outfile.erase(debug_outfile.find(".ply"), 4);
    debug_outfile += "_cumulative2.txt";
    if(! FileIO::saveMeshDebug(debug_outfile.c_str(), mesher))
    {
        std::cerr<<"Pb saving the mesh; exiting."<<std::endl;
        return EXIT_FAILURE;
    }
    std::cout<<"Mesh saved in "<<debug_outfile<<std::endl;

    return EXIT_SUCCESS;
}
