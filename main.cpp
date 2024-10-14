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
    int parallel_flag = -1;

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
                parallel_flag = 1;
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

    Octree octree;
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

    std::cout<<"Octree with depth "<<octree.getDepth()<<" created."<<std::endl;
    std::cout<<"Octree contains "<<octree.getNpoints()
            <<" points. The bounding box size is "
            <<octree.getSize()<<std::endl;
    std::cout<<"Reading and sorting points in this octree took "
            <<difftime(end,start)<<" s."<<std::endl;
    std::cout<<"Octree statistics"<<std::endl;
    octree.printOctreeStat();

    std::cout << "Octree root depth: " << octree.getRoot()->getDepth()<< std::endl;
    std::cout << "Octree root size: " << octree.getRoot()->getSize()<< std::endl;
    //octree.debugPrint();

    std::cout<<"****** Reconstructing with radii "<<std::flush;
    std::list<double>::const_iterator ri = radii.begin();
    while(ri != radii.end())
    {
        std::cout<< *ri <<"; ";
        ++ri;
    }
    std::cout<<"******"<<std::endl;

    OctreeIterator iterator(&octree);


    if(radius>0)
        iterator.setR(radius);

    std::time(&start);

    Mesher mesher(&octree, &iterator);
    if(parallel_flag == 1)
        mesher.parallelReconstruct(radii);
    else
        mesher.reconstruct(radii);
    std::time(&end);

    std::cout<<"Reconstructed mesh: "<<mesher.nVertices()
             <<" vertices; "<<mesher.nFacets()<<" facets. "<<std::endl;
    std::cout<<mesher.nBorderEdges()<<" border edges"<<std::endl;
    std::cout<<"Reconstructing the mesh took "<<difftime(end,start)
             <<"s."<<std::endl;

    std::cout << "Filling holes..." << std::endl;
    mesher.fillHoles();

    std::set<Facet*>& boundary_facets = mesher.getBoundaryFacets();
    std::cout << "Address of boundary_facets: " << &boundary_facets << std::endl;
    mesher.trimBoundaryFacets(boundary_facets);

    std::cout << "Number of ball centers: " << mesher.getNumBallCenters() << std::endl;
    
    // Function to write ball centers to a file
    auto writeBallCentersToFile = [&mesher](const std::string& filename) {
        std::ofstream outFile(filename);
        if (!outFile.is_open()) {
            std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
            return;
        }
        
        outFile << "x y z" << std::endl;  // Header
        
        //const auto& ballCenters = mesher.getBallCenters();
        const Point_list& ballCenters = mesher.getBallCenters();
        for (const auto& center : ballCenters) {
            outFile << center.x() << " " << center.y() << " " << center.z() << std::endl;
        }
        
        outFile.close();
        std::cout << "Ball centers written to " << filename << std::endl;
    };
    
    // Call the function to write ball centers
    writeBallCentersToFile("ball_centers.txt");

    //std::time(&start);
    //mesher.fillHoles();
    //std::time(&end);

    //std::cout<<difftime(end,start)<<" s."<<std::endl;
    //std::cout<<"Final mesh: "<<mesher.nVertices()
    //         <<" vertices; "<<mesher.nFacets()<<" facets. "<<std::endl;
    //std::cout<<mesher.nBorderEdges()<<" border edges"<<std::endl;

    std::cout << "Saving mesh to " << outfile << std::endl;
    if(! FileIO::saveMesh(outfile.c_str(), mesher))
    {
        std::cerr<<"Pb saving the mesh; exiting."<<std::endl;
        return EXIT_FAILURE;
    }
    std::cout<<"Mesh saved in "<<outfile<<std::endl;

    std::cout << "Saving the debug mesh to " << outfile << std::endl;

    std::string debug_outfile = outfile;
    debug_outfile.erase(debug_outfile.find(".ply"), 4);
    debug_outfile += "_debug.txt";

    if(! FileIO::saveMeshDebug(debug_outfile.c_str(), mesher))
    {
        std::cerr<<"Pb saving the mesh; exiting."<<std::endl;
        return EXIT_FAILURE;
    }
    std::cout<<"Mesh saved in "<<debug_outfile<<std::endl;

    return EXIT_SUCCESS;
}
