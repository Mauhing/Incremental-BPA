/** @file FileIO.h
 * @brief file declaring methods to read points from a file
 * @author Julie Digne julie.digne@liris.cnrs.fr
 * @date 2012/10/22
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

#ifndef FILEIO_H
#define FILEIO_H

#include <fstream>
#include <iostream>

#include "Octree.h"
#include "Mesher.h"
#include "types.h"
#include "FileParsePose.h"

/**
 * @class FileIO
 * @brief class providing access to input/output operations
 *
 * This class defines methods for reading points from an input file
 * and saving the resulting mesh in a ply file.
 */
class FileIO
{
public:
    /** @brief constructor*/
    FileIO();

    /** @brief destructor*/
    ~FileIO();

private:
    static std::string baseOutputFilename;

public:
    static void setBaseOutputFilename(const std::string &filename);

public:
    /** @brief read points and poses from a file
     * @param filename name of the file to read points from
     * @return the points and poses
     */ 
    static std::vector<SensorFrame> readIntoFileBatch(const char *filename, bool pcd_with_normal);

    template <typename T>
    static std::tuple<Point, double, unsigned int> originAndDepth(
        const std::list<T>& points,
        double min_radius, 
        typename std::enable_if<std::is_base_of<Point, T>::value>::type* = nullptr)
    {
        Eigen::MatrixXd points_matrix(points.size(), 3);
        size_t i = 0;
        for(auto &point : points) {
            points_matrix(i, 0) = point.x();
            points_matrix(i, 1) = point.y(); 
            points_matrix(i, 2) = point.z();
            i++;
        }

        Eigen::Vector3d min_xyz = points_matrix.colwise().minCoeff();
        Eigen::Vector3d max_xyz = points_matrix.colwise().maxCoeff();

        double xmin = min_xyz(0);
        double ymin = min_xyz(1); 
        double zmin = min_xyz(2);
        double xmax = max_xyz(0);
        double ymax = max_xyz(1);
        double zmax = max_xyz(2);

        double lx = xmax - xmin;
        double ly = ymax - ymin;
        double lz = zmax - zmin;

        // Get size of one of the largest dimension
        double size = lx > ly ? lx : ly;
        size = size > lz ? size : lz;

        size = 1.1 * size;
        double margin;

        unsigned int depth = 0;
        if (min_radius > 0)
        {
            depth = (unsigned int)ceil(log2(size / (min_radius)));
            double adapted_size = pow2(depth) * min_radius;
            margin = 0.5 * (adapted_size - size);
            size = adapted_size;
        }
        else
        {
            std::cerr << "Warning: min_radius has to bigger than 0" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        // The orgin of the octree is the lower left corner (2D) of the bounding box
        double ox = xmin - margin;
        double oy = ymin - margin;
        double oz = zmin - margin;
        
        Point origin(ox, oy, oz);
        return std::tuple<Point, double, unsigned int>(origin, size, depth);
    }


public:
    /** @brief read points and poses from a file
     * @param sensor_frames the sensor frames
     * @param batch_index the batch index
     * @param batch_size the batch size
     * @return the points and poses
     */
    static std::pair<std::list<Vertex>, Eigen::Matrix<double, 3, 4>> readFromBatchToList(const std::vector<SensorFrame> &sensor_frames, size_t batch_index, size_t batch_size);

public: // debug
    /** @brief save triangulation
     * @param filename name of the file to save to
     * @param mesher name of the mesher to save vertices and facets from
     * @return false if something went wrong
     */
    static bool debugSaveLineset(const char *filename, const Edge_star_list &border_edges);

    /** @brief save points
     * @param filename name of the file to save to
     * @param octree the octree to save
     * @return false if something went wrong
     */
    static bool debugSavePoints(const char *filename, OctreeVertices &octree);

    /** @brief save ball centers
     * @param filename name of the file to save to
     * @param ball_centers the ball centers to save
     * @param radius the radius of the ball
     * @return false if something went wrong
     */
    static bool debugSaveBallCenters(const char *filename, const Point_UnOrdSet &ball_centers, const double &radius);

    /** @brief save points inside balls
     * @param filename name of the file to save to
     * @param vertices_inside_balls the points inside balls to save
     * @return false if something went wrong
     */
    static bool debugSavePoints(const char *filename, const Point_UnOrdSet &vertices_inside_balls);

    /** @brief save the mesh in a ply file
     * @param filename name of the file to save the mesh to
     * @param facets the facets to save
     * @return false if the file could not be opened
     */
    static bool saveMeshDebug(const char *filename, const std::list<Facet *> &facets);

private:
    /** @brief save all vertices contained in a node
     * @param node node to save from
     * @param f stream to save to
     */
    static void debugSaveContent(OctreeNodeV *node, std::ofstream &f);
};

#endif
