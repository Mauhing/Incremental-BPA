/** @file FileIO.cpp
 * @brief file defining methods to read points from a file
 * see FileIO.h for methods declaration
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

#include "FileIO.h"
#include <iostream>
#include <fstream>
#include <istream>
#include <ostream>
#include <sstream>
#include <limits>

using namespace std;

FileIO::FileIO()
{
}

FileIO::~FileIO()
{
}

std::string FileIO::baseOutputFilename;

bool FileIO::debugSavePoints(const char *filename, OctreeVertices &octree)
{
    ofstream out;
    out.open(filename);
    out.precision(numeric_limits<double>::digits10 + 1);

    if (!out)
        return false;

    OctreeNodeV *node = octree.getRoot();
    debugSaveContent(node, out);

    out.close();

    return true;
}

void FileIO::debugSaveContent(OctreeNodeV *node, ofstream &f)
{
    if (node->getDepth() != 0)
    {
        for (int i = 0; i < 8; i++)
            if (node->getChild(i) != nullptr)
                debugSaveContent(node->getChild(i), f);
    }
    else if (node->getNpts() != 0)
    {
        // Vertex_list::const_iterator iter;
        // for(iter = node->points_begin(); iter != node->points_end();
        //     ++iter)
        //{
        //     f << *iter <<std::endl;
        // }
        Vertex_UnOrdSet::const_iterator iter;
        for (iter = node->points_begin(); iter != node->points_end();
             ++iter)
        {
            Vertex *v = *iter;
            f << *v << std::endl;
        }
    }
}

bool FileIO::saveMeshDebug(const char *output_filename, const std::list<Facet *> &facets)
{
    std::string filename = baseOutputFilename + output_filename;

    ofstream out;
    out.open(filename);
    out.precision(numeric_limits<double>::digits10 + 1);

    if (!out)
    { // file couldn't be opened
        cerr << "Error: file could not be opened" << endl;
        return false;
    }

    out << "The first section will be idx, x, y, z, nx, ny, nz" << endl;
    out << "The second section will be 3, idx0, idx1, idx2" << endl;

    std::unordered_map<int, Vertex *> vertices;
    for (std::list<Facet *>::const_iterator fi = facets.begin();
         fi != facets.end(); ++fi)
    {
        for (int i = 0; i < 3; ++i)
        {
            const Facet *f = *fi;
            Vertex *v = f->vertex(i);
            vertices[v->index()] = v;
        }
    }
    for (auto &pair : vertices)
    {
        Vertex *v = pair.second;
        out << v->index() << "\t" << v->x() << "\t" << v->y() << "\t" << v->z() << "\t" << v->nx() << "\t" << v->ny() << "\t" << v->nz() << endl;
    }

    // add a blank line
    out << endl;

    for (std::list<Facet *>::const_iterator fi = facets.begin();
         fi != facets.end(); ++fi)
    {
        out << 3 << "\t";
        const Facet *f = *fi;

        Vertex *v0 = f->vertex(0);
        Vertex *v1 = f->vertex(1);
        Vertex *v2 = f->vertex(2);

        double nx = v0->nx() + v1->nx() + v2->nx();
        double ny = v0->ny() + v1->ny() + v2->ny();
        double nz = v0->nz() + v1->nz() + v2->nz();

        double tx = v1->x() - v0->x();
        double ty = v1->y() - v0->y();
        double tz = v1->z() - v0->z();

        double sx = v2->x() - v0->x();
        double sy = v2->y() - v0->y();
        double sz = v2->z() - v0->z();

        double cprodx, cprody, cprodz;
        cross_product(tx, ty, tz, sx, sy, sz, cprodx, cprody, cprodz);

        // << Ensure orientation
        if (nx * cprodx + ny * cprody + nz * cprodz > 0)
        {
            out << v0->index() << "\t";
            out << v1->index() << "\t";
            out << v2->index() << endl;
        }
        else
        {
            out << v0->index() << "\t";
            out << v2->index() << "\t";
            out << v1->index() << endl;
        }
        // >>
    }
    out.close(); // close the file
    return true;
}

 
std::pair<std::list<Vertex>, Eigen::Matrix<double, 3, 4>> FileIO::readFromBatchToList(const std::vector<SensorFrame> &sensor_frames, size_t batch_index, size_t batch_size)
{
list<Vertex> input_vertices;

for (size_t i = 0; i < batch_size; i++) {
    const Eigen::Matrix<double, 3, 4> &pose = sensor_frames[batch_index + i].pose;
    const std::vector<Point3D> &points = sensor_frames[batch_index + i].points;
    for (const auto &point : points) {
        // Each point also have normal. it is calculated by (tx, ty, tz) - (xi, yi, zi) and normalize it.
        double tx = pose(0, 3);
        double ty = pose(1, 3);
        double tz = pose(2, 3);
        Eigen::Vector3d normal = (Eigen::Vector3d(tx, ty, tz) - Eigen::Vector3d(point.x, point.y, point.z)).normalized();

        input_vertices.push_back(Vertex(point.x, point.y, point.z, normal.x(), normal.y(), normal.z()));
    }
}

const Eigen::Matrix<double, 3, 4> pose = sensor_frames[batch_index + size_t(batch_size/2)].pose;

return std::make_pair(input_vertices, pose);
}

bool FileIO::debugSaveLineset(const char *output_filename, const Edge_star_list &border_edges)
{
    std::string filename = baseOutputFilename + output_filename;

    ofstream out;
    out.open(filename);
    out.precision(numeric_limits<double>::digits10 + 1);

    if (!out)
    { // file couldn't be opened
        cerr << "Error: file could not be opened" << endl;
        return false;
    }

    for (auto &edge : border_edges)
    {
        Vertex *v0 = edge->getSource();
        Vertex *v1 = edge->getTarget();
        out << v0->x() << "\t" << v0->y() << "\t" << v0->z() << "\t" << v1->x() << "\t" << v1->y() << "\t" << v1->z() << endl;
    }
    out.close(); // close the file
    return true;
}

bool FileIO::debugSaveBallCenters(const char *output_filename, const Point_UnOrdSet &ball_centers, const double &radius)
{
    std::string filename = baseOutputFilename + output_filename;

    ofstream out;
    out.open(filename);
    out.precision(numeric_limits<double>::digits10 + 1);

    if (!out)
    { // file couldn't be opened
        cerr << "Error: file could not be opened" << endl;
        return false;
    }

    // First line is explaination
    out << "Line 2: radius. Rest of the lines are x, y, z" << endl;

    // Save the radius
    out << radius << endl;

    // Save the ball centers
    for (auto &center : ball_centers)
    {
        out << center->x() << "\t" << center->y() << "\t" << center->z() << endl;
    }

    out.close();
    return true;
}

bool FileIO::debugSavePoints(const char *output_filename, const Point_UnOrdSet &points)
{
    std::string filename = baseOutputFilename + output_filename;

    ofstream out;
    out.open(filename);
    out.precision(numeric_limits<double>::digits10 + 1);

    if (!out)
    { // file couldn't be opened
        cerr << "Error: file could not be opened" << endl;
        return false;
    }

    // Save the points
    for (auto &point : points)
    {
        out << point->x() << "\t" << point->y() << "\t" << point->z() << endl;
    }

    out.close();
    return true;
}

void FileIO::setBaseOutputFilename(const std::string &filename)
{
    baseOutputFilename = filename;
}

std::vector<SensorFrame> FileIO::readIntoFileBatch(const char *filename)
{
    return OfflineDataParser::parseFile(filename);
} 

//std::tuple<Point, double, unsigned int> FileIO::originAndDepth(const SensorFrame &sensor_frames, double min_radius)
//{
//
//    //const Eigen::Matrix<double, 3, 4> &pose = first_frame.pose;
//    const std::vector<Point3D> &points = sensor_frames.points;
//
//    return originAndDepth(points, min_radius);
//}