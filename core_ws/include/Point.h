/**
 * @file Point.h
 * @brief Declares a generic point class
 * @author Julie Digne
 * @date 2012/10/10
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
 * 
 * @author Mau Hing Yip mauhingyip@hotmail.com
 * @date 2025-06-12
 * @note This file was further developed based on prior work by Julie Digne.
 */

#ifndef POINT_H
#define POINT_H

#include <iostream>

template <typename T>
class TOctreeNode;
/**
 * @class Point
 * @brief Generic unoriented point
 *
 * The most standard 3D point structure: only contains three coordinates
 */
class Point
{
public:
    /** @brief default constructor*/
    Point();

    /** @brief constructor
     * @param x x coordinate
     * @param y y coordinate
     * @param z z coordinate
     */
    Point(double x, double y, double z);

    /** @brief destructor*/
    ~Point();

    /** @brief access x coordinate
     * @return x
     */
    double x() const;

    /** @brief access y coordinate
     * @return y
     */
    double y() const;

    /** @brief access y coordinate
     * @return y
     */
    double z() const;

    /** @brief copy constructor
     * @param other Point to copy from
     */
    Point(const Point& other);

    /** @brief assignment operator
     * @param other Point to copy from
     */
    Point& operator=(const Point& other);

    /**
     * @brief set the octree node leaf
     * This function does nothing. It exists to ensure default implementation of the function.
     * This function is meant to be overridden by the derived classes.
     * @param node octree node
     */
    void setOctreeNodeLeaf(TOctreeNode<Point> *node)
    {
        (void)node; // Mark the parameter as unused
    };

    /**
     * @brief output operator
     * @param out output stream
     * @param v point
     * @return output stream
     */
    friend std::ostream &operator<<(std::ostream &out, const Point &v);

private:
    /** @brief x coordinate*/
    double m_x;

    /** @brief y coordinate*/
    double m_y;

    /** @brief z coordinate*/
    double m_z;
};

#endif