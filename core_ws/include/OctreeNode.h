/**
 * @file OctreeNode.h
 * @brief defines an octree node
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

#ifndef OCTREENODE_H
#define OCTREENODE_H

#include <cstdlib>
#include <list>

#include "Point.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <random>
#include <vector>
#include <unordered_set>

class BallCenter;
class Vertex;

/**
 * @class TOctreeNode
 * @brief Implements a generic node for a generic octree
 *
 * Templated class implementing a node of the octree, leaf nodes
 * contain the input points.
 */
template <class T>
class TOctreeNode
{
protected:
    /**
     * @brief pointer to the parent of the node
     */
    TOctreeNode<T> *m_parent;

    /**
     * @brief pointer to the eight children of the node
     */
    TOctreeNode<T> *m_child[8];

    /**
     * @brief  child number of the node (depends on the relative location
     * of the node to the middle of its parent)
     *
     * \verbatim
     *   0-------4
     *  /|      /|
     * 2-------6 |
     * | 1-----|-5
     * |/      |/
     * 3-------7
     *
     * axis:
     * x: along direction 0->4
     * y: along direction 0->2
     * z: along direction 0->1
     * \endverbatim
     * REMARK: this convention is not important because we are
     * dealing with a cube. It will not affect neither
     * representation nor computations
     */
    unsigned int m_nchild;

    /** @brief origin of the node*/
    Point m_origin;

    /** @brief level of the node*/
    unsigned int m_depth;

    /**
     * @brief x locational code
     */
    unsigned int m_xloc;

    /**
     * @brief y locational code
     */
    unsigned int m_yloc;

    /**
     * @brief z locational code
     */
    unsigned int m_zloc;

    /**
     * @brief size of the node side
     */
    double m_size;

    /**
     * @brief unordered_set of points contained in the node
     * (empty if the node is not a leaf)
     */
    std::unordered_set<T *> m_points;

    /**
     * @brief buffer of points contained in the node
     * When the downsampling is done, this container will have no points.
     */
    std::unordered_set<T *> m_points_buffer;

public:
    /**
     * @brief Default constructor initializes all variables
     */
    TOctreeNode();

    /**
     * @brief Constructor initializes size, depth and origin
     * @param size;
     * @param origin
     * @param depth
     */
    TOctreeNode(const Point &origin, double size, unsigned int depth);

    /**
     * @brief Destructor
     */
    ~TOctreeNode();

    /**
     * @brief set node size
     * @param size desired node size
     */
    void setSize(double size);

    /**
     * @brief get node size
     * @return size of the node
     */
    double getSize() const;

    /** @brief returns the number of points contained in the node
     * @return number of points contained in the node (if leaf)
     * or node's children
     */
    unsigned int getNpts() const;

    /**
     * @brief set child number (depends on the relative location of the
     * node to the middle of its parent)
     * @param a child number
     */
    void setNchild(unsigned int a);

    /**
     * @brief get the child number
     * @return child number
     */
    const unsigned int &getNChild() const;

    /**
     * @brief set the origin of the node by a point structure
     * @param pt Origin point
     */
    void setOrigin(Point &pt);

    /** @brief get the origin of a given node
     * @return origin
     */
    Point getOrigin() const;

    /**
     * @brief set parent of a node
     * @param node pointer to the parent node
     */
    void setParent(TOctreeNode<T> *node);

    /**
     * @brief get parent of a node
     * @return TOctreeNode* pointer to the parent node
     */
    TOctreeNode<T> *getParent() const;

    /** @brief get child of a node
     * @param index of the child
     * @return child node
     */
    TOctreeNode<T> *getChild(unsigned int index);

    /** @brief set child of a node
     * @param index of the child
     * added by yip
     */
    void setChild(unsigned int index, TOctreeNode<T> *node);

    /**
     * @brief set level of a node
     * @param l level
     */
    void setDepth(unsigned int l);

    /**
     * @brief get depth of the node
     * @return depth
     */
    unsigned int getDepth() const;

    /**
     * @brief check if a point given by its coordinates is inside a node
     * @param x coordinates of the point
     * @param y
     * @param z
     * @return true if the point is inside
     */
    bool isInside(double x, double y, double z) const;

    /**
     * @brief check if a point given by its coordinates is inside a node
     * @param p  point
     * @return true if the point is inside
     */
    bool isInside(const Point &p) const;

    /**
     * @brief check if a point given by its coordinates is inside or
     * in a band around the node
     * @param p  point
     * @param d width of the band around the node
     * @return true if the point is inside
     */
    bool isInside(const Point &p, double d) const;

    /**
     * @brief locationnal code method
     * @return unsigned int x locationnal code of the node
     */
    unsigned int getXLoc() const;

    /**
     * @brief locationnal code method
     * @return unsigned int y locationnal code of the node
     */
    unsigned int getYLoc() const;

    /**
     * @brief locationnal code method
     * @return unsigned int z locationnal code of the node
     */
    unsigned int getZLoc() const;

    /**
     * @brief locationnal code method
     * @param Xloc locationnal code of the node
     */
    void setXLoc(unsigned int Xloc);

    /**
     * @brief locationnal code method
     * @param Yloc locationnal code of the node
     */
    void setYLoc(unsigned int Yloc);

    /**
     * @brief locationnal code method
     * @param Zloc locationnal code of the node
     */
    void setZLoc(unsigned int Zloc);

    /** @brief get a pointer to the list of points
     * @return pointer to the beginning of the list
     */
    // typename std::list<T>::iterator points_begin();
    typename std::unordered_set<T *>::const_iterator points_begin();

    /** @brief get a pointer to the end of the list of points
     * @return pointer to the end of 'points'
     */
    // typename std::list<T>::iterator points_end();
    typename std::unordered_set<T *>::const_iterator points_end();

    /** @brief add a point to the list of points included in the cell
     * PREREQUISITE: the node is a leaf in the octree
     * @param pt point to add
     */
    T *addPoint(const T &pt);

    /** @brief build the i^th child of the node
     * @param index child index
     * @param origin origin of the node
     * @return pointer to the created node
     */
    TOctreeNode<T> *initializeChild(unsigned int index, Point origin);

    /** @brief get the points contained in the node
     * @return the points contained in the node
     */
    std::unordered_set<T *> &GetPoints();

    /**
     * @brief remove a point from the node
     */
    void removeElement(T *element);

    /**
     * @brief check if the node is a leaf
     * @return true if the node is a leaf
     */
    bool isLeaf() const;

    /**
     * @brief downsample the node
     * @param max_points maximum number of points in the node
     * @param recruited_points vector of points to recruit
     */
    void downSample(const int max_points, std::vector<T*> &recruited_points, const bool &random_device, const int &seed); 

    /**
     * @brief get the digit location of the node
     * @return digit location
     */
    unsigned int getOrdinalPositionForChild() const;

    /**
     * @brief check if all points are in the volume
     */
    void checkAllPointsInVolume();
};

#include "OctreeNode.hpp"

#endif