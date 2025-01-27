/**
 * @file OctreeVertices.h
 * @brief declares an octree storing structure
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
 */

#ifndef OCTREE_H
#define OCTREE_H

#include "utilities.h"
#include "Point.h"
#include "OctreeNode.h"
#include "BallCenter.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath> // Use for log2
#include <thread>
#include <chrono>

#include <bitset> // Use for debugging

/**
 * @class TOctree
 * @brief Data structure for storing and sorting the points
 *
 * Templated octree data structure permitting to sort the input points
 */
template <class T>
class TOctree
{
public: // constructors/destructors
    /**
     * @brief default constructor
     */
    TOctree(const int max_points);

    /**
     * @brief Destructor
     */
    ~TOctree();

public: // accessors and modifiers
    /**
     * @brief get octree depth
     * @return depth
     */
    unsigned int getDepth() const;

    /**
     * @brief set octree depth
     * @param depth
     */
    void setDepth(unsigned int depth);

    /**
     * @brief get origign
     * @return origin of the octree
     */
    const Point &getOrigin() const;

    /**get side size of the octree
     * @return size of the octree
     */
    double getSize() const;

    /**
     * @brief set size of the octree
     * @param size
     */
    void setSize(double size);

    /**
     * @brief get the bin size
     * @return binsize
     */
    unsigned int getBinSize() const;

    /**
     * @brief get root of the octree
     * @return root of the octree
     */
    TOctreeNode<T> *getRoot() const;

    // copy constructor
    TOctree(const TOctree &other);

    unsigned int getMaxPoints() const;

public: // adding points
    /**
     * @brief initialize the octree with the origin and size
     * @param origin origin of the octree
     * @param size side size
     **/
    void initialize(const Point &origin, double size);

    /**
     * @brief Adding a point to the octree
     * @param pt point to add
     */
    T *addPoint(const T &pt);

    /**
     * @brief get all nodes at given depth
     * @param depth input depth
     * @param starting_node node to start from
     * @param[out] nodes vector of nodes
     * */
    void getNodes(unsigned int depth, TOctreeNode<T> *starting_node,
                  std::vector<TOctreeNode<T> *> &nodes);

    /**
     * @brief get nodes in separate buckets according to the
     * parity of the indices with respect to the three
     * coordinates (8 buckets)
     * process each node separately
     * @param depth input depth
     * @param starting_node node to start from
     * @param[out] node_collection vector of vector of nodes
     * (each vector of nodes can be processed separately)
     */
    void getNodes(unsigned int depth, TOctreeNode<T> *starting_node,
                  std::vector<std::vector<TOctreeNode<T> *>> &node_collection);

    T *checkSizeAndaddPoint(const T &pt);

    // void debugPrint()
    //{
    //     for (auto* node: m_created_nodes)
    //     {
    //         std::cout << " <<<< " << std::endl;
    //         std::cout << "Node: depth " << node->getDepth() << " size " << node->getSize() << std::endl;
    //         std::cout << "X start " << node->getOrigin().x() << std::endl;
    //         std::cout << "X end " << node->getOrigin().x() + node->getSize() << std::endl;
    //         std::cout << "Depth in binary " << std::bitset<32>(node->getDepth()) << std::endl;
    //         std::cout << "XLoc in binary: " << std::bitset<32>(node->getXLoc()) << std::endl;
    //         if (node->getDepth() == 0) {
    //             std::cout << "Number of pts contained: " << node->getNpts() << std::endl;
    //         }
    //     }
    // }

    void checkSizeAndexpand(const T &pt);

protected:
    /**
     *@brief Maximum depth of the octree
     * Given that m_root_depth is n, the are total n+1 levels
     * and the root is at level n. The leaf nodes are at level 0.
     */
    unsigned int m_root_depth;


    /**
     * @brief Origin of the octree
     */
    Point m_origin;

    /**
     * @brief size of the side of the entire octree
     */
    double m_size;

    /**
     * @brief Total number of interval; interval = pow2(depth)
     * the bin size is an octree parameter that determines*
     * the locational code of each node
     */
    unsigned int m_nb_interval;

    /**
     *@brief root of the octree
     */
    TOctreeNode<T> *m_root;

    /**
     *@brief number of non-empty cells per level
     */
    // std::vector<unsigned int> m_nb_non_empty_cells;

    /**
     * @brief maximum number of points per node
     */
    const int m_max_points;

private: // helper functions to make another function shorter
    void expandAllNodeLoc(unsigned int level, unsigned int x_insert_index, unsigned int y_insert_index, unsigned int z_insert_index);

    void updateNodeLoc(TOctreeNode<T> *node, unsigned int level, unsigned int x_insert_index, unsigned int y_insert_index, unsigned int z_insert_index);

#ifdef _DEBUG
public:
    template <typename U>
    TOctree<U> debugCopySkeleton() const;

    /**
     * @brief print the mean number of points per non
     * empty cell at each level
     */
    void debugPrintStats();
#endif

   public:
   void integrityCheck();
};

#include "Octree.hpp"

#endif