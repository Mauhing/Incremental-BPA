/**
 * @file BallCenter.h
 * @brief declares methods for building a surface mesh from points stored in an
 * octree
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
 */

#ifndef BALLCENTER_H
#define BALLCENTER_H

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <set>

#include "Point.h"

class Facet;

using namespace std;

/**
 * @class BallCenter
 * @brief Ball center of a facet
 *
 */
class BallCenter : public Point
{
public:
    friend ostream &operator<<(ostream &out, const BallCenter &v);

private: // properties
    Facet *m_facet;

    TOctreeNode<BallCenter> *m_octree_node;

public: // constructor+destructor
    /** @brief default constructor*/
    BallCenter() = delete;

    /** @brief constructor from coordinates and facet*/
    BallCenter(double x, double y, double z, Facet *facet);

    /** @brief constructor from point and facet*/
    BallCenter(const Point &point, Facet *facet);

    /** @brief default destrictor*/
    ~BallCenter();

    /** @brief copy constructor*/
    BallCenter(const BallCenter &other);

public: // accessors + modifiers
    /** @brief set the octree node*/
    void setOctreeNodeLeaf(TOctreeNode<BallCenter> *node);

    /** @brief get the facet
     * @return the facet
     */
    Facet *getFacet() const;
};
#endif