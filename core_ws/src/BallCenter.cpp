/**
 * @file BallCenter.cpp
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

#include "BallCenter.h"
#include "OctreeNode.h"

BallCenter::BallCenter(double x, double y, double z, Facet *facet) : Point(x, y, z),
                                                                     m_facet(facet),
                                                                     m_octree_node(nullptr)
{
}

BallCenter::BallCenter(const Point &point, Facet *facet) : Point(point),
                                                           m_facet(facet),
                                                           m_octree_node(nullptr)
{
}

BallCenter::~BallCenter()
{
    if (m_octree_node != nullptr)
    {
        m_octree_node->removeElement(this);
        m_octree_node = nullptr;
    }
    #ifdef _DEBUG
    if (m_octree_node != nullptr)
    {
        std::cerr << "Error: m_octree_node is not nullptr" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    #endif
    m_facet = nullptr;
    m_octree_node = nullptr;
}

BallCenter::BallCenter(const BallCenter &other) : Point(other),
                                                  m_facet(other.m_facet),
                                                  m_octree_node(other.m_octree_node)
{
}

ostream &operator<<(ostream &out, const BallCenter &v)
{
    out << "BallCenter: " << v.x() << " " << v.y() << " " << v.z() << std::endl;
    return out;
}

void BallCenter::setOctreeNodeLeaf(TOctreeNode<BallCenter> *node)
{
    m_octree_node = node;
}

Facet *BallCenter::getFacet() const
{
    return m_facet;
}

