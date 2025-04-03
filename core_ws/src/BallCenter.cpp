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

