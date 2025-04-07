#ifndef OCTREENODE_HPP
#define OCTREENODE_HPP

#include "OctreeNode.h"

template <class T>
TOctreeNode<T>::TOctreeNode()
{
    for (int i = 0; i < 8; i++)
        m_child[i] = nullptr;
    m_parent = nullptr;
    m_xloc = m_yloc = m_zloc = 0;
    m_depth = 0;
    m_origin = Point();
    m_size = 0.0;
    m_points.clear();
}

template <class T>
TOctreeNode<T>::TOctreeNode(const Point &origin, double size, unsigned int depth)
{
    for (int i = 0; i < 8; i++)
        m_child[i] = nullptr;
    m_parent = nullptr;
    m_xloc = m_yloc = m_zloc = 0;
    m_depth = depth;
    m_origin = origin;
    m_size = size;
    m_points.clear();
}

template <class T>
TOctreeNode<T>::~TOctreeNode()
{
    for (T *element : m_points)
    {
        delete element;
    }
    m_points.clear();
    m_xloc = m_yloc = m_zloc = 0;
    m_depth = 0;
    for (int i = 0; i < 8; i++)
    {
        delete m_child[i];
        m_child[i] = nullptr;
    }
    m_parent = nullptr;
    m_origin = Point();
    m_size = 0.0;
}

template <class T>
unsigned int TOctreeNode<T>::getDepth() const
{
    return m_depth;
}

template <class T>
void TOctreeNode<T>::setDepth(unsigned int l)
{
    m_depth = l;
}

template <class T>
double TOctreeNode<T>::getSize() const
{
    return m_size;
}

template <class T>
void TOctreeNode<T>::setSize(double size)
{
    m_size = size;
}

template <class T>
unsigned int TOctreeNode<T>::getNpts() const
{
    return static_cast<unsigned int>(m_points.size());
}

template <class T>
const unsigned int &TOctreeNode<T>::getNChild() const
{
    return m_nchild;
}

template <class T>
void TOctreeNode<T>::setNchild(unsigned int a)
{
    m_nchild = a;
}

template <class T>
void TOctreeNode<T>::setParent(TOctreeNode *parent)
{
    m_parent = parent;
}

template <class T>
TOctreeNode<T> *TOctreeNode<T>::getParent() const
{
    return m_parent;
}

template <class T>
TOctreeNode<T> *TOctreeNode<T>::getChild(unsigned int index)
{
    unsigned int i = index % 8;
    return m_child[i];
}

template <class T>
unsigned int TOctreeNode<T>::getXLoc() const
{
    return m_xloc;
}

template <class T>
void TOctreeNode<T>::setXLoc(unsigned int xloc)
{
    m_xloc = xloc;
}

template <class T>
unsigned int TOctreeNode<T>::getYLoc() const
{
    return m_yloc;
}

template <class T>
void TOctreeNode<T>::setYLoc(unsigned int yloc)
{
    m_yloc = yloc;
}

template <class T>
unsigned int TOctreeNode<T>::getZLoc() const
{
    return m_zloc;
}

template <class T>
void TOctreeNode<T>::setZLoc(unsigned int zloc)
{
    m_zloc = zloc;
}

template <class T>
bool TOctreeNode<T>::isInside(double x, double y, double z) const
{

    if ((x >= m_origin.x()) && (x < m_origin.x() + m_size) && (y >= m_origin.y()) && (y < m_origin.y() + m_size) && (z >= m_origin.z()) && (z < m_origin.z() + m_size))
        return true;
    else
        return false;
}

template <class T>
bool TOctreeNode<T>::isInside(const Point &p) const
{
    if ((p.x() >= m_origin.x()) && (p.x() < m_origin.x() + m_size) && (p.y() >= m_origin.y()) && (p.y() < m_origin.y() + m_size) && (p.z() >= m_origin.z()) && (p.z() < m_origin.z() + m_size))
        return true;
    else
        return false;
}

template <class T>
bool TOctreeNode<T>::isInside(const Point &p, double d) const
{
    double offset = m_size + d;
    if ((p.x() >= m_origin.x() - d) && (p.x() < m_origin.x() + offset) && (p.y() >= m_origin.y() - d) && (p.y() < m_origin.y() + offset) && (p.z() >= m_origin.z() - d) && (p.z() < m_origin.z() + offset))
        return true;
    else
        return false;
}

template <class T>
void TOctreeNode<T>::setOrigin(Point &pt)
{
    m_origin = pt;
}

template <class T>
Point TOctreeNode<T>::getOrigin() const
{
    return m_origin;
}

template <class T>
typename std::unordered_set<T *>::const_iterator TOctreeNode<T>::points_begin()
{
    return m_points.cbegin();
}

template <class T>
typename std::unordered_set<T *>::const_iterator TOctreeNode<T>::points_end()
{
    return m_points.cend();
}

template <class T>
T *TOctreeNode<T>::addPoint(const T &t)
{ 
    if constexpr (std::is_same<T, BallCenter>::value)
    {
        T *t_ptr = new T(t);
        t_ptr->setOctreeNodeLeaf(this);
        m_points.insert(t_ptr);
        return t_ptr;
    }

    if constexpr (std::is_same<T, Vertex>::value)
    {
        T *t_ptr = new T(t);
        t_ptr->setOctreeNodeLeaf(this);
        m_points_buffer.insert(t_ptr);
        return t_ptr;
    }

    #ifdef _DEBUG
    {
        std::cerr << "\033[33mWarning: Adding non-Vertex or BallCenter type to octree\033[0m" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    #endif
    
}

template <class T>
TOctreeNode<T> *TOctreeNode<T>::initializeChild(unsigned int index, Point
                                                                        origin)
{
    double size = m_size / 2.0;
    unsigned int depth = m_depth - 1;
    m_child[index] = new TOctreeNode<T>(origin, size, depth);
    m_child[index]->setParent(this);
    m_child[index]->setNchild(index);

    return m_child[index];
}

template <class T>
void TOctreeNode<T>::setChild(unsigned int index, TOctreeNode<T> *node)
{
    m_child[index] = node;
}

template <class T>
std::unordered_set<T *> &TOctreeNode<T>::GetPoints() { return m_points; }

template <class T>
void TOctreeNode<T>::removeElement(T *element)
{
    m_points.erase(element);
}

template <class T>
bool TOctreeNode<T>::isLeaf() const
{
    return m_depth == 0;
}

template <class T>
void TOctreeNode<T>::downSample(const int max_points, 
                        std::vector<T*> &recruited_points, 
                        const bool &random_device, const int &seed)
{
    if (!isLeaf())
    {
        for (unsigned int i = 0; i < 8; i++)
        {
            TOctreeNode<T> *node = getChild(i);
            if (node != nullptr)
            {
                node->downSample(max_points, recruited_points, random_device, seed);
            }
        }
    }
    else
    {
        if (static_cast<int>(m_points_buffer.size()) > 0)
        {
            std::vector<T*> temp_points;
            temp_points.reserve(m_points.size() + m_points_buffer.size());

            // Move orphan points from m_points to temp_points
            for (auto it = m_points.begin(); it != m_points.end();) {
                if ((*it)->getType() == T::VertexType::ORPHAN) {
                    temp_points.push_back(*it);
                    it = m_points.erase(it);
                } else {
                    ++it;
                }
            }
            
            // Add buffer points to temp_points
            for (auto* point : m_points_buffer) {
                temp_points.push_back(point);
            }
            
            // Random shuffle
            if (random_device) {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::shuffle(temp_points.begin(), temp_points.end(), gen);
            } else {
                std::mt19937 gen(seed);
                std::shuffle(temp_points.begin(), temp_points.end(), gen);
            }
            
            // Store points that will be deleted
            std::vector<T*> points_to_delete;
            if (static_cast<int>(temp_points.size()) > max_points) {
                // Split temp_points into two vectors at max_points
                points_to_delete.assign(temp_points.begin() + max_points, temp_points.end());
                temp_points.resize(max_points);
            }
            
            // Delete points and free memory
            for (T* point : points_to_delete) {
                delete point;
            }

            // Add selected points back to m_points
            for (T* point : temp_points) {
                m_points.insert(point);
            }

            // Add recruited points
            for (T* point : temp_points) {
                if (m_points_buffer.find(point) != m_points_buffer.end()) {
                    recruited_points.push_back(point);
                }
            }
            m_points_buffer.clear();
        }

        // memory leak here.
    }
}

template <class T>
unsigned int TOctreeNode<T>::getOrdinalPositionForChild() const
{
    return m_depth - 1;
}

template <class T>
void TOctreeNode<T>::checkAllPointsInVolume()
{
    for (T* point : m_points)
    {
        if (!isInside(*point))
        {
            std::cerr << "In function checkAllPointsInVolume: first loop" << std::endl;
            std::cerr << "Point is not in the volume" << std::endl;
            std::cerr << "Point: " << point->x() << ", " << point->y() << ", " << point->z() << std::endl;
            std::cerr << "Volume: " << m_origin.x() << ", " << m_origin.y() << ", " << m_origin.z() << " to " << m_origin.x() + m_size << ", " << m_origin.y() + m_size << ", " << m_origin.z() + m_size << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }
    for (T* point : m_points_buffer)
    {
        if (!isInside(*point))
        {
            std::cerr << "In function checkAllPointsInVolume: second loop" << std::endl;
            std::cerr << "Point is not in the volume" << std::endl;
            std::cerr << "Point: " << point->x() << ", " << point->y() << ", " << point->z() << std::endl;
            std::cerr << "Volume: " << m_origin.x() << ", " << m_origin.y() << ", " << m_origin.z() << " to " << m_origin.x() + m_size << ", " << m_origin.y() + m_size << ", " << m_origin.z() + m_size << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }
}
#endif