#ifndef OCTREE_HPP
#define OCTREE_HPP

#include "Octree.h"  // Include the header file

template <class T>
void TOctree<T>::expandAllNodeLoc(unsigned int level, unsigned int x_insert_index, unsigned int y_insert_index, unsigned int z_insert_index)
{
    updateNodeLoc(m_root, level, x_insert_index, y_insert_index, z_insert_index);
}

template <class T>
void TOctree<T>::updateNodeLoc(TOctreeNode<T> *node, unsigned int level, unsigned int x_insert_index, unsigned int y_insert_index, unsigned int z_insert_index)
{
    if (node->getDepth() != 0)
    {
        for (unsigned int i = 0; i < 8; i++)
        {
            if (node->getChild(i) != nullptr)
            {
                updateNodeLoc(node->getChild(i), level, x_insert_index, y_insert_index, z_insert_index);
            }
        }
    }
    node->setXLoc(node->getXLoc() + (x_insert_index << (level)));
    node->setYLoc(node->getYLoc() + (y_insert_index << (level)));
    node->setZLoc(node->getZLoc() + (z_insert_index << (level)));
}

template <class T>
TOctree<T>::TOctree(const int max_points): m_max_points(max_points)
{
    m_size = 0;
    m_root_depth = 0;
    m_nb_interval = 0;
    m_origin = Point();
    m_root = new TOctreeNode<T>();
    m_root->setDepth(0);
}


template <class T>
TOctree<T>::~TOctree()
{
    std::cout << "TOctree destructor" << std::endl;
    m_size = 0;
    m_root_depth = 0;
    m_nb_interval = 0;
    m_origin = Point();

    if (m_root != nullptr)
    {
        delete m_root;
        m_root = nullptr;
    }
}

template <class T>
void TOctree<T>::initialize(const Point &origin, double size)
{
    m_size = size;
    m_origin = origin;

    if (m_root != nullptr)
    {
        std::cout << "The initializer detected that the root is not nullptr" << std::endl;
        std::cout << "resetting the root" << std::endl;
        delete m_root;
        m_root = nullptr;
    }

    m_root = new TOctreeNode<T>(m_origin, m_size, m_root_depth);

    // Setting the locational code for root is actually not necessary
    // However, it is done here for consistency
    m_root->setXLoc(0);
    m_root->setYLoc(0);
    m_root->setZLoc(0);

    m_root->setParent(nullptr);
}

template <class T>
unsigned int TOctree<T>::getDepth() const
{
    return m_root_depth;
}

template <class T>
void TOctree<T>::setDepth(unsigned int depth)
{
    m_root_depth = depth;
    m_nb_interval = pow2(depth);
}

template <class T>
double TOctree<T>::getSize() const
{
    // The size is the physical length
    return m_size;
}

template <class T>
void TOctree<T>::setSize(double size)
{
    m_size = size;
}

template <class T>
unsigned int TOctree<T>::getBinSize()
    const
{
    return m_nb_interval;
}

template <class T>
const Point &TOctree<T>::getOrigin() const
{
    return m_origin;
}

template <class T>
TOctreeNode<T> *TOctree<T>::getRoot() const
{
    return m_root;
}


enum PointRespectToBox
{
    RIGHT = 0,
    LEFT = 1,
    INSIDE = 2
};
template <class T>
void TOctree<T>::checkSizeAndexpand(const T &pt)
{
    // 0 means the x coordinate is larger than the right border of the octree
    // 1 means the x coordinate is smaller than the left border of the octree
    // 2 means the x coordinate is within the octree
    PointRespectToBox x_in_box = PointRespectToBox::INSIDE;
    PointRespectToBox y_in_box = PointRespectToBox::INSIDE;
    PointRespectToBox z_in_box = PointRespectToBox::INSIDE;
    unsigned int n_x = 0;
    unsigned int n_y = 0;
    unsigned int n_z = 0;

    if (pt.x() > (m_origin.x() + m_size))
    {
        n_x = static_cast<unsigned int>(std::ceil(log2((pt.x() - m_origin.x()) / m_size)));
        x_in_box = PointRespectToBox::RIGHT;
    }
    if (pt.x() < m_origin.x())
    {
        n_x = static_cast<unsigned int>(ceil(log2((m_origin.x() + m_size - pt.x()) / m_size)));
        x_in_box = PointRespectToBox::LEFT;
    }

    if (pt.y() > (m_origin.y() + m_size))
    {
        n_y = static_cast<unsigned int>(ceil(log2((pt.y() - m_origin.y()) / m_size)));
        y_in_box = PointRespectToBox::RIGHT;
    }
    if (pt.y() < m_origin.y())
    {
        n_y = static_cast<unsigned int>(ceil(log2((m_origin.y() + m_size - pt.y()) / m_size)));
        y_in_box = PointRespectToBox::LEFT;
    }

    if (pt.z() > (m_origin.z() + m_size))
    {
        n_z = static_cast<unsigned int>(ceil(log2((pt.z() - m_origin.z()) / m_size)));
        z_in_box = PointRespectToBox::RIGHT;
    }
    if (pt.z() < m_origin.z())
    {
        n_z = static_cast<unsigned int>(ceil(log2((m_origin.z() + m_size - pt.z()) / m_size)));
        z_in_box = PointRespectToBox::LEFT;
    }

    if (x_in_box != PointRespectToBox::INSIDE || y_in_box != PointRespectToBox::INSIDE || z_in_box != PointRespectToBox::INSIDE)
    {

#ifdef _DEBUG
    // print the class name of the object
    std::cout << "Type: " << (std::is_same<T, Vertex>::value ? "Vertex" :
                            std::is_same<T, BallCenter>::value ? "BallCenter" :
                            "Unknown") << std::endl;
    if (x_in_box != PointRespectToBox::INSIDE) {
        std::cout << "x outside the box" << std::endl;
        std::cout << "n_x: " << n_x << std::endl;
        std::cout << "x_in_box: " << x_in_box << std::endl;
        std::cout << "Point location: " << pt.x() << " " << pt.y() << " " << pt.z() << std::endl;
        std::cout << "Lower bound: " << m_origin.x() << " " << m_origin.y() << " " << m_origin.z() << std::endl;
        std::cout << "Upper bound: " << m_origin.x() + m_size << " " << m_origin.y() + m_size << " " << m_origin.z() + m_size << std::endl;
    }
    if (y_in_box != PointRespectToBox::INSIDE) {
        std::cout << "y outside the box" << std::endl;
        std::cout << "n_y: " << n_y << std::endl;
        std::cout << "y_in_box: " << y_in_box << std::endl;
        std::cout << "Point location: " << pt.x() << " " << pt.y() << " " << pt.z() << std::endl;
        std::cout << "Lower bound: " << m_origin.x() << " " << m_origin.y() << " " << m_origin.z() << std::endl;
        std::cout << "Upper bound: " << m_origin.x() + m_size << " " << m_origin.y() + m_size << " " << m_origin.z() + m_size << std::endl;
    }
    if (z_in_box != PointRespectToBox::INSIDE) {
        std::cout << "z outside the box" << std::endl;
        std::cout << "n_z: " << n_z << std::endl;
        std::cout << "z_in_box: " << z_in_box << std::endl;
        std::cout << "Point location: " << pt.x() << " " << pt.y() << " " << pt.z() << std::endl;
        std::cout << "Lower bound: " << m_origin.x() << " " << m_origin.y() << " " << m_origin.z() << std::endl;
        std::cout << "Upper bound: " << m_origin.x() + m_size << " " << m_origin.y() + m_size << " " << m_origin.z() + m_size << std::endl;
    }
#endif

        unsigned int n_max = (n_x > n_y) ? ((n_x > n_z) ? n_x : n_z) : ((n_y > n_z) ? n_y : n_z);
        unsigned int x_insert_index = (x_in_box == PointRespectToBox::LEFT) ? 1 : 0;
        unsigned int y_insert_index = (y_in_box == PointRespectToBox::LEFT) ? 1 : 0;
        unsigned int z_insert_index = (z_in_box == PointRespectToBox::LEFT) ? 1 : 0;
        std::cout << "\033[35mExpanding octree\033[0m" << std::endl;
        std::cout << "\033[35mType: " << (std::is_same<T, Vertex>::value ? "Vertex" : 
                                 std::is_same<T, BallCenter>::value ? "BallCenter" : 
                                 "Unknown") << "\033[0m" << std::endl;
        // At this stage, n_max will be minimum 1.
        // this for loop can not be parallelized
        for (unsigned int i = 0; i < n_max; ++i)
        {
            // First update the locational code for all nodes
            unsigned int top_level = m_root->getDepth();

            // Update all node
            expandAllNodeLoc(top_level, x_insert_index, y_insert_index, z_insert_index);

            // Then add parent
            double x = (x_in_box == PointRespectToBox::LEFT) ? m_origin.x() - m_size : m_origin.x();
            double y = (y_in_box == PointRespectToBox::LEFT) ? m_origin.y() - m_size : m_origin.y();
            double z = (z_in_box == PointRespectToBox::LEFT) ? m_origin.z() - m_size : m_origin.z();

            // Create new root node
            Point new_origin(x, y, z);
            TOctreeNode<T> *new_root_node = new TOctreeNode<T>(new_origin, double(2 * m_size), (unsigned int)(m_root_depth + 1));

            unsigned int childIndex = (x_insert_index << 2) + (y_insert_index << 1) + z_insert_index;
            m_root->setNchild(childIndex);
            m_root->setParent(new_root_node);

            // add the old root node as a child of the new root node
            new_root_node->setChild(childIndex, m_root);

            // re-asign new root node
            m_root = new_root_node;

            // Update the Octree
            m_size *= 2;
            m_nb_interval = pow2(m_root_depth + 1);
            m_origin = new_origin;
            m_root_depth = m_root->getDepth();
        }
    }
}

template <class T>
T *TOctree<T>::checkSizeAndaddPoint(const T &pt)
{
    checkSizeAndexpand(pt);
    T *new_pt = addPoint(pt);
    return new_pt;
}

static unsigned int leftOrRightInLevel(unsigned int code, unsigned int level)
{
    unsigned int childBranchBit = 1 << level;
    return (code & childBranchBit)>>level;
}

template <class T>
T *TOctree<T>::addPoint(const T &pt)
{
    // unsigned int is important here, as we will use bitwise operations. The size of unsigned int is 4 bytes.
    // This means the depth of the octree should be less than 32.
    unsigned int codx = (unsigned int)((pt.x() - m_origin.x()) / m_size * m_nb_interval);
    unsigned int cody = (unsigned int)((pt.y() - m_origin.y()) / m_size * m_nb_interval);
    unsigned int codz = (unsigned int)((pt.z() - m_origin.z()) / m_size * m_nb_interval);

    // At this stage, there are no nodes in the octree. We need to create a root node.
    TOctreeNode<T> *node = getRoot();
    unsigned int l = node->getOrdinalPositionForChild();



    // traverse the octree until we reach a leaf
    while (node->getDepth() != 0) // The node here will get redefined at each iteration
    {
        // check if the point is inside the volume
        if (! node->isInside(pt))
        {
            std::cerr << "Point is not in the volume" << std::endl;
            std::cerr << "Point: " << pt.x() << ", " << pt.y() << ", " << pt.z() << std::endl;
            std::cerr << "Volume: " << m_origin.x() << ", " << m_origin.y() << ", " << m_origin.z() << " to " << m_origin.x() + m_size << ", " << m_origin.y() + m_size << ", " << m_origin.z() + m_size << std::endl;
            std::exit(EXIT_FAILURE);
        }
        //unsigned int childBranchBit = 1 << l;            // For example, if l=2, childBranchBit=100
        //unsigned int x = ((codx & childBranchBit) >> l); // return 1 if codx has a 1 at the l-th position, 0 otherwise.
        //unsigned int y = ((cody & childBranchBit) >> l);
        //unsigned int z = ((codz & childBranchBit) >> l);
        unsigned int x = leftOrRightInLevel(codx, l); // return 1 if codx has a 1 at the l-th position, 0 otherwise.
        unsigned int y = leftOrRightInLevel(cody, l);
        unsigned int z = leftOrRightInLevel(codz, l);
        unsigned int childIndex = (x << 2) + (y << 1) + z;

        if (node->getChild(childIndex) == nullptr)
        {
            double childSize = node->getSize() / 2.0;
            unsigned int childDepth = node->getDepth() - 1; // childDepth is smaller and amaller over the while loop.
            Point origin = node->getOrigin();
            Point childOrigin = Point(origin.x() + x * childSize,
                                      origin.y() + y * childSize,
                                      origin.z() + z * childSize);

            TOctreeNode<T> *child = node->initializeChild(childIndex,
                                                          childOrigin);

            // if node is root, the getXLoc() will return 0
            child->setXLoc(node->getXLoc() + (x << (childDepth)));
            child->setYLoc(node->getYLoc() + (y << (childDepth)));
            child->setZLoc(node->getZLoc() + (z << (childDepth)));
            // m_nb_non_empty_cells[childDepth] += 1;
            // std::cout << "Adding child node at depth " << childDepth << std::endl;
        }
        node = node->getChild(childIndex);
        l--;
    }

    T *pt_ptr = node->addPoint(pt);
    return pt_ptr;
}

template <class T>
void TOctree<T>::getNodes(unsigned int depth, TOctreeNode<T> *starting_node,
                          std::vector<TOctreeNode<T> *> &nodes)
{
    if (starting_node->getDepth() == depth)
        nodes.push_back(starting_node);
    else
    {
        for (int i = 0; i < 8; ++i)
            if (starting_node->getChild(i) != nullptr)
                getNodes(depth, starting_node->getChild(i), nodes);
    }
}

template <class T>
void TOctree<T>::getNodes(unsigned int depth,
                          TOctreeNode<T> *starting_node,
                          std::vector<std::vector<TOctreeNode<T> *>> &node_collection)
{
    std::vector<TOctreeNode<T> *> nodes;
    getNodes(depth, starting_node, nodes);

    for (int i = 0; i < 8; ++i)
    {
        std::vector<TOctreeNode<T> *> ov;
        node_collection.push_back(ov);
    }

    while (!nodes.empty())
    {
        TOctreeNode<T> *node = nodes.back();
        nodes.pop_back();
        unsigned int nchild = node->getNChild();
        node_collection[nchild].push_back(node);
    }
}

template <class T>
unsigned int TOctree<T>::getMaxPoints() const
{
    return m_max_points;
}

#ifdef _DEBUG
template <class T>
void TOctree<T>::debugPrintStats()
{
    std::cout << "OctreeVertices statistics" << std::endl;
    std::cout << "OctreeVertices with depth " << this->getDepth() << " created." << std::endl;
    std::cout << "OctreeVertices contains " << this->getNpoints()
              << " points. The bounding box size is "
              << this->getSize() << std::endl;
    std::cout << "OctreeVertices statistics" << std::endl;

    double size = m_size / 2;
    for (int i = m_root_depth - 1; i >= 0; i--)
    {
        //std::cout << "level " << i << " ; The size length " << size
        //          << " ; mean number of points: "
        //          //<<(double)m_npoints / ((double)m_nb_non_empty_cells[i])
        //          << "currently unavailable"
        //          << std::endl;
        //size = size / 2.0;
    }
    TOctreeNode<T> *root = getRoot();
    // Print the size of the root node
    std::cout << "Size of the root node: " << root->getSize() << std::endl;
    std::cout << "Depth of the root node: " << root->getDepth() << std::endl;
    // Find and print size of a leaf node by traversing down
    TOctreeNode<T> *current = root;
    std::cout << "Root depth: " << root->getDepth() << std::endl;
    while (!current->isLeaf())
    {
        // Move to first non-null child
        for (int i = 0; i < 8; i++)
        {
            if (current->getChild(i) != nullptr)
            {
                current = current->getChild(i);
                std::cout << "Current node depth: " << current->getDepth() << std::endl;
                break;
            }
        }
    }
    std::cout << "Size of a leaf node: " << current->getSize() << std::endl;
    std::cout << "Depth of a leaf node: " << current->getDepth() << std::endl;
}

template <class T>
template <typename U>
TOctree<U> TOctree<T>::debugCopySkeleton() const
{
    TOctree<U> new_octree;
    new_octree.setDepth(m_root_depth);
    Point origin = m_origin;
    new_octree.initialize(origin, m_size);
    return new_octree;
}

template <class T>
TOctree<T>::TOctree(const TOctree<T> &other)
{
    std::cout << "Copy constructor called" << std::endl;
    std::cout << "Copy constructor called" << std::endl;
    std::cout << "Copy constructor called" << std::endl;
    std::cout << "Copy constructor called" << std::endl;
    std::cout << "Copy constructor called" << std::endl;
    std::cout << "Copy constructor called" << std::endl;
    this->setDepth(other.getDepth());
    Point origin = other.getOrigin();
    this->initialize(origin, other.getSize());
}
#endif

template <class T>
void TOctree<T>::integrityCheck()
{
    TOctreeNode<T> *root = getRoot(); 
    checkIntegrity(root);
}

template <class T>
static void checkIntegrity(TOctreeNode<T> *node)
{
   for (int i = 0; i < 8; i++)
   {
        Point start_point = node->getOrigin();
        double size = node->getSize();

       if (node->getChild(i) != nullptr)
       {
            bool x_code = i & (1 << 2);
            bool y_code = i & (1 << 1);
            bool z_code = i & (1 << 0);
            
            Point child_origin = Point(start_point.x() + x_code * size / 2,
                                      start_point.y() + y_code * size / 2,
                                      start_point.z() + z_code * size / 2);   
            checkIntegrity(node->getChild(i), child_origin, size/2, x_code, y_code, z_code);
            // current we do not check the xloc yloc zloc.
            checkIntegrity(node->getChild(i));
       }
   }
};

template <class T>
static void checkIntegrity(TOctreeNode<T> *childnode, Point suspect_origin, double suspect_size, bool x_code, bool y_code, bool z_code)
{
    Point child_origin = childnode->getOrigin();
    double child_size = childnode->getSize();

    if (child_size != suspect_size)
    {
        std::cerr << "Child size: " << child_size << std::endl;
        std::cerr << "Supspec size: " << suspect_size << std::endl;
        std::cerr << "Integrity check failed" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    const double epsilon = 1e-10;
    if (std::abs(child_origin.x() - suspect_origin.x() ) > epsilon ||
        std::abs(child_origin.y() - suspect_origin.y() ) > epsilon ||
        std::abs(child_origin.z() - suspect_origin.z() ) > epsilon)
    {
        std::cerr << "Integrity check failed" << std::endl;
        std::cerr << "Child origin: " << child_origin.x() << ", " << child_origin.y() << ", " << child_origin.z() << std::endl;
        std::cerr << "Supspec origin: " << suspect_origin.x() << ", " << suspect_origin.y() << ", " << suspect_origin.z() << std::endl;
        std::cerr << "Child size: " << child_size << std::endl;
        std::cerr << "Supspec size: " << suspect_size << std::endl;
        std::cerr << "X code: " << x_code << std::endl;
        std::cerr << "Y code: " << y_code << std::endl;
        std::cerr << "Z code: " << z_code << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
     unsigned int child_nchild = childnode->getNChild();
     unsigned int suspect_nchild = (x_code<<2) + (y_code<<1) + z_code;

     if (child_nchild != suspect_nchild)
     {
        std::cerr << "Child nchild: " << child_nchild << std::endl;
        std::cerr << "Supspec nchild: " << suspect_nchild << std::endl;
        std::cerr << "Integrity check failed" << std::endl;
        std::exit(EXIT_FAILURE);
     }
    if (childnode->isLeaf())
    {
        childnode->checkAllPointsInVolume();    
    }
    // current we do not check the xloc yloc zloc.
}

#endif
