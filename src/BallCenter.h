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

    BallCenter(const Point &point, Facet *facet);

    /** @brief default destrictor*/
    ~BallCenter();

    // copy constructor
    BallCenter(const BallCenter &other);

public: // accessors + modifiers
    void setOctreeNodeLeaf(TOctreeNode<BallCenter> *node);

    Facet *getFacet() const;
};
#endif