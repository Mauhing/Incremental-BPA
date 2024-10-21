#ifndef VERTEX_H
#define VERTEX_H

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <set>

#include "Point.h"
#include "types.h"
//#include "OctreeNode.h"

class Edge;
class Facet;
//class OctreeNodeV;
template<typename T> class TOctreeNode; // Forward declaration

using namespace std;


/**
 * @class Vertex
 * @brief Input samples to be triangulated
 * 
 * Sample point inserted as vertex in the program:
 * to begin with, it is an orphan vertex which will be aggreggated
 * during the triangulation contains topology information
 */
class BallCenter : public Point
{
    public:
    friend ostream& operator << (ostream& out, const BallCenter& v);
  
    private : //properties
    
        /** @brief nx, ny, nz normal coordinates*/
        double m_nx, m_ny, m_nz;
        
        Facet* m_facet;
        

    public : //constructor+destructor
    
        /** @brief default constructor*/
        BallCenter() = delete;
        
        /** @brief constructor from coordinates and normal*/
        BallCenter(double x, double y, double z, Facet* facet);
          
        /** @brief default destrictor*/
        ~BallCenter();
    
    public : //accessors + modifiers
   
};
#endif
