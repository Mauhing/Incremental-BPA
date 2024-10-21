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