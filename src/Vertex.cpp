/**
 * @file Vertex.cpp
 * @brief implementation of the vertex methods declared in Vertex.h
 * @author Julie Digne
 * @date 2012-10-08
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

#include "Vertex.h"
#include "Edge.h"
#include "Facet.h"
#include "utilities.h"
#include <cstdio>
#include <iostream>

Vertex::Vertex() : Point()
{
    m_nx=m_ny=m_nz=0.0;
    m_index = -1;
    //m_type = 0;
    m_type = Vertex::ORPHAN; //0
    m_octreeNodeLeaf = NULL;
}

Vertex::Vertex(double x, double y, double z, double nx, double ny, double nz)
                : Point(x,y,z)
{
    m_nx = nx;
    m_ny = ny;
    m_nz = nz;
    m_index = -1;
    //m_type = 0;
    m_type = Vertex::ORPHAN; //0
    m_octreeNodeLeaf = NULL;
}

Vertex::~Vertex()
{
    m_nx=m_ny=m_nz=0.0;
    m_index = -1;
    m_adjacentEdges.clear();
    m_adjacentFacets.clear();
    //m_type = 0;
    m_type = Vertex::ORPHAN; //0
    m_octreeNodeLeaf = NULL;
}

bool Vertex::addAdjacentEdge(Edge* edge)
{
    pair<set<Edge*>::iterator,bool> insertion_result;
    insertion_result = m_adjacentEdges.insert(edge);
    return insertion_result.second;
}

void Vertex::removeAdjacentEdge(Edge* edge)
{
    m_adjacentEdges.erase(edge);
}

bool Vertex::addAdjacentFacet(Facet* facet)
{
    pair<set<Facet*>::iterator,bool> insertion_result;
    insertion_result = m_adjacentFacets.insert(facet);
    return insertion_result.second;
}

void Vertex::removeAdjacentFacet(Facet* facet)
{
    m_adjacentFacets.erase(facet);
}

int Vertex::index()
{
    return m_index;
}

void Vertex::setIndex(int index)
{
    m_index = index;
}

Edge_set& Vertex::adjacentEdges()
{
    return m_adjacentEdges;
}


Edge* Vertex::getLinkingEdge(Vertex* vertex)
{
    return getCommonElement(m_adjacentEdges, vertex->adjacentEdges());
}

double Vertex::nx() const
{
    return m_nx;
}

double Vertex::ny() const
{
    return m_ny;
}

double Vertex::nz() const
{
    return m_nz;
}

bool Vertex::isCompatibleWith(const Edge& e) const
{
    double ntx,nty,ntz;
    const Vertex &src = *(e.getSource());
    const Vertex &tgt = *(e.getTarget());


    cross_product(x() - src.x(), y() - src.y(), z() - src.z(),
                  tgt.x() - src.x(), tgt.y() - src.y(),
                  tgt.z() - src.z(), ntx, nty, ntz);

    normalize(ntx,nty,ntz);

    if( (ntx * nx() + nty * ny() + ntz * nz() > -1e-16)
        &&(ntx * src.nx() + nty * src.ny() + ntz * src.nz() > -1e-16)
        &&(ntx * tgt.nx() + nty * tgt.ny() + ntz * tgt.nz() > -1e-16))
        return true;

    return false;
}

bool Vertex::isCompatibleWith(const Vertex& v1, const Vertex &v2) const
{
    
    //By defualt, we assume that the v0, v1, v2 form a postive vector compare to all normals.

    double ntx,nty,ntz;

    //cross_product(x() - v1.x(), y() - v1.y(), z() - v1.z(),
    //              v2.x() - v1.x(), v2.y() - v1.y(), v2.z() - v1.z(),
    //              ntx, nty, ntz);
    cross_product(v1.x() - x(),    v1.y() - y(),    v1.z() - z(),
                  v2.x() - v1.x(), v2.y() - v1.y(), v2.z() - v1.z(),
                  ntx, nty, ntz);
    normalize(ntx,nty,ntz);

    // Flip the normal such that it aligns with *this vertex.
    if( ntx * nx() + nty * ny() + ntz * nz() < -1e-16)
    {
        ntx = - ntx;
        nty = - nty;
        ntz = - ntz;
        std::cout << "Flipped the normal." << std::endl;
    }
    // ntx, nty, ntz is now the normal of the plane defined by *this, v1 and v2.

    if((ntx * v1.nx() + nty * v1.ny() + ntz * v1.nz() > -1e-16)
        &&(ntx * v2.nx() + nty * v2.ny() + ntz * v2.nz() > -1e-16))
        return true;

    return false;
}

//added by mauhing
bool Vertex::isCompatibleWithAndHandnessCheck(const Vertex& v1, const Vertex &v2, bool &changeHandness) const
{
    changeHandness = false;
    //With changeHandness as false, we assume that the v0 go to v1, v1 go to v2, v2 go to v0.

    double ntx,nty,ntz;

    //cross_product(x() - v1.x(), y() - v1.y(), z() - v1.z(),
    //              v2.x() - v1.x(), v2.y() - v1.y(), v2.z() - v1.z(),
    //              ntx, nty, ntz);
    cross_product(v1.x() - x(),    v1.y() - y(),    v1.z() - z(),
                  v2.x() - v1.x(), v2.y() - v1.y(), v2.z() - v1.z(),
                  ntx, nty, ntz);
    normalize(ntx,nty,ntz);

    // Flip the normal such that it aligns with *this vertex.
    if( ntx * nx() + nty * ny() + ntz * nz() < -1e-16)
    {
        ntx = - ntx;
        nty = - nty;
        ntz = - ntz;
        changeHandness = true;
        //With changeHandness as true, v0 go to v2, v2 go to v1, v1 go to v0.
    }
    // ntx, nty, ntz is now the normal of the plane defined by *this, v1 and v2.

    if((ntx * v1.nx() + nty * v1.ny() + ntz * v1.nz() > -1e-16)
        &&(ntx * v2.nx() + nty * v2.ny() + ntz * v2.nz() > -1e-16))
        return true;

    return false;
}

bool Vertex::isAdjacent(Facet* facet)
{
    if(m_adjacentFacets.find(facet) != m_adjacentFacets.end())
        return true;
    return false;
}

Vertex::VertexType Vertex::getType() const
{
    return m_type;
}

void Vertex::setType(Vertex::VertexType type)
{
    m_type = type;
}



void Vertex::updateType()
{
    if(m_adjacentEdges.empty())
    {
        //m_type = 0;
        m_type = Vertex::ORPHAN; //0
        return;
    }
    Edge_set::const_iterator ei;
    for(ei = m_adjacentEdges.begin(); ei != m_adjacentEdges.end(); ++ei)
    {
        const Edge *e = *ei;
        if(e->getType() != Edge::INNER) //2
        {
            //m_type = 1;
            m_type = Vertex::FRONT; //1
            return;
        }
    }
    //m_type = 2;
    m_type = Vertex::INNER; //2
}


ostream& operator << (ostream& out, const Vertex& v)
{
    out << v.x() << "\t" << v.y() << "\t" << v.z()
        << "\t" << v.nx() << "\t" << v.ny() << "\t" << v.nz();
    return out;
}


//"this" is the source and test is the target
Vertex* Vertex::findBorder(Vertex* test)
{
    Edge *e0 = getLinkingEdge(test);
    Facet *facet = e0->getFacet1();

    Edge_set::iterator ei = m_adjacentEdges.begin();
    Vertex *candidate = NULL;
    while( ei != m_adjacentEdges.end())
    {
        //if((*ei)->getType() != 0)
        if((*ei)->getType() != Edge::BORDER)
        {
            ++ei;
            continue;
        }
        Vertex *v = (*ei)->getSource();

        if(v==this)
        {
            ++ei;
            continue;
        }

        if(facet->hasVertex(v))
        {
            ++ei;
            continue;
        }

        Edge *e = v->getLinkingEdge(test);
        if(e==NULL)
        {
            ++ei;
            continue;
        }

        //if((e->getType()!=0)||(e->getSource()!=test))
        if((e->getType()!=Edge::BORDER)||(e->getSource()!=test))
        {
            ++ei;
            continue;
        }
        return(v);
    }
    return candidate;
}


void Vertex::setOctreeNodeLeaf(TOctreeNode<Vertex>* node)
{
    m_octreeNodeLeaf = node;
}

const Facet_set& Vertex::adjacentFacets() const
{
    return m_adjacentFacets;
}