#ifndef REGION_H
#define REGION_H

#include "Facet.h"

class Boundary
{
    private:
        std::set<Edge*> m_edges;
        bool m_is_ordered;
        double m_sq_length;
    public:
        Boundary();
        ~Boundary();

    Boundary(const Boundary& other);

    Boundary& operator=(const Boundary& other);

    void addEdge(Edge* edge);

    void setOrdered(bool is_ordered);

    const std::set<Edge*>& getEdges() const;

    void setSqLength();

    double getSqLength() const;
};

class Region
{
    private:
        std::unordered_set<Facet*> land;
        Boundary coast;
        double sq_length;
        std::list<Boundary> lakes;

        bool is_mainland;
    public:
        Region() = default;
        Region(const std::unordered_set<Facet*>& land, const Boundary& coast, const std::list<Boundary>& lakes);
        Region(const Region& other);
        Region& operator=(const Region& other);
        ~Region(); 
        
        double getSqLength() const;
        unsigned int getNumFacets() const;
        std::unordered_set<Facet*> getFacets() const;
};
#endif
