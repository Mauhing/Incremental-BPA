#ifndef REGION_H
#define REGION_H

#include "Facet.h"

/**
 * @brief Boundary class
 * @details This class represents a boundary of a region.
 */
class Boundary
{
    private:
        /**
         * @brief set of edges
         */
        std::set<Edge*> m_edges;

        /**
         * @brief flag to check if the boundary is ordered
         */
        bool m_is_ordered;

        /**
         * @brief square length of the boundary
         */
        double m_sq_length;

    public:
        /**
         * @brief default constructor
         */
        Boundary();

        /**
         * @brief destructor
         */
        ~Boundary();

        /**
         * @brief copy constructor
         * @param other boundary to copy
         */
        Boundary(const Boundary& other);

        /**
         * @brief assignment operator
         * @param other boundary to assign
         * @return reference to the assigned boundary
         */
        Boundary& operator=(const Boundary& other);

        /**
         * @brief add an edge to the boundary
         * @param edge edge to add
         */
        void addEdge(Edge* edge);

        /**
         * @brief set the ordered flag
         * @param is_ordered flag to set
         */
        void setOrdered(bool is_ordered);

        /**
         * @brief get the edges of the boundary
         * @return reference to the set of edges
         */
        const std::set<Edge*>& getEdges() const;

        /**
         * @brief set the square length of the boundary
         */
        void setSqLength();

        /**
         * @brief get the square length of the boundary
         * @return square length of the boundary
         */
        double getSqLength() const;
};

/**
 * @brief Region class
 * @details This class represents a region.
 */
class Region
{
    private:
        /**
         * @brief set of facets
         */
        std::unordered_set<Facet*> m_land;

        /**
         * @brief coast line boundary
         */
        Boundary m_coast;
        
        /**
         * @brief list of lake boundaries
         */
        std::list<Boundary> m_lakes;

        /**
         * @brief square length of the region
         */
        double m_sq_length;

        /**
         * @brief flag to check if the region is a mainland
         */
        bool m_is_mainland;

    public:
        /**
         * @brief default constructor
         */
        Region() = default;

        /**
         * @brief constructor
        */
        Region(const std::unordered_set<Facet*>& land, const Boundary& coast, const std::list<Boundary>& lakes);

        /**
         * @brief copy constructor
         * @param other region to copy
         */
        Region(const Region& other);

        /**
         * @brief assignment operator
         * @param other region to assign
         * @return reference to the assigned region
         */
        Region& operator=(const Region& other);

        /**
         * @brief destructor
         */
        ~Region(); 
        
        /**
         * @brief get the square length of the region
        */
        double getSqLength() const;

        /**
         * @brief get the number of facets in the region
         * @return number of facets in the region
         */
        unsigned int getNumFacets() const;

        /**
         * @brief get the facets in the region
         */
        std::unordered_set<Facet*> getFacets() const;
};
#endif
