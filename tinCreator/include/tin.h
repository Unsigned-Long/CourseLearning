#pragma once

/**
 * @file tin.h
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2021-12-17
 * 
 * @copyright Copyright (c) 2021
 */

#include "conhull.h"
#include "triangle.hpp"
#include <list>
#include <unordered_set>

namespace ns_tin
{
    class TinCreator
    {
    public:
        /**
         * @brief thr Voronoi in the tins[2d]
         * 
         */
        struct Voronoi
        {
        private:
            // the polygon
            ns_geo::Polygonf _vor;
            // the center's id of the polygon
            uint _cenid;

        public:
            Voronoi(const ns_geo::Polygonf &vor, uint cenid)
                : _vor(vor), _cenid(cenid) {}

            const ns_geo::Polygonf &voronoi() const { return this->_vor; }

            uint cenid() const { return this->_cenid; }
        };

    private:
        ns_geo::RefPointSet2f _rps;

        std::list<ns_geo::RefTriangle2f> _rtri;

    public:
        TinCreator() = delete;

        TinCreator(const ns_geo::PointSet2f &ps)
        {
            for (uint i = 0; i != ps.size(); ++i)
                this->_rps.insert({i, ps.at(i)});
            this->initTins(ps);
        }

        const ns_geo::RefPointSet2f &refPointSet() const { return this->_rps; }

        /**
         * @brief get the reference triangles in the tinCreator
         * 
         * @return const std::list<ns_geo::RefTriangle2f>& 
         */
        const std::list<ns_geo::RefTriangle2f> &tins() const { return this->_rtri; }

        /**
         * @brief insert a point to the tin
         * 
         * @param p the point
         */
        inline void insert(const ns_geo::Point2f &p)
        {
            auto id = static_cast<uint>(this->_rps.size());
            this->_rps.insert({id, p});
            this->insert({id, p});
        }

        /**
         * @brief get the voronois of the tin
         * 
         * @return std::list<Voronoi> 
         */
        std::list<Voronoi> voronois() const;

    protected:
        /**
         * @brief build the init tins
         * 
         * @param ps 
         */
        void initTins(const ns_geo::PointSet2f &ps);

        /**
         * @brief insert point to the tin
         * 
         * @param p 
         */
        void insert(const ns_geo::RefPoint2f &p);
    };
} // namespace tin
