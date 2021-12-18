/**
 * @file tin.cpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2021-12-17
 * 
 * @copyright Copyright (c) 2021
 */

#include "tin.h"

namespace ns_tin
{
    void TinCreator::initTins(const ns_geo::PointSet2f &ps)
    {
        auto tempConvex = ns_ch::ConvecHull::genIndex(ps);

        std::list<std::size_t> convex(tempConvex.cbegin(), tempConvex.cend());

        auto iter1 = convex.cbegin();
        auto iter2 = ++convex.cbegin();
        auto iter3 = ++(++convex.cbegin());

        while (convex.size() != 3)
        {
            auto cirr = ns_geo::Triangle2f(ps.at(*iter1), ps.at(*iter2), ps.at(*iter3)).circumCircle();
            auto center = cirr.first;
            float radius = cirr.second;
            bool isDelaunay = true;
            for (const auto &pid : convex)
            {
                if (pid == *iter1 || pid == *iter2 || pid == *iter3)
                    continue;
                if (ns_geo::distance(ps.at(pid), center) < radius)
                {
                    isDelaunay = false;
                    break;
                }
            }

            if (isDelaunay)
            {
                this->_rtri.push_back(this->_rps.createRefTriangle2(*iter1, *iter2, *iter3));
                iter2 = convex.erase(iter2);
                (iter2 == convex.cend()) ? iter2 = convex.cbegin() : iter2;
                iter3 = iter2, ++iter3;
                (iter3 == convex.cend()) ? iter3 = convex.cbegin() : iter3;
                (iter2 == convex.cbegin()) ? (iter1 = --convex.cend()) : (iter1 = iter2, --iter1);
            }
            else
            {
                ++iter1, ++iter2, ++iter3;
                (iter1 == convex.cend()) ? iter1 = convex.cbegin() : iter1;
                (iter2 == convex.cend()) ? iter2 = convex.cbegin() : iter2;
                (iter3 == convex.cend()) ? iter3 = convex.cbegin() : iter3;
            }
        }

        this->_rtri.push_back(this->_rps.createRefTriangle2(*iter1, *iter2, *iter3));

        std::unordered_set<uint> usedNodes(tempConvex.cbegin(), tempConvex.cend());

        for (uint i = 0; i != ps.size(); ++i)
            if (usedNodes.find(i) == usedNodes.cend())
                this->insert({i, ps.at(i)});

        return;
    }

    void TinCreator::insert(const ns_geo::RefPoint2f &p)
    {
        // find triangles affected by this point and delete them
        std::unordered_set<uint> affectedNodes;
        for (auto iter = this->_rtri.cbegin(); iter != this->_rtri.cend();)
        {
            auto cirr = iter->circumCircle();
            auto center = cirr.first;
            auto radius = cirr.second;
            if (ns_geo::distance(p, center) < radius)
            {
                affectedNodes.insert(iter->p1().id());
                affectedNodes.insert(iter->p2().id());
                affectedNodes.insert(iter->p3().id());
                iter = this->_rtri.erase(iter);
            }
            else
                ++iter;
        }
        // create new triangles
        std::vector<std::pair<uint, float>> sortAzi;
        for (const auto &id : affectedNodes)
            sortAzi.push_back(std::pair(id, ns_geo::RHandRule::azimuth(p, this->_rps.at(id))));
        std::sort(sortAzi.begin(), sortAzi.end(), [](const auto &p1, const auto &p2)
                  { return p1.second < p2.second; });
        for (auto iter = sortAzi.cbegin(); iter != --sortAzi.cend();)
        {
            auto id1 = p.id();
            auto id2 = iter->first;
            auto id3 = (++iter)->first;
            this->_rtri.push_back(this->_rps.createRefTriangle2(id1, id2, id3));
        }
        this->_rtri.push_back(this->_rps.createRefTriangle2(p.id(), sortAzi.back().first, sortAzi.front().first));
        return;
    }

    std::list<TinCreator::Voronoi> TinCreator::voronois() const
    {
        std::list<TinCreator::Voronoi> voronois;
        for (const auto &[id, rp] : this->_rps)
        {
            std::vector<std::pair<ns_geo::Point2f, float>> cenAzi;
            for (const auto &tri : this->_rtri)
                // find the reftriangles that is related to this point
                if (tri.p1().id() == id || tri.p2().id() == id || tri.p3().id() == id)
                {
                    auto cen = tri.circumCircle().first;
                    cenAzi.push_back(std::make_pair(cen, ns_geo::RHandRule::azimuth(rp, cen)));
                }
            // sort the centers by the azimuth from the point to the center
            std::sort(cenAzi.begin(), cenAzi.end(), [](const auto &p1, const auto &p2)
                      { return p1.second < p2.second; });
            // construct the voronois
            ns_geo::Polygonf vor;
            for (const auto &[cen, azi] : cenAzi)
                vor.push_back(cen);
            voronois.push_back(Voronoi(vor, id));
        }
        return voronois;
    }
} // namespace tin