/**
 * @file handler.cpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2021-12-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "conhull.h"
#include "stack"
#include <set>

namespace ns_ch
{

    std::vector<std::size_t> ConvecHull::genIndex(const ns_geo::PointSet2f &ps)
    {
        // find the max-min [x|y] points
        std::size_t left = 0, right = 0, top = 0, below = 0;
        for (int i = 0; i != ps.size(); ++i)
        {
            if (ps.at(i).x() < ps.at(left).x())
                left = i;
            if (ps.at(i).x() > ps.at(right).x())
                right = i;
            if (ps.at(i).y() < ps.at(below).y())
                below = i;
            if (ps.at(i).y() > ps.at(top).y())
                top = i;
        }

        std::set<std::size_t> record;
        record.insert(left);
        record.insert(top);
        record.insert(right);
        record.insert(below);

        std::stack<std::size_t> s;
        s.push(left);
        s.push(top);
        s.push(right);
        s.push(below);
        s.push(left);

        std::vector<std::size_t> ch;
        while (s.size() != 1)
        {
            auto fromID = s.top();
            auto from = ps.at(s.top());
            s.pop();
            auto toID = s.top();
            auto to = ps.at(s.top());
            if (fromID == toID)
                continue;
            std::vector<std::pair<std::size_t, float>> probPoints;
            for (int i = 0; i != ps.size(); ++i)
            {
                if (record.find(i) != record.cend() || !ns_geo::RHandRule::palright(ps.at(i), {from, to}))
                    continue;
                probPoints.push_back(std::make_pair(i, ns_geo::distance(ps.at(i), {from, to})));
            }
            if (!probPoints.empty())
            {
                std::sort(probPoints.begin(), probPoints.end(), [](const auto &p1, const auto &p2)
                          { return p1.second < p2.second; });
                s.push(probPoints.back().first);
                s.push(fromID);
                record.insert(probPoints.back().first);
            }
            else
                ch.push_back(fromID);
        }
        return ch;
    }

    ns_geo::Polygonf ConvecHull::genPolygon(const ns_geo::PointSet2f &ps)
    {
        auto index = ConvecHull::genIndex(ps);
        ns_geo::Polygonf polygon(index.size());
        for (int i = 0; i != index.size(); ++i)
            polygon.at(i) = ps.at(index.at(i));
        return polygon;
    }
} // namespace ns_ch
