#pragma once

/**
 * @file handler.h
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <set>

namespace ns_dj
{
    /**
     * @brief the path pair
     */
    struct PathPair
    {
    public:
        // the start node
        int _snode;
        // the end node
        int _enode;
        // the cost from start node to end node
        float _cost;
    };

    /**
     * @brief calculate shortest path with the dijkstra algorithm
     */
    class Dijkstra
    {
    public:
        using path_type = std::vector<int>;

    private:
        std::vector<PathPair> _pls;

    public:
        Dijkstra() = delete;

        /**
         * @brief Construct a new Dijkstra object
         * 
         * @param pls the path pair list that records the distance weight
         */
        Dijkstra(const std::vector<PathPair> &pls) : _pls(pls) {}

    public:
        /**
         * @brief find the shortest path from the start node to the end node
         * 
         * @param startNode the start node of the path
         * @param endNode the end node of the path
         * @return std::pair<path_type, float> 
         */
        std::pair<path_type, float> findPath(int startNode, int endNode) const;
    };
} // namespace ns_dj
