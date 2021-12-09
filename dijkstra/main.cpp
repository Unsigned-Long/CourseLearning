/**
 * @file main.cpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "handler.h"

int main(int argc, char const *argv[])
{
    std::vector<ns_dj::PathPair> pls{
        {1, 2, 5},
        {2, 3, 4},
        {3, 4, 6},
        {5, 6, 3},
        {6, 7, 4},
        {7, 8, 2},
        {9, 10, 2},
        {10, 11, 3},
        {11, 12, 5},
        {13, 14, 3},
        {14, 15, 4},
        {15, 16, 3},
        {1, 5, 4},
        {5, 9, 1},
        {9, 13, 2},
        {2, 6, 3},
        {6, 10, 3},
        {10, 14, 3},
        {3, 7, 2},
        {7, 11, 2},
        {11, 15, 1},
        {4, 8, 1},
        {8, 12, 1},
        {12, 16, 3}};

    auto sz = pls.size();
    for (int i = 0; i != sz; ++i)
    {
        auto pp = pls.at(i);
        /**
         * @brief the operator is to make the path section double pass
         * @attention exchange the enode and the snode
         */
        pls.push_back({pp._enode, pp._snode, pp._cost});
    }
    ns_dj::Dijkstra dj(pls);
    // find path
    int sn = 16, en = 9;
    auto path = dj.findPath(sn, en);
    // output
    std::cout << "from " << sn << " to " << en << std::endl;
    for (const auto &elem : path.first)
        std::cout << elem << ' ';
    std::cout << "\ncost : " << path.second << std::endl;
    return 0;
}
