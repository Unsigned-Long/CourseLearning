/**
 * @file main.cpp
 * @author csl 3079625093@qq.com
 * @version 0.1
 * @date 2022-01-06
 * 
 * @copyright Copyright (c) 2022
 * 
 * @attention if you want to run this program, just:
 *            [1] cd netsolver/
 *            [2] mkdir build
 *            [3] cmake ..
 *            [4] make
 *            [5] ./netSolver
 */

#include "handler.h"

int main(int argc, char const *argv[])
{
    try
    {
        // read the arcs data
        auto arcs = ns_net::Handler::readData("../data/data.csv");
        // construct the base nodes
        auto baseNodes = std::vector<ns_net::Node>{ns_net::Node(2, 209.057), ns_net::Node(20, 212.706)};

        // solve the net
        ns_net::Handler handler(arcs, baseNodes);
        handler.solve(true);
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
