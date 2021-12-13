/**
 * @file main.cpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2021-12-13
 * 
 * @copyright Copyright (c) 2021
 */

#include "handler.h"
#include <random>
#include <fstream>

int main(int argc, char const *argv[])
{
    std::default_random_engine e;
    std::uniform_real_distribution<float> u(-100.0, 100.0);
    ns_geo::PointSet2f ps;
    std::fstream opfile("../pyDrawer/op.csv", std::ios::out);
    std::fstream chfile("../pyDrawer/ch.csv", std::ios::out);
    for (int i = 0; i != 40; ++i)
    {
        ps.push_back({u(e), u(e)});
        opfile << ps.back().x() << ',' << ps.back().y() << std::endl;
    }
    auto polygon = ns_ch::ConvecHull::genConvecHull(ps);
    for (const auto &point : polygon)
        chfile << point.x() << ',' << point.y() << std::endl;
    opfile.close();
    chfile.close();
    return 0;
}
