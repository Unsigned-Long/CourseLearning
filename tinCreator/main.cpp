/**
 * @file main.cpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2021-12-17
 * 
 * @copyright Copyright (c) 2021
 */

#include "tin.h"
#include <random>

int main(int argc, char const *argv[])
{
    std::default_random_engine e;
    std::uniform_real_distribution<float> u(-100.0, 100.0);
    ns_geo::PointSet2f ps;
    std::fstream pfile("../pyDrawer/pf.csv", std::ios::out),
        tfile("../pyDrawer/tf.csv", std::ios::out),
        vfile("../pyDrawer/vf.csv", std::ios::out);

    for (int i = 0; i != 50; ++i)
    {
        ps.push_back({u(e), u(e)});
        pfile << ps.back().x() << ',' << ps.back().y() << std::endl;
    }

    ns_tin::TinCreator tin(ps);
    tin.insert({-25.0, 25});
    for (const auto &tri : tin.tins())
    {
        tfile << tri.p1().x() << ',' << tri.p1().y() << ',';
        tfile << tri.p2().x() << ',' << tri.p2().y() << ',';
        tfile << tri.p3().x() << ',' << tri.p3().y() << std::endl;
    }

    for (const auto &vor : tin.voronois())
    {
        auto voronoi = vor.voronoi();
        for (const auto &p : voronoi)
            vfile << p.x() << ',' << p.y() << ',';
        vfile << voronoi.front().x() << ',' << voronoi.front().y() << std::endl;
    }
    pfile.close(), tfile.close(), vfile.close();

    return 0;
}
