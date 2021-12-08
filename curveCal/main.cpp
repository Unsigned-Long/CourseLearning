/**
 * @file main.cpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2021-12-07
 * @copyright Copyright (c) 2021
 */

#include "handler.h"
#include <fstream>

void test(const ns_cc::Handler &handler)
{
    std::fstream file1("../pyDrawer/output_part1.csv", std::ios::out);
    std::fstream file2("../pyDrawer/output_part2.csv", std::ios::out);
    std::fstream file3("../pyDrawer/output_part3.csv", std::ios::out);
    std::fstream file4("../pyDrawer/output_part4.csv", std::ios::out);
    std::fstream file5("../pyDrawer/output_part5.csv", std::ios::out);
    for (float K = 14000.000; K < 16000.000; K += 1.0)
    {
        auto pos = handler.calculate(K, false);
        if (K < handler.K_ZH())
            file1 << pos.x() << ',' << pos.y() << std::endl;
        else if (K < handler.K_HY())
            file2 << pos.x() << ',' << pos.y() << std::endl;
        else if (K < handler.K_YH())
            file3 << pos.x() << ',' << pos.y() << std::endl;
        else if (K < handler.K_HZ())
            file4 << pos.x() << ',' << pos.y() << std::endl;
        else
            file5 << pos.x() << ',' << pos.y() << std::endl;
    }
    file1.close();
    file2.close();
    file3.close();
    file4.close();
    file5.close();
}

void require(const ns_cc::Handler &handler)
{
    std::fstream file("../output/output.log", std::ios::out);
    handler.outputReport(14000, 16000, 10, file);
    file.close();
}

int main(int argc, char const *argv[])
{
    auto handler = ns_cc::Handler(14906.807, {144534.846, 99482.202}, ns_angle::Degree(66.0 + 1.0 / 60.0 + 15.0 / 3600.0),
                                  ns_cc::CornerDir::RIGHT, 700, 100, ns_angle::Degree(273.0 + 15.0 / 60.0 + 23.4 / 3600.0));
    ::test(handler);
    // ::require(handler);
    return 0;
}
