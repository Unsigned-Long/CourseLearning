/**
 * @file handler.h
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2021-12-13
 * 
 * @copyright Copyright (c) 2021
 */

#pragma once

#include <iostream>
#include "polygon.hpp"
#include "line.hpp"

namespace ns_ch
{
    class ConvecHull
    {
    public:
        static std::vector<std::size_t> genIndex(const ns_geo::PointSet2f &ps);

        static ns_geo::Polygonf genPolygon(const ns_geo::PointSet2f &ps);

    private:
        ConvecHull() = delete;
    };
} // namespace ns_ch
