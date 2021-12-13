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
        static ns_geo::Polygonf genConvecHull(const ns_geo::PointSet2f &ps);

    private:
        ConvecHull() = delete;

    protected:
        static bool pointAtLineRight(const ns_geo::Point2f &p, const ns_geo::Line2f &l);
    };
} // namespace ns_ch
