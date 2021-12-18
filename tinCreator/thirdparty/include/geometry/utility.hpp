#pragma once

/**
 * @file utility.hpp
 * @author csl (3079625093@qq.com)
 * @brief Provide operation support for point classes
 * @version 0.1
 * @date 2021-12-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <cmath>
#include <fstream>
#include <string>
#include <exception>
#include <algorithm>
#include <array>
#include <vector>
#include <unordered_map>

namespace ns_geo
{
#pragma region gemetry types
    template <typename _Ty>
    class Point2;
    /**
     * \brief some Commonly used Point2 types
     */
    using Point2f = Point2<float>;
    using Point2d = Point2<double>;
    using Point2i = Point2<int>;

    template <typename _Ty>
    class RefPoint2;
    /**
     * \brief some Commonly used RefPoint2 types
     */
    using RefPoint2f = RefPoint2<float>;
    using RefPoint2d = RefPoint2<double>;
    using RefPoint2i = RefPoint2<int>;

    template <typename _Ty>
    class Point3;
    /**
     * \brief some Commonly used Point3 types
     */
    using Point3f = Point3<float>;
    using Point3d = Point3<double>;
    using Point3i = Point3<int>;

    template <typename _Ty>
    class RefPoint3;
    /**
     * \brief some Commonly used RefPoint3 types
     */
    using RefPoint3f = RefPoint3<float>;
    using RefPoint3d = RefPoint3<double>;
    using RefPoint3i = RefPoint3<int>;

    template <typename _Ty>
    class Line2;
    /**
     * \brief some Commonly used Line2 types
     */
    using Line2d = Line2<double>;
    using Line2f = Line2<float>;
    using Line2i = Line2<int>;

    template <typename _Ty>
    class RefLine2;
    /**
     * \brief some Commonly used RefLine2 types
     */
    using RefLine2d = RefLine2<double>;
    using RefLine2f = RefLine2<float>;
    using RefLine2i = RefLine2<int>;

    template <typename _Ty>
    class Line3;
    /**
     * \brief some Commonly used Line3 types
     */
    using Line3d = Line3<double>;
    using Line3f = Line3<float>;
    using Line3i = Line3<int>;

    template <typename _Ty>
    class RefLine3;
    /**
     * \brief some Commonly used RefLine3 types
     */
    using RefLine3d = RefLine3<double>;
    using RefLine3f = RefLine3<float>;
    using RefLine3i = RefLine3<int>;

    template <typename _Ty>
    class Rectangle;
    /**
     * \brief some Commonly used Rectangle types
     */
    using Rectangled = Rectangle<double>;
    using Rectanglef = Rectangle<float>;
    using Rectanglei = Rectangle<int>;

    template <typename _Ty>
    class RefRectangle;
    /**
     * \brief some Commonly used RefRectangle types
     */
    using RefRectangled = RefRectangle<double>;
    using RefRectanglef = RefRectangle<float>;
    using RefRectanglei = RefRectangle<int>;

    template <typename _Ty>
    class Triangle2;
    /**
     * \brief some Commonly used Triangle2 types
     */
    using Triangle2d = Triangle2<double>;
    using Triangle2f = Triangle2<float>;
    using Triangle2i = Triangle2<int>;

    template <typename _Ty>
    class RefTriangle2;
    /**
     * \brief some Commonly used RefTriangle2 types
     */
    using RefTriangle2d = RefTriangle2<double>;
    using RefTriangle2f = RefTriangle2<float>;
    using RefTriangle2i = RefTriangle2<int>;

    template <typename _Ty>
    class Triangle3;
    /**
     * \brief some Commonly used Triangle3 types
     */
    using Triangle3d = Triangle3<double>;
    using Triangle3f = Triangle3<float>;
    using Triangle3i = Triangle3<int>;

    template <typename _Ty>
    class RefTriangle3;
    /**
     * \brief some Commonly used RefTriangle3 types
     */
    using RefTriangle3d = RefTriangle3<double>;
    using RefTriangle3f = RefTriangle3<float>;
    using RefTriangle3i = RefTriangle3<int>;

    template <typename _Ty>
    class Polygon;
    /**
     * \brief some Commonly used Polygon types
     */
    using Polygond = Polygon<double>;
    using Polygonf = Polygon<float>;
    using Polygoni = Polygon<int>;

    template <typename _Ty>
    class RefPolygon;
    /**
     * \brief some Commonly used RefPolygon types
     */
    using RefPolygond = RefPolygon<double>;
    using RefPolygonf = RefPolygon<float>;
    using RefPolygoni = RefPolygon<int>;

    template <typename _Ty>
    class LineString2;
    /**
     * \brief some Commonly used LineString2 types
     */
    using LineString2d = LineString2<double>;
    using LineString2f = LineString2<float>;
    using LineString2i = LineString2<int>;

    template <typename _Ty>
    class RefLineString2;
    /**
     * \brief some Commonly used RefLineString2 types
     */
    using RefLineString2d = RefLineString2<double>;
    using RefLineString2f = RefLineString2<float>;
    using RefLineString2i = RefLineString2<int>;

    template <typename _Ty>
    class LineString3;
    /**
     * \brief some Commonly used LineString3 types
     */
    using LineString3d = LineString3<double>;
    using LineString3f = LineString3<float>;
    using LineString3i = LineString3<int>;

    template <typename _Ty>
    class RefLineString3;
    /**
     * \brief some Commonly used RefLineString3 types
     */
    using RefLineString3d = RefLineString3<double>;
    using RefLineString3f = RefLineString3<float>;
    using RefLineString3i = RefLineString3<int>;

    template <typename _PointType>
    class KdTree2;
    using KdTree2i = KdTree2<Point2i>;
    using KdTree2f = KdTree2<Point2f>;
    using KdTree2d = KdTree2<Point2d>;

    template <typename _PointType>
    class KdTree3;
    using KdTree3i = KdTree3<Point3i>;
    using KdTree3f = KdTree3<Point3f>;
    using KdTree3d = KdTree3<Point3d>;

    template <typename _PointType>
    class RefKdTree2;
    using RefKdTree2i = RefKdTree2<RefPoint2i>;
    using RefKdTree2f = RefKdTree2<RefPoint2f>;
    using RefKdTree2d = RefKdTree2<RefPoint2d>;

    template <typename _PointType>
    class RefKdTree3;
    using RefKdTree3i = RefKdTree3<RefPoint3i>;
    using RefKdTree3f = RefKdTree3<RefPoint3f>;
    using RefKdTree3d = RefKdTree3<RefPoint3d>;

#pragma endregion

#pragma region helpers
    /**
     * @brief calculate the stride between the 'from' point to the 'to' point
     * 
     * @tparam _Ty the type of value
     * @param from the start point
     * @param to the end point
     * @return std::array<_Ty, 3> return a array contains three elements
     */
    template <typename _Ty>
    std::array<_Ty, 2> stride(const Point2<_Ty> &from, const Point2<_Ty> &to)
    {
        return std::array<_Ty, 2>{to.x() - from.x(), to.y() - from.y()};
    }

    /**
     * @brief calculate the stride between the 'from' point to the 'to' point
     * 
     * @tparam _Ty the type of value
     * @param from the start point
     * @param to the end point
     * @return std::array<_Ty, 3> return a array contains three elements
     */
    template <typename _Ty>
    std::array<_Ty, 3> stride(const Point3<_Ty> &from, const Point3<_Ty> &to)
    {
        return std::array<_Ty, 3>{to.x() - from.x(), to.y() - from.y(), to.z() - from.z()};
    }

    /**
     * @brief Calculate the distance between two points
     * 
     * @tparam _Ty the type of value
     * @param p1 one of two points
     * @param p2 one of two points
     * @return float the distance between two points
     */
    template <typename _Ty>
    float distance(const Point2<_Ty> &p1, const Point2<_Ty> &p2)
    {
        return static_cast<float>(std::sqrt(std::pow(p1.x() - p2.x(), 2) + std::pow(p1.y() - p2.y(), 2)));
    }

    /**
     * @brief Calculate the distance between two points
     * 
     * @tparam _Ty the type of value
     * @param p1 one of two points
     * @param p2 one of two points
     * @return float the distance between two points
     */
    template <typename _Ty>
    float distance(const Point3<_Ty> &p1, const Point3<_Ty> &p2)
    {
        return static_cast<float>(std::sqrt(std::pow(p1.x() - p2.x(), 2) + std::pow(p1.y() - p2.y(), 2) + std::pow(p1.z() - p2.z(), 2)));
    }

    /**
     * @brief calculate the distance from the point to the line
     * 
     * @tparam _Ty the type of value
     * @param p the point
     * @param l the line
     * @return float the distance from the point to the line
     */
    template <typename _Ty>
    float distance(const Point2<_Ty> &p, const Line2<_Ty> &l)
    {
        auto vec1 = stride(l.p1(), l.p2());
        auto vec2 = stride(l.p1(), p);
        float z = vec1[0] * vec2[1] - vec1[1] * vec2[0];
        float dis = std::abs(z) / distance(l.p1(), l.p2());
        return dis;
    }

    /**
     * @brief calculate the distance from the point to the line
     * 
     * @tparam _Ty the type of value
     * @param p the point
     * @param l the line
     * @return float the distance from the point to the line
     */
    template <typename _Ty>
    float distance(const Point3<_Ty> &p, const Line3<_Ty> &l)
    {
        float vec1_x = l.p2().x() - l.p1().x();
        float vec1_y = l.p2().y() - l.p1().y();
        float vec1_z = l.p2().z() - l.p1().z();
        float vec2_x = p.x() - l.p1().x();
        float vec2_y = p.y() - l.p1().y();
        float vec2_z = p.z() - l.p1().z();
        auto val1 = std::pow(vec1_y * vec2_z - vec1_z * vec2_y, 2);
        auto val2 = std::pow(vec2_x * vec1_z - vec1_x * vec2_z, 2);
        auto val3 = std::pow(vec1_x * vec2_y - vec1_y * vec2_x, 2);
        float dis = std::sqrt(val1 + val2 + val3) / distance(l.p1(), l.p2());
        return dis;
    }

    namespace RHandRule
    {
        /**
         * @brief calculate the azimuth according the left hand rule
         * 
         * @tparam _Ty the type of value
         * @param p1 one of two points
         * @param p2 one of two points
         * @return float the azimuth[radian]
         */
        template <typename _Ty>
        float azimuth(const Point2<_Ty> &from, const Point2<_Ty> &to)
        {
            float detaX = to.x() - from.x();
            float detaY = to.y() - from.y();
            float angle = std::atan2(detaX, detaY);
            if (detaX < 0.0)
                angle += 2 * M_PI;
            return angle;
        }

        /**
         * @brief judge whether the point is at the left of the line
         * 
         * @tparam _Ty the template type
         * @param p the point[2d]
         * @param l the line[2d]
         * @return true 
         * @return false 
         */
        template <typename _Ty>
        bool palleft(const Point2<_Ty> &p, const Line2<_Ty> &l)
        {
            auto v1 = stride(l.p1(), l.p2());
            auto v2 = stride(l.p1(), p);
            return static_cast<float>(v1[0] * v2[1] - v1[1] * v2[0]) > 0.0;
        }

        /**
         * @brief judge whether the point is at the right of the line
         * 
         * @tparam _Ty the template type
         * @param p the point[2d]
         * @param l the line[2d]
         * @return true 
         * @return false 
         */
        template <typename _Ty>
        bool palright(const Point2<_Ty> &p, const Line2<_Ty> &l)
        {
            auto v1 = stride(l.p1(), l.p2());
            auto v2 = stride(l.p1(), p);
            return static_cast<float>(v1[0] * v2[1] - v1[1] * v2[0]) < 0.0;
        }
    } // namespace RHandRule

    namespace LHandRule
    {
        /**
         * @brief calculate the azimuth according the left hand rule
         * 
         * @tparam _Ty the type of value
         * @param p1 one of two points
         * @param p2 one of two points
         * @return float the azimuth[radian]
         */
        template <typename _Ty>
        float azimuth(const Point2<_Ty> &from, const Point2<_Ty> &to)
        {
            float detaX = to.x() - from.x();
            float detaY = to.y() - from.y();
            float angle = std::atan2(detaY, detaX);
            if (detaY < 0.0)
                angle += 2 * M_PI;
            return angle;
        }

        /**
         * @brief judge whether the point is at the left of the line
         * 
         * @tparam _Ty the template type
         * @param p the point[2d]
         * @param l the line[2d]
         * @return true 
         * @return false 
         */
        template <typename _Ty>
        bool palleft(const Point2<_Ty> &p, const Line2<_Ty> &l)
        {
            auto v1 = stride(l.p1(), l.p2());
            auto v2 = stride(l.p1(), p);
            return static_cast<float>(v1[0] * v2[1] - v1[1] * v2[0]) < 0.0;
        }

        /**
         * @brief judge whether the point is at the right of the line
         * 
         * @tparam _Ty the template type
         * @param p the point[2d]
         * @param l the line[2d]
         * @return true 
         * @return false 
         */
        template <typename _Ty>
        bool palright(const Point2<_Ty> &p, const Line2<_Ty> &l)
        {
            auto v1 = stride(l.p1(), l.p2());
            auto v2 = stride(l.p1(), p);
            return static_cast<float>(v1[0] * v2[1] - v1[1] * v2[0]) > 0.0;
        }
    } // namespace LHandRule

#pragma endregion
} // namespace ns_geo
