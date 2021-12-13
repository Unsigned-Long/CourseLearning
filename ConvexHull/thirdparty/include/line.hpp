#pragma once

/**
 * @file line.hpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2021-12-06
 * @copyright Copyright (c) 2021
 * 
 * @brief the details
 *        [1] class type
 *              0. Line2<_Ty>, Line3<_Ty>
 *              1. RefLine2<_Ty>, RefLine3<_Ty>
 * 
 *        [2] methods for Line
 *              0. azimuthRHR, azimuthLHR
 *              1. length
 *              2. operator "<<" for Line2<_Ty>, Line3<_Ty>
 *              3. operator "<<" for RefLine2<_Ty>, RefLine3<_Ty>
 */

#include "point.hpp"

namespace ns_geo
{
#pragma region Line2
    /**
     * \brief a sample template class to describe the 2-dime lines
     */
    template <typename _Ty = float>
    class Line2
    {
    public:
        using value_type = _Ty;
        using point_type = ns_geo::Point2<value_type>;
        using ary_type = std::array<point_type, 2>;

    private:
        point_type _p1;
        point_type _p2;

    public:
        /**
         * \brief constructors
         */
        Line2() = default;
        Line2(const point_type &p1, const point_type &p2)
            : _p1(p1), _p2(p2) {}
        Line2(const point_type points[2])
            : _p1(points[0]), _p2(points[1]) {}
        Line2(const ary_type &points)
            : _p1(points[0]), _p2(points[1]) {}
        Line2(value_type p1x, value_type p1y, value_type p2x, value_type p2y)
            : _p1(p1x, p1y), _p2(p2x, p2y) {}

        const point_type &p1() const { return this->_p1; }
        point_type &p1() { return this->_p1; }

        const point_type &p2() const { return this->_p2; }
        point_type &p2() { return this->_p2; }

        ary_type points() const { return ary_type{this->_p1, this->_p2}; }

        float length() const { return ns_geo::distance(_p1, _p2); }

        float azimuthRHR() const { return ns_geo::azimuthRHR(_p1, _p2); }

        float azimuthLHR() const { return ns_geo::azimuthLHR(_p1, _p2); }
    };
    /**
     * \brief overload operator "<<" for Line2
     */
    template <typename _Ty = float>
    std::ostream &operator<<(std::ostream &os, const Line2<_Ty> &line)
    {
        os << '{';
        os << line.p1() << ", ";
        os << line.p2() << '}';
        return os;
    }

#pragma endregion

#pragma region Line3
    /**
     * \brief a sample template class to describe the 3-dime lines
     */
    template <typename _Ty = float>
    class Line3
    {
    public:
        using value_type = _Ty;
        using point_type = ns_geo::Point3<value_type>;
        using ary_type = std::array<point_type, 2>;

    private:
        point_type _p1;
        point_type _p2;

    public:
        /**
         * \brief constructors
         */
        Line3() = default;
        Line3(const point_type &p1, const point_type &p2)
            : _p1(p1), _p2(p2) {}
        Line3(const point_type points[2])
            : _p1(points[0]), _p2(points[1]) {}
        Line3(const ary_type &points)
            : _p1(points[0]), _p2(points[1]) {}
        Line3(value_type p1x, value_type p1y, value_type p1z,
              value_type p2x, value_type p2y, value_type p2z)
            : _p1(p1x, p1y, p1z), _p2(p2x, p2y, p2z) {}

        const point_type &p1() const { return this->_p1; }
        point_type &p1() { return this->_p1; }

        const point_type &p2() const { return this->_p2; }
        point_type &p2() { return this->_p2; }

        ary_type points() const { return ary_type{this->_p1, this->_p2}; }

        float length() const { return ns_geo::distance(_p1, _p2); }
    };
    /**
     * \brief overload operator "<<" for Line3
     */
    template <typename _Ty = float>
    std::ostream &operator<<(std::ostream &os, const Line3<_Ty> &line)
    {
        os << '{';
        os << line.p1() << ", ";
        os << line.p2() << '}';
        return os;
    }
#pragma endregion

#pragma region RefLine2
    template <typename _Ty = float>
    class RefLine2
    {
    public:
        using value_type = _Ty;
        using id_type = uint;
        using refpoint_type = ns_geo::RefPoint2<value_type>;
        using refpointset_type = RefPointSet2<value_type>;
        using ary_type = std::array<refpoint_type, 2>;

    public:
        friend class RefPointSet2<value_type>;

    private:
        id_type _pid1;
        id_type _pid2;
        // thr reference point set's pointer
        const refpointset_type *const _rps;

    protected:
        /**
         * \brief constructors
         */
        RefLine2(id_type pid1, id_type pid2, const refpointset_type *const refpointset)
            : _pid1(pid1), _pid2(pid2), _rps(refpointset) {}

        RefLine2() = delete;

    public:
        const refpointset_type *const refPointSet() const { return this->_rps; };

        operator Line2<value_type>() { return Line2<value_type>(this->p1(), this->p2()); }

        const refpoint_type &p1() const { return this->_rps->at(this->_pid1); }

        const refpoint_type &p2() const { return this->_rps->at(this->_pid2); }

        ary_type refPoints() const { return ary_type{this->p1(), this->p2()}; }

        float length() const { return ns_geo::distance(p1(), p2()); }

        float azimuthRHR() const { return ns_geo::azimuthRHR(p1(), p2()); }

        float azimuthLHR() const { return ns_geo::azimuthLHR(p1(), p2()); }
    };
    /**
     * \brief overload operator "<<" for RefLine2
     */
    template <typename _Ty = float>
    std::ostream &operator<<(std::ostream &os, const RefLine2<_Ty> &line)
    {
        auto p1 = line.p1();
        auto p2 = line.p2();
        os << '{';
        os << p1.id() << ": [" << p1.x() << ", " << p1.y() << ']' << ", ";
        os << p2.id() << ": [" << p2.x() << ", " << p2.y() << "]}";
        return os;
    }
#pragma endregion

#pragma region RefLine3

    template <typename _Ty = float>
    class RefLine3
    {
    public:
        using value_type = _Ty;
        using id_type = uint;
        using refpoint_type = ns_geo::RefPoint3<value_type>;
        using refpointset_type = RefPointSet3<value_type>;
        using ary_type = std::array<refpoint_type, 2>;

    public:
        friend class RefPointSet3<value_type>;

    private:
        id_type _pid1;
        id_type _pid2;
        const refpointset_type *const _refpointset;

    protected:
        /**
         * \brief constructors
         */
        RefLine3(id_type pid1, id_type pid2, const refpointset_type *const refpointset)
            : _pid1(pid1), _pid2(pid2), _refpointset(refpointset) {}

        RefLine3() = delete;

    public:
        const refpointset_type *const refPointSet() const { return this->_rps; };

        operator Line3<value_type>(){return Line3<value_type>(this->p1(), this->p2());}

        const refpoint_type &p1() const { return this->_refpointset->at(this->_pid1); }

        const refpoint_type &p2() const { return this->_refpointset->at(this->_pid2); }

        ary_type refPoints() const { return ary_type{this->p1(), this->p2()}; }

        float length() const { return ns_geo::distance(p1(), p2()); }
    };
    /**
     * \brief overload operator "<<" for RefLine3
     */
    template <typename _Ty = float>
    std::ostream &operator<<(std::ostream &os, const RefLine3<_Ty> &line)
    {
        auto p1 = line.p1();
        auto p2 = line.p2();
        os << '{';
        os << p1.id() << ": [" << p1.x() << ", " << p1.y() << ", " << p1.z() << ']' << ", ";
        os << p2.id() << ": [" << p2.x() << ", " << p2.y() << ", " << p2.z() << "]}";
        return os;
    }
#pragma endregion
} // namespace ns_geo
