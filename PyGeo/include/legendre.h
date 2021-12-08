#pragma once

#include <iostream>
#include <math.h>
#include <unordered_map>
#include "parameter.h"

namespace ns_pygeo
{
#pragma region class IndexOrder
    /**
     * \brief this class serves the Legendre class
     */
    class IndexOrder
    {
    public:
        int _n;
        int _m;
        IndexOrder(int index_n, int order_m) : _n(index_n), _m(order_m) {}
        ~IndexOrder() {}
        // overload the operator '=='
        bool operator==(const IndexOrder &i) const;
        // hash function to get the hash value for each object
        static std::size_t indexorder_hash(const IndexOrder &i);
    };
#pragma endregion

#pragma region class Legendre
    /**
     *  \class Legendre [static class]
     *  \brief this class is designed to calculate the normalized spherical harmonic function.
     *         This class provides four methods for calculating Legendre function: 
     *              [1] standard forward column method
     *              [2] standard forward row method
     *              [3] cross order recursive method 
     *              [4] Belikov column method
     */
    class Legendre
    {
    public:
        using value_type = long double;
        /**
         * \brief the function pointer
         */
        using legendre_fun = value_type (*)(int index_n, int order_m, float angle);
        using indexorder_hashfun = std::size_t (*)(const IndexOrder &i);
        using legendre_map = std::unordered_map<IndexOrder, value_type, indexorder_hashfun>;
        // [P_0_0] [P_1_0] [P_1_1] [cos_angle] [sin_angle] [cot_angle] [return value] [is reture]
        using check_tuple = std::tuple<value_type, value_type, value_type, value_type, value_type, value_type, value_type, bool>;

    private:
        Legendre() = default;
        ~Legendre() {}
        /**
         * \brief two middle functions in the belikov method
         */
        static value_type _belikov_denorm(int index_n, int order_m, float angle);
        static value_type _belikov_denorm(int index_n, int order_m);
        /**
         * \brief function to check and initialize the necessary things before starting the algorithm
         * \return [P_0_0] [P_1_0] [P_1_1] [cos_angle] [sin_angle] [cot_angle] [return value] [is reture]
         */
        static check_tuple _check_init(int index_n, int order_m, float radian_angle);

    public:
        /**
         * \brief loop methods
         * \attention the angle is passed by the radian form
         */
        // [1] standard forward column method
        static value_type forwardColumn(int index_n, int order_m, float radian_angle);
        static value_type forwardColumn(const IndexOrder &io, float radian_angle);
        // [2] standard forward row method
        static value_type forwardRow(int index_n, int order_m, float radian_angle);
        static value_type forwardRow(const IndexOrder &io, float radian_angle);
        // [3] cross order recursive method
        static value_type crossOrder(int index_n, int order_m, float radian_angle);
        static value_type crossOrder(const IndexOrder &io, float radian_angle);
        // [4] Belikov column method
        static value_type belikov(int index_n, int order_m, float radian_angle);
        static value_type belikov(const IndexOrder &io, float radian_angle);
        /**
         *  accuracy estimation
         * \attention the angle is passed by the radian form
         */
        static value_type accuracy_estimation(int index_n, float radian_angle, legendre_fun fun);
        /**
         * \brief calculate all elememts from [n = 0,m = 0] to [n = end_n,m = end_n]
         *        based on the standard forward column method
         *        It's faster than you calculate one by one
         * \attention the angle is passed by the radian form
         * \return a map contains all results
         */
        static legendre_map legendre_dataset(int end_n, float radian_angle);
    };
#pragma endregion
} // namespace ns_pygeo
