#include "legendre.h"
#include <exception>
#include <vector>

namespace ns_pygeo
{
#pragma region class IndexOrder
    bool IndexOrder::operator==(const IndexOrder &i) const
    {
        // equal if two objects' order and index are same
        return this->_n == i._n && this->_m == i._m;
    }
    std::size_t IndexOrder::indexorder_hash(const IndexOrder &i)
    {
        // use hash template class to get the hash value of the build-in data type
        std::hash<int> h;
        return h(i._n) ^ h(i._m);
    }
#pragma endregion

#pragma region class Legendre

    Legendre::check_tuple Legendre::_check_init(int n, int m, float angle)
    {
        // If m or n is negative, an exception is thrown.
        if (n < 0 || m < 0)
            throw std::invalid_argument("the index and the order can't be a negative where n[" + std::to_string(n) + "] " + "m[" + std::to_string(m) + ']');
        // Calculate initial value
        static constexpr value_type P_0_0 = 1.0;
        const value_type cos_angle = std::cos(angle);
        const value_type sin_angle = std::sin(angle);
        const value_type cot_angle = cos_angle / sin_angle;
        const value_type P_1_0 = std::sqrt(3) * cos_angle;
        const value_type P_1_1 = std::sqrt(3) * sin_angle;
        // If the calculated value is within the initial value range, the result will be output directly
        value_type return_value;
        bool is_return = false;
        if (n == 0 && m == 0)
        {
            return_value = P_0_0;
            is_return = true;
        }
        if (n == 1 && m == 0)
        {
            return_value = P_1_0;
            is_return = true;
        }
        if (n == 1 && m == 1)
        {
            return_value = P_1_1;
            is_return = true;
        }
        if (m > n)
        {
            return_value = 0.0;
            is_return = true;
        }
        // [P_0_0] [P_1_0] [P_1_1] [cos_angle] [sin_angle] [cot_angle] [return value] [is reture]
        return std::make_tuple(P_0_0, P_1_0, P_1_1, cos_angle, sin_angle, cot_angle, return_value, is_return);
    }

    Legendre::value_type Legendre::crossOrder(int n, int m, float angle)
    {
        auto [P_0_0, P_1_0, P_1_1, cos_angle, sin_angle, cot_angle, return_value, is_return] = Legendre::_check_init(n, m, angle);
        if (is_return == true)
            return return_value;
        // When m equals 0 or 1
        if (m == 0 || m == 1)
        {
            return Legendre::forwardColumn(n, m, angle);
        }
        // When m is greater than or equal to 2
        else
        {
            int start_n = 0, start_m = 0;
            n % 2 == 0 ? (m % 2 == 0 ? (start_n = start_m = 0) : (start_n = 2, start_m = 1))
                       : (m % 2 == 0 ? (start_n = 1, start_m = 0) : (start_n = start_m = 1));
            // create the submap
            std::vector<value_type> first_submap((n - start_n) / 2 + 1, 0.0);
            std::vector<value_type> second_submap((n - start_n) / 2 + 1, 0.0);
            // init the first col of the submap
            value_type f2, f3, f4, f5;
            f2 = std::sqrt(static_cast<value_type>(2 * (start_m + 1) + 1));
            auto first_val = Legendre::forwardColumn(start_m, start_m, angle);
            auto second_val = f2 * cos_angle * first_val;
            first_submap.at(0) = Legendre::forwardColumn(start_n, start_m, angle);
            for (int cur_n = 2; cur_n <= n; ++cur_n)
            {
                if (cur_n != start_m + 1)
                {
                    f3 = std::sqrt(static_cast<value_type>(2 * cur_n + 1) / (static_cast<value_type>(cur_n - start_m) * (cur_n + start_m)));
                    f4 = std::sqrt(static_cast<value_type>(2 * cur_n - 1));
                    f5 = std::sqrt(static_cast<value_type>(cur_n - start_m - 1) * (cur_n + start_m - 1) / (2 * cur_n - 3));
                    auto temp_val = f3 * (f4 * cos_angle * second_val - f5 * first_val);
                    first_val = second_val;
                    second_val = temp_val;
                }
                if ((cur_n - start_n) % 2 == 0)
                    first_submap.at((cur_n - start_n) / 2) = second_val;
            }
            // calculate the rest cols of the submap until the result is calculated
            // here we create two pointers to point to two containers in turn
            std::vector<value_type> *get_submap;
            std::vector<value_type> *set_submap;
            value_type alpha, beta, gamma, delta;
            for (int cur_m = start_m + 2; cur_m <= m; cur_m += 2)
                for (int cur_n = start_n - start_m + cur_m; cur_n <= n; cur_n += 2)
                {
                    // swap the direction of two pointers
                    if ((cur_m - start_m) / 2 % 2 == 1)
                    {
                        get_submap = &first_submap;
                        set_submap = &second_submap;
                    }
                    else
                    {
                        get_submap = &second_submap;
                        set_submap = &first_submap;
                    }
                    // get three values  we need
                    value_type left_val = get_submap->at((cur_n - start_n) / 2);
                    value_type up_val = set_submap->at((cur_n - start_n) / 2 - 1);
                    value_type left_up_val = get_submap->at((cur_n - start_n) / 2 - 1);
                    cur_m == 2 ? delta = 2.0 : delta = 1.0;
                    // get the coefficients we need
                    alpha = std::sqrt(static_cast<value_type>(2 * cur_n + 1) * (cur_n - cur_m) * (cur_n - cur_m - 1) / (static_cast<value_type>(2 * cur_n - 3) * (cur_n + cur_m) * (cur_n + cur_m - 1)));
                    beta = std::sqrt(delta * (2 * cur_n + 1) * (cur_n + cur_m - 2) * (cur_n + cur_m - 3) / (static_cast<value_type>(2 * cur_n - 3) * (cur_n + cur_m) * (cur_n + cur_m - 1)));
                    gamma = std::sqrt(delta * (cur_n - cur_m + 1) * (cur_n - cur_m + 2) / (static_cast<value_type>(cur_n + cur_m) * (cur_n + cur_m - 1)));
                    // calculate the result and asign it to current position
                    set_submap->at((cur_n - start_n) / 2) = alpha * up_val + beta * left_up_val - gamma * left_val;
                }
            return set_submap->at((n - start_n) / 2);
        }
    }

    Legendre::value_type Legendre::forwardColumn(int n, int m, float angle)
    {
        auto [P_0_0, P_1_0, P_1_1, cos_angle, sin_angle, cot_angle, return_value, is_return] = Legendre::_check_init(n, m, angle);
        if (is_return == true)
            return return_value;
        value_type f1, f2, f3, f4, f5;
        if (n == m)
        {
            f1 = std::sqrt(static_cast<value_type>(2 * n + 1) / (2 * n));
            return f1 * sin_angle * Legendre::forwardColumn(n - 1, n - 1, angle);
        }
        else
        {
            f2 = std::sqrt(static_cast<value_type>(2 * (m + 1) + 1));
            auto first_val = Legendre::forwardColumn(m, m, angle);
            auto second_val = f2 * cos_angle * first_val;
            if (n != m + 1)
                for (int cur_n = m + 2; cur_n <= n; ++cur_n)
                {
                    f3 = std::sqrt(static_cast<value_type>(2 * cur_n + 1) / (static_cast<value_type>(cur_n - m) * (cur_n + m)));
                    f4 = std::sqrt(static_cast<value_type>(2 * cur_n - 1));
                    f5 = std::sqrt(static_cast<value_type>(cur_n - m - 1) * (cur_n + m - 1) / (2 * cur_n - 3));
                    auto temp_val = f3 * (f4 * cos_angle * second_val - f5 * first_val);
                    first_val = second_val;
                    second_val = temp_val;
                }
            return second_val;
        }
    }

    Legendre::value_type Legendre::forwardRow(int n, int m, float angle)
    {
        auto [P_0_0, P_1_0, P_1_1, cos_angle, sin_angle, cot_angle, return_value, is_return] = Legendre::_check_init(n, m, angle);
        if (is_return == true)
            return return_value;
        if (m == n)
        {
            auto cm = std::sqrt(static_cast<value_type>(2 * m + 1) / (2 * m));
            return cm * sin_angle * Legendre::forwardRow(m - 1, m - 1, angle);
        }
        else
        {
            auto first_val = Legendre::forwardRow(n, n, angle);
            int temp_m = n - 1;
            value_type g = 2.0 * (temp_m + 1) / std::sqrt(static_cast<value_type>(n - temp_m) * (n + temp_m + 1));
            value_type h = 0.0;
            value_type delta;
            temp_m == 0 ? delta = 2.0 : delta = 1.0;
            value_type second_val = 1.0 / std::sqrt(delta) * (g * cot_angle * first_val);
            if (n != m + 1)
                for (int cur_m = n - 2; cur_m >= m; --cur_m)
                {
                    g = 2.0 * (cur_m + 1) / std::sqrt(static_cast<value_type>(n - cur_m) * (n + cur_m + 1));
                    h = std::sqrt(static_cast<value_type>(n + cur_m + 2) * (n - cur_m - 1) / (static_cast<value_type>(n + cur_m + 1) * (n - cur_m)));
                    cur_m == 0 ? delta = 2.0 : delta = 1.0;
                    auto temp_val = 1.0 / std::sqrt(delta) * (g * cot_angle * second_val - h * first_val);
                    first_val = second_val;
                    second_val = temp_val;
                }
            return second_val;
        }
    }

    Legendre::value_type Legendre::_belikov_denorm(int n, int m)
    {
        // If m or n is negative, an exception is thrown.
        if (n < 0 || m < 0)
            throw std::invalid_argument("the index and the order must be be a positive instead of n[" + std::to_string(n) + "] " + "m[" + std::to_string(m) + ']');
        if ((n == 0 && m == 0) || (n == 1 && m == 0) || (n == 1 && m == 1))
            return 1.0;
        if (n < m)
            return 0.0;
        if (m == n)
            return std::sqrt(1.0 - 1.0 / (2 * n)) * Legendre::_belikov_denorm(n - 1, m - 1);
        else
            return std::sqrt(1.0 - std::pow(static_cast<value_type>(m) / n, 2)) * Legendre::_belikov_denorm(n - 1, m);
    }

    Legendre::value_type Legendre::_belikov_denorm(int n, int m, float angle)
    {
        auto [P_0_0, P_1_0, P_1_1, cos_angle, sin_angle, cot_angle, return_value, is_return] = Legendre::_check_init(n, m, angle);
        if (is_return == true)
            return return_value;
        std::vector<value_type> first_submap(n + 2, 0.0);
        std::vector<value_type> second_submap(n + 2, 0.0);
        first_submap.at(0) = 1.0;
        std::vector<value_type> *get_submap;
        std::vector<value_type> *set_submap;
        for (int cur_n = 1; true; ++cur_n)
            for (int cur_m = 0; cur_m != cur_n + 1; ++cur_m)
            {
                if (cur_n % 2 == 1)
                {
                    get_submap = &first_submap;
                    set_submap = &second_submap;
                }
                else
                {
                    get_submap = &second_submap;
                    set_submap = &first_submap;
                }
                if (cur_m == 0)
                    set_submap->at(0) = cos_angle * get_submap->at(0) - sin_angle / 2.0 * get_submap->at(1);
                else
                    set_submap->at(cur_m) = cos_angle * get_submap->at(cur_m) - sin_angle * (get_submap->at(cur_m + 1) / 4.0 - get_submap->at(cur_m - 1));
                if (cur_m == m && cur_n == n)
                    return set_submap->at(cur_m);
            }
        return 0.0;
    }

    Legendre::value_type Legendre::belikov(int n, int m, float angle)
    {
        auto P_denorm = Legendre::_belikov_denorm(n, m, angle);
        auto N = Legendre::_belikov_denorm(n, m);
        return std::sqrt(2.0 * n + 1) * N * P_denorm;
    }

    Legendre::value_type Legendre::accuracy_estimation(int index_n, float angle, legendre_fun fun)
    {
        Legendre::value_type sum = 0.0;
        for (int m = 0; m <= index_n; ++m)
            sum += std::pow(fun(index_n, m, angle), 2);
        return 1.0 / (2 * index_n + 1) * std::abs(sum - 2 * index_n - 1);
    }

    Legendre::value_type Legendre::forwardColumn(const IndexOrder &io, float angle)
    {
        return Legendre::forwardColumn(io._n, io._m, angle);
    }

    Legendre::value_type Legendre::forwardRow(const IndexOrder &io, float angle)
    {
        return Legendre::forwardRow(io._n, io._m, angle);
    }

    Legendre::value_type Legendre::crossOrder(const IndexOrder &io, float angle)
    {
        return Legendre::crossOrder(io._n, io._m, angle);
    }

    Legendre::value_type Legendre::belikov(const IndexOrder &io, float angle)
    {
        return Legendre::belikov(io._n, io._m, angle);
    }

    Legendre::legendre_map Legendre::legendre_dataset(int end_n, float angle)
    {
        // If m or n is negative, an exception is thrown.
        if (end_n < 0)
            throw std::invalid_argument("the index and the order must be be a positive instead of n[" + std::to_string(end_n) + "]");
        // Calculate initial value
        static constexpr value_type P_0_0 = 1.0;
        const value_type cos_angle = std::cos(angle);
        const value_type sin_angle = std::sin(angle);
        const value_type cot_angle = cos_angle / sin_angle;
        const value_type P_1_0 = std::sqrt(3) * cos_angle;
        const value_type P_1_1 = std::sqrt(3) * sin_angle;
        // create the container
        legendre_map con(0, IndexOrder::indexorder_hash);
        if (end_n == 0)
        {
            con.insert(std::make_pair(IndexOrder(0, 0), P_0_0));
        }
        else
        {
            con.insert(std::make_pair(IndexOrder(0, 0), P_0_0));
            con.insert(std::make_pair(IndexOrder(1, 0), P_1_0));
            con.insert(std::make_pair(IndexOrder(1, 1), P_1_1));
            if (end_n > 1)
            {
                value_type f1, f2, f3, f4, f5;
                // step 1 : calculate the edge elememts' value
                value_type first_val = P_1_1, second_val;
                for (int cur_n = 2; cur_n <= end_n; ++cur_n)
                {
                    f1 = std::sqrt(static_cast<value_type>(2 * cur_n + 1) / (2 * cur_n));
                    auto val = f1 * sin_angle * first_val;
                    con.insert(std::make_pair(IndexOrder(cur_n, cur_n), val));
                    first_val = val;
                }
                // step 2 : loop
                for (int cur_m = 0; cur_m < end_n; ++cur_m)
                {
                    f2 = std::sqrt(static_cast<value_type>(2 * (cur_m + 1) + 1));
                    first_val = (con.find(IndexOrder(cur_m, cur_m)))->second;
                    second_val = f2 * cos_angle * first_val;
                    con.insert(std::make_pair(IndexOrder(cur_m + 1, cur_m), second_val));
                    for (int cur_n = cur_m + 2; cur_n <= end_n; ++cur_n)
                    {
                        f3 = std::sqrt(static_cast<value_type>(2 * cur_n + 1) / (static_cast<value_type>(cur_n - cur_m) * (cur_n + cur_m)));
                        f4 = std::sqrt(static_cast<value_type>(2 * cur_n - 1));
                        f5 = std::sqrt(static_cast<value_type>(cur_n - cur_m - 1) * (cur_n + cur_m - 1) / (2 * cur_n - 3));
                        auto temp_val = f3 * (f4 * cos_angle * second_val - f5 * first_val);
                        first_val = second_val;
                        second_val = temp_val;
                        con.insert(std::make_pair(IndexOrder(cur_n, cur_m), second_val));
                    }
                }
            }
        }
        return con;
    }
#pragma endregion
} // namespace ns_pygeo
