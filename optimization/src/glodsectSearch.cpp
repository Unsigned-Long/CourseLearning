/**
 * @file glodsectSearch.cpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2022-01-24
 * 
 * @copyright Copyright (c) 2022
 */

#include <iostream>
#include <cmath>

static constexpr double scalar = 0.618;

using fun_type = double (*)(double);

/**
 * @brief the gloden section search method to find the min value
 * 
 * @param fun the target function
 * @param left the left boundary
 * @param right the right boundary
 * @param log whether output the log infomation
 * @param threshold the threshold
 * @return std::pair<double, double> the minpos and the minval 
 */
std::pair<double, double> glodenSectionSearch(const fun_type &fun, double left, double right, bool log = false, double threshold = 1E-8)
{
    bool fromLeft;
    double temp_val, temp_left = right - scalar * (right - left), temp_right = left + scalar * (right - left);

    double temp_left_val = fun(temp_left), temp_right_val = fun(temp_right);

    if (temp_left_val < temp_right_val)
        right = temp_right, temp_val = temp_left_val, fromLeft = false;
    else
        left = temp_left, temp_val = temp_right_val, fromLeft = true;

    while ((right - left) > threshold)
    {
        if (log)
            std::cout << "left: " << left << ", right: " << right << std::endl;

        if (fromLeft)
        {
            temp_left = temp_right;
            temp_right = left + scalar * (right - left);
            temp_left_val = temp_val;
            temp_right_val = fun(temp_right);
        }
        else
        {
            temp_right = temp_left;
            temp_left = right - scalar * (right - left);
            temp_right_val = temp_val;
            temp_left_val = fun(temp_left);
        }

        if (temp_left_val < temp_right_val)
            right = temp_right, temp_val = temp_left_val, fromLeft = false;
        else
            left = temp_left, temp_val = temp_right_val, fromLeft = true;
    }

    double minpos = (right + left) / 2.0;
    double minval = fun(minpos);
    return {minpos, fun(minpos)};
}

int main(int argc, char const *argv[])
{
    auto target = [](double x) -> double
    {
        return std::pow(M_E, x) - 5.0 * x;
    };
    auto res = glodenSectionSearch(target, 1.0, 2.0);
    std::cout << "the result is [minpos: " << res.first << ", minval: " << res.second << "]\n";

    /**
     * @brief output
     * the result is [minpos: 1.60943, minval: -3.04719]
     */

    return 0;
}
