/**
 * @file dichSearch.cpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2022-01-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <iostream>

using fun_type = double (*)(double);

/**
 * @brief the dichotomous search method to find the min value
 * 
 * @param fun the target function
 * @param left the left boundary
 * @param right the right boundary
 * @param stride the stride
 * @param log whether output the log infomation
 * @return std::pair<double, double> the minpos and the minval 
 */
std::pair<double, double> dichotomousSearch(const fun_type &fun, double left, double right, double stride = 1E-8, bool log = false)
{
    double temp_left, temp_right, center;
    while (std::abs(right - left) > 2.5 * stride)
    {
        if (log)
            std::cout << "left: " << left << ", right: " << right << std::endl;
        center = (right + left) / 2.0;
        temp_left = center - stride;
        temp_right = center + stride;
        if (fun(temp_left) < fun(temp_right))
            right = temp_right;
        else
            left = temp_left;
    }
    double minpos = (right + left) / 2.0;
    double minval = fun(minpos);
    return {minpos, fun(minpos)};
}

int main(int argc, char const *argv[])
{
    auto target1 = [](double x) -> double
    {
        return 2.0 * x * x + 3.0 * x + 10.0;
    };
    auto res1 = dichotomousSearch(target1, -100.0, 500.0);
    std::cout << "the result is [minpos: " << res1.first << ", minval: " << res1.second << "]\n";

    auto target2 = [](double x) -> double
    {
        return 8.0 * x * x * x - 2.0 * x * x - 7.0 * x + 3.0;
    };
    auto res2 = dichotomousSearch(target2, 0.0, 1.0);
    std::cout << "the result is [minpos: " << res2.first << ", minval: " << res2.second << "]\n";
    return 0;
}
