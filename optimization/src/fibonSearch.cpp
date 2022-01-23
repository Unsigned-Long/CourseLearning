/**
 * @file fibonSearch.cpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2022-01-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <iostream>
#include <cmath>

/**
 * @brief calculate the fibonacci value based on n
 * 
 * @param n the n parameter
 * @return int 
 */
int fibonacci(int n)
{
    if (n == 0 || n == 1)
        return 1;
    int val1 = 1;
    int val2 = 1;
    int count = 1;
    while (count != n)
    {
        auto temp = val1 + val2;
        val1 = val2;
        val2 = temp;
        ++count;
    }
    return val2;
}

using fun_type = double (*)(double);

/**
 * @brief the fibonacci search method to find the min value
 * 
 * @param fun the target function
 * @param left the left boundary
 * @param right the right boundary
 * @param threshold the threshold
 * @param log whether output the log infomation
 * @return std::pair<double, double> the minpos and the minval 
 */
std::pair<double, double> fibonacciSearch(const fun_type &fun, double left, double right, double threshold = 1E-8, bool log = false)
{
    auto l1 = right - left;
    int iterCount = 0;
    while (1.0 / fibonacci(++iterCount) > threshold)
    {
    }
    auto l2 = fibonacci(iterCount - 1) * l1 / fibonacci(iterCount);
    if (log)
        std::cout << "the iter count is [" << iterCount << "], the l2 is [" << l2 << "]." << std::endl;

    int count = 0;
    double temp_left = right - l2, temp_right = left + l2;
    double temp_val, temp_left_val = fun(temp_left), temp_right_val = fun(temp_right);
    bool fromLeft;

    if (temp_left_val < temp_right_val)
        right = temp_right, fromLeft = false, temp_val = temp_left_val;
    else
        left = temp_left, fromLeft = true, temp_val = temp_right_val;

    while (count != iterCount)
    {
        if (log)
            std::cout << "left: " << left << ", right: " << right << std::endl;

        if (fromLeft)
        {
            temp_left = temp_right;
            temp_right = left + (right - temp_left);
            temp_left_val = temp_val;
            temp_right_val = fun(temp_right);
        }
        else
        {
            temp_right = temp_left;
            temp_left = right - (temp_right - left);
            temp_left_val = fun(temp_left);
            temp_right_val = temp_val;
        }

        if (temp_left_val < temp_right_val)
            right = temp_right, fromLeft = false, temp_val = temp_left_val;
        else
            left = temp_left, fromLeft = true, temp_val = temp_right_val;

        ++count;
    }

    double minpos = (right + left) / 2.0;
    double minval = fun(minpos);
    return {minpos, fun(minpos)};
}

int main(int argc, char const *argv[])
{
    auto target = [](double x) -> double
    {
        return x * x - 6.0 * x + 2.0;
    };
    auto res1 = fibonacciSearch(target, 0.0, 10.0, 0.0001, false);
    std::cout << "the result is [minpos: " << res1.first << ", minval: " << res1.second << "]\n";

    /**
     * @brief output
     * the result is [minpos: 2.99927, minval: -7]
     */
    return 0;
}
