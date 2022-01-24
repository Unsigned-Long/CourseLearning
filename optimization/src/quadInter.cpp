/**
 * @file quadInter.cpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2022-01-24
 * 
 * @copyright Copyright (c) 2022
 */

#include <iostream>
#include <cmath>

using fun_type = double (*)(double);

/**
 * @brief the quadratic interpolation method to find the min value
 * 
 * @param fun the target function
 * @param start the start pos to calculate
 * @param v the direction
 * @return std::pair<double, double> 
 */
std::pair<double, double> quadraticInterpolation(const fun_type &fun, double start, double v)
{
    auto g = [fun, start, v](double lambda)
    { return fun(start + lambda * v); };

    auto g0 = g(0), g1 = g(1);
    double a = 0.0, b = 1.0, c = 2.0;

    if (g1 > g0)
    {
        double lambda = 2.0;
        while (g(lambda) > g0)
        {
            lambda *= 0.5;
        }
        a = 0, b = lambda, c = 2.0 * lambda;
    }
    else
    {
        while (g(c) < g(b))
        {
            a = b, b = c, c *= 2.0;
        }
    }
    double ga = g(a), gb = g(b), gc = g(c);
    double lambda_hat = 0.5 * (ga * (c * c - b * b) + gb * (a * a - c * c) + gc * (b * b - a * a)) /
                        (ga * (c - b) + gb * (a - c) + gc * (b - a));
    double minpos;
    if (g(lambda_hat) < gb)
        minpos = lambda_hat;
    else
        minpos = b;
    double minval = fun(minpos);
    return {minpos, fun(minpos)};
}

int main(int argc, char const *argv[])
{
    auto target1 = [](double x) -> double
    {
        return 8.0 * x * x * x - 2.0 * x * x - 7.0 * x + 3.0;
    };
    auto res1 = quadraticInterpolation(target1, 0.0, 1.0);
    std::cout << "the result is [minpos: " << res1.first << ", minval: " << res1.second << "]\n";

    auto target2 = [](double x) -> double
    {
        return std::pow(M_E, x) - 5.0 * x;
    };
    auto res2 = quadraticInterpolation(target2, 1.0, 1.0);
    std::cout << "the result is [minpos: " << res2.first << ", minval: " << res2.second << "]\n";

    /**
     * @brief output
     * the result is [minpos: 0.522727, minval: -0.0629226]
     * the result is [minpos: 0.541021, minval: -0.987347]
     */

    return 0;
}
