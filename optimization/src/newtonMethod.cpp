/**
 * @file newtonMethod.cpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2022-01-24
 * 
 * @copyright Copyright (c) 2022
 */

#include <iostream>
#include <array>
#include <eigen3/Eigen/Dense>

using fun_type = double (*)(const Eigen::MatrixXd &);
using firDerFun_type = Eigen::MatrixXd (*)(const Eigen::MatrixXd &);
using sedDerFun_type = Eigen::MatrixXd (*)(const Eigen::MatrixXd &);

/**
 * @brief the newton method to find the min value
 * 
 * @param fun the origin function
 * @param firDerFun the first derivative function
 * @param sedDerFun the second derivative function
 * @param initVal the initial value
 * @param threshold the threshold
 * @return std::array<double, 3> 
 */
std::array<double, 3> newtonMethod(const fun_type &fun,
                                   const firDerFun_type &firDerFun, const sedDerFun_type &sedDerFun,
                                   Eigen::MatrixXd initVal, double threshold = 1E-8)
{
    Eigen::MatrixXd newVal = initVal - sedDerFun(initVal).inverse() * firDerFun(initVal);
    while ((newVal - initVal).norm() > threshold)
    {
        initVal = newVal;
        newVal = initVal - sedDerFun(initVal).inverse() * firDerFun(initVal);
    }

    return {newVal(0, 0), newVal(1, 0), fun(newVal)};
}

int main(int argc, char const *argv[])
{
    auto fun = [](const Eigen::MatrixXd &val)
    {
        auto x1 = val(0, 0), x2 = val(1, 0);
        return 4.0 * x1 * x1 + 2.0 * x1 * x2 + 2.0 * x2 * x2 + x1 + x2;
    };
    auto firDerFun = [](const Eigen::MatrixXd &val)
    {
        auto x1 = val(0, 0), x2 = val(1, 0);
        Eigen::MatrixXd res(2, 1);
        res(0, 0) = 8.0 * x1 + 2.0 * x2 + 1.0;
        res(1, 0) = 2.0 * x1 + 4.0 * x2 + 1.0;
        return res;
    };
    auto sedDerFun = [](const Eigen::MatrixXd &val)
    {
        Eigen::MatrixXd res(2, 2);
        res(0, 0) = 8.0;
        res(0, 1) = 2.0;
        res(1, 0) = 2.0;
        res(1, 1) = 4.0;
        return res;
    };
    Eigen::MatrixXd initVal(2, 1);
    initVal(0, 0) = 0.0;
    initVal(1, 0) = 0.0;
    
    auto res = newtonMethod(fun, firDerFun, sedDerFun, initVal);
    std::cout << "the result is [minpos: {" << res[0] << ", " << res[1] << "}, minval: "
              << res[2] << "]\n";

    /**
     * @brief output
     * the result is [minpos: {-0.0714286, -0.214286}, minval: -0.142857]
     */

    return 0;
}
