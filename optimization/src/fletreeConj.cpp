/**
 * @file fletreeConj.cpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2022-01-25
 * 
 * @copyright Copyright (c) 2022
 */

#include <iostream>
#include <eigen3/Eigen/Dense>

using fun_type = double (*)(const Eigen::Vector2d &argvs);
using firDerFun_type = Eigen::Vector2d (*)(const Eigen::Vector2d &argvs);

/**
 * @brief use fletcher-Reeves conjugate gradient algorithm to calculate the min value
 * 
 * @param fun the target function
 * @param firDerFun the first derivative function
 * @param initVals the start pos
 * @param lambda the params
 * @param threshold the threshold
 * @return std::pair<Eigen::Vector2d, double> 
 */
std::pair<Eigen::Vector2d, double> fletcherReeves(const fun_type &fun,
                                                  const firDerFun_type &firDerFun,
                                                  Eigen::Vector2d &initVals,
                                                  double lambda = 0.01,
                                                  double threshold = 1E-8)
{
    Eigen::Vector2d lastGrad = firDerFun(initVals);
    Eigen::Vector2d lastV;
    Eigen::Vector2d curV = -lastGrad;
    Eigen::Vector2d newVals = initVals + lambda * curV;
    
    while ((initVals - newVals).norm() > threshold)
    {
        initVals = newVals;
        lastV = curV;

        Eigen::Vector2d curGrad = firDerFun(newVals);
        curV = -curGrad + (curGrad.norm()) / (lastGrad.norm()) * lastV;
        newVals = initVals + lambda * curV;
    }
    return {newVals, fun(newVals)};
}

int main(int argc, char const *argv[])
{
    auto fun = [](const Eigen::Vector2d &argvs) -> double
    {
        Eigen::Matrix2d A;
        A(0, 0) = A(0, 1) = A(1, 0) = 1.0;
        A(1, 1) = 2;
        return 0.5 * argvs.transpose() * A * argvs;
    };

    auto firDerFun = [](const Eigen::Vector2d &argvs) -> Eigen::Vector2d
    {
        Eigen::Vector2d grad;
        grad(0, 0) = argvs(0, 0) + argvs(1, 0);
        grad(1, 0) = argvs(0, 0) + 2.0 * argvs(1, 0);
        return grad;
    };
    Eigen::Vector2d initVals;
    initVals(0, 0) = 10.0;
    initVals(1, 0) = -5.0;
    auto res = fletcherReeves(fun, firDerFun, initVals);

    std::cout << "the result is [minpos: {" << res.first(0, 0) << ", " << res.first(1, 0) << "}, minval: "
              << res.second << "]\n";

    /**
     * @brief output
     * the result is [minpos: {2.21437e-06, -1.36856e-06}, minval: 1.29417e-12]
     */
    return 0;
}
