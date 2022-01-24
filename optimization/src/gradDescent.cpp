#include <iostream>
#include <array>
#include <cmath>

template <std::size_t ParamNum>
using fun_type = double (*)(const std::array<double, ParamNum> &);

template <std::size_t ParamNum>
using firDerFun_type = fun_type<ParamNum>;

/**
 * @brief using gradient descent method to calculate the min value
 * 
 * @tparam ParamNum the number of params
 * @param fun the origin function
 * @param firDerFuns the first derivative function
 * @param initVals the init values
 * @param k the stride
 * @param threshold the threshold
 * @return std::pair<std::array<double, ParamNum>, double> 
 */
template <std::size_t ParamNum>
std::pair<std::array<double, ParamNum>, double>
gradientDescent(const fun_type<ParamNum> &fun,
                const std::array<firDerFun_type<ParamNum>, ParamNum> &firDerFuns,
                std::array<double, ParamNum> initVals, double k = -0.001, double threshold = 1E-6)
{
    std::array<double, ParamNum> newVals;
    std::array<double, ParamNum> grads;

    for (int i = 0; i != ParamNum; ++i)
        grads[i] = firDerFuns[i](initVals);

    for (int i = 0; i != ParamNum; ++i)
        newVals[i] = initVals[i] + k * grads[i];

    auto error = [](const std::array<double, ParamNum> &argv1,
                    const std::array<double, ParamNum> &argv2) -> double
    {
        double temp = 0.0;
        for (int i = 0; i != ParamNum; ++i)
            temp += std::abs(argv1[i] - argv2[i]);
        return temp;
    };

    while (error(initVals, newVals) > threshold)
    {
        for (int i = 0; i != ParamNum; ++i)
            initVals[i] = newVals[i];

        for (int i = 0; i != ParamNum; ++i)
            grads[i] = firDerFuns[i](initVals);

        for (int i = 0; i != ParamNum; ++i)
            newVals[i] = initVals[i] + k * grads[i];
    }
    return {newVals, fun(newVals)};
}

int main(int argc, char const *argv[])
{
    auto fun = [](const std::array<double, 2> &argvs)
    {
        return argvs[0] * argvs[0] + argvs[1] * argvs[1] + 10.0;
    };

    auto firDerFun1 = [](const std::array<double, 2> &argvs)
    {
        return 2.0 * argvs[0];
    };

    auto firDerFun2 = [](const std::array<double, 2> &argvs)
    {
        return 2.0 * argvs[1];
    };

    auto res = gradientDescent<2>(fun, {firDerFun1, firDerFun2}, {10.0, 20.0});

    std::cout << "the result is [minpos: {" << res.first[0] << ", " << res.first[1] << "}, minval: "
              << res.second << "]\n";

    /**
     * @brief output
     * the result is [minpos: {0.000166183, 0.000332365}, minval: 10]
     */

    return 0;
}
