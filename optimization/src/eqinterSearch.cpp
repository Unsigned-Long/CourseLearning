#include <iostream>

using fun_type = double (*)(double);

/**
 * @brief the equal interval search method to find the min value, using three points
 * 
 * @param fun the target function
 * @param left the left boundary
 * @param right the right boundary
 * @param log whether output the log infomation
 * @param threshold the threshold
 * @return std::pair<double, double> the minpos and the minval 
 */
std::pair<double, double> equalIntervalSearch3(const fun_type &fun, double left, double right, bool log = false, double threshold = 1E-8)
{
    while (std::abs(right - left) > threshold)
    {
        if (log)
            std::cout << "left: " << left << ", right: " << right << std::endl;

        auto temp_left = left + 0.33 * (right - left);
        auto temp_right = left + 0.66 * (right - left);

        if (fun(temp_left) < fun(temp_right))
            right = temp_right;
        else
            left = temp_left;
    }

    double minpos = (right + left) / 2.0;
    double minval = fun(minpos);
    return {minpos, fun(minpos)};
}

/**
 * @brief the equal interval search method to find the min value, using four points
 * 
 * @param fun the target function
 * @param left the left boundary
 * @param right the right boundary
 * @param log whether output the log infomation
 * @param threshold the threshold
 * @return std::pair<double, double> the minpos and the minval 
 */
std::pair<double, double> equalIntervalSearch4(const fun_type &fun, double left, double right, bool log = false, double threshold = 1E-8)
{
    auto temp_left = left + 0.25 * (right - left);
    auto temp_middle = left + 0.5 * (right - left);
    auto temp_right = left + 0.75 * (right - left);

    auto temp_left_val = fun(temp_left), temp_middle_val = fun(temp_middle), temp_right_val = fun(temp_right);

    if (temp_left_val < temp_middle_val && temp_left_val < temp_right_val)
        right = temp_middle, temp_middle_val = temp_left_val, temp_middle = temp_left;
    else if (temp_middle_val < temp_left_val && temp_middle_val < temp_right_val)
        left = temp_left, right = temp_right;
    else
        left = temp_middle, temp_middle_val = temp_right_val, temp_middle = temp_right;

    while (std::abs(right - left) > threshold)
    {
        if (log)
            std::cout << "left: " << left << ", right: " << right << std::endl;

        temp_left = left + 0.25 * (right - left);
        temp_right = left + 0.75 * (right - left);

        temp_left_val = fun(temp_left), temp_right_val = fun(temp_right);

        if (temp_left_val < temp_middle_val && temp_left_val < temp_right_val)
            right = temp_middle, temp_middle_val = temp_left_val, temp_middle = temp_left;
        else if (temp_middle_val < temp_left_val && temp_middle_val < temp_right_val)
            left = temp_left, right = temp_right;
        else
            left = temp_middle, temp_middle_val = temp_right_val, temp_middle = temp_right;
    }

    double minpos = (right + left) / 2.0;
    double minval = fun(minpos);
    return {minpos, fun(minpos)};
}

int main(int argc, char const *argv[])
{
    auto target = [](double x) -> double
    {
        return 8.0 * x * x * x - 2.0 * x * x - 7.0 * x + 3.0;
    };
    auto res1 = equalIntervalSearch3(target, 0.0, 1.0);
    std::cout << "the result is [minpos: " << res1.first << ", minval: " << res1.second << "]\n";

    auto res2 = equalIntervalSearch4(target, 0.0, 1.0);
    std::cout << "the result is [minpos: " << res2.first << ", minval: " << res2.second << "]\n";

    /**
     * @brief output
     * the result is [minpos: 0.629787, minval: -0.203425]
     * the result is [minpos: 0.629787, minval: -0.203425]
     */
    return 0;
}
