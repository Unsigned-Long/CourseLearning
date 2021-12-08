#pragma once

#include <iostream>
#include "eigen3/Eigen/Dense"

namespace ns_ml
{
#pragma region Gradient Descent
    template <std::size_t Dime>
    class GradDesc
    {
    public:
        using grad_type = Eigen::Vector<double, Dime>;
        using argv_type = grad_type;
        using grad_function = grad_type (*)(const argv_type &argv);

    private:
        GradDesc() = delete;

    public:
        /**
         * \brief function to get the min value with gradient descent method
         * \param grad the function to get the grad
         * \param initArgv the init arguement values
         * \param learningRate the rate of the learning
         * \param output whether output to the console
         * \param thresold the thresold to stop the process
         * \return the value where we get the min value
         */ 
        static argv_type process(const grad_function &grad, const argv_type &initArgv, double learningRate,
                                 bool output = false, double thresold = 1E-10)
        {
            argv_type curArgv = initArgv;
            int count = 0;
            while (true)
            {
                auto newArgv = curArgv - learningRate * grad(curArgv);
                auto diff = (newArgv - curArgv).norm();
                curArgv = newArgv;
                if (output)
                {
                    std::string str("cur argv {");
                    for (int i = 0; i != curArgv.rows(); ++i)
                    {
                        str += std::to_string(curArgv(i)) + ',';
                    }
                    str.pop_back();
                    str += "}\n";
                    std::cout << str;
                }
                ++count;
                if (diff < thresold)
                    break;
            }
            if (output)
                std::cout << "iter count {" << count << "}\n";
            return curArgv;
        }
    };
#pragma endregion
} // namespace ns_ml
