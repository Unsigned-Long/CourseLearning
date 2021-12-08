#pragma once

#include "../../sample/include/sampleset.hpp"
#include "ceres/ceres.h"
#include <array>
#include <algorithm>

namespace ns_ml
{
    template <std::size_t Dime, std::size_t ParamsNum>
    struct RegressCostFunction
    {
    public:
        // the type of the sample
        using sample_type = ns_ml::RegressSample<Dime>;

        using sample_value_type = typename ns_ml::RegressSample<Dime>::value_type;

        // the type of the target function
        using target_fun_type = double (*)(const sample_value_type &sampleValue,
                                           const double params[ParamsNum]);

    private:
        // the members
        const sample_type *_sample;

        // the target function
        target_fun_type _targetFun;

    public:
        // the constructor
        RegressCostFunction(const sample_type &sample, const target_fun_type &targetFun)
            : _sample(&sample), _targetFun(targetFun) {}

        // overload for operator '()'
        bool operator()(const double *const params, double *out) const
        {
            out[0] = this->_sample->label() - this->_targetFun(this->_sample->values(), params);
            return true;
        }
    };

    template <std::size_t Dime, std::size_t ParamsNum>
    class Regression
    {
    public:
        // the type of the sample
        using sample_type = ns_ml::RegressSample<Dime>;

        using sample_value_type = typename ns_ml::RegressSample<Dime>::value_type;

        // the type of the sample set
        using sampleset_type = ns_ml::SampleSet<sample_type>;

        // the type of the target function
        using target_fun_type = double (*)(const sample_value_type &sampleValue,
                                           const double params[ParamsNum]);

    private:
        Regression() = delete;

    public:
        /**
         * \brief the main function to do the regression 
         * \param set the sample set
         * \param targetFun the target defined by the user
         * \return the hypothesis function
         */
        static auto process(const sampleset_type &set,
                            const target_fun_type &targetFun,
                            double param[ParamsNum] = nullptr)
        {
            // init the params
            double params[ParamsNum];
            if (param != nullptr)
                std::copy_n(param, ParamsNum, params);
            else
                std::fill(params, params + ParamsNum, 0.0);

            // construct the problem
            ceres::Problem prob;
            for (const auto &elem : set.samples())
            {
                auto fun = new ceres::NumericDiffCostFunction<RegressCostFunction<Dime, ParamsNum>, ceres::CENTRAL, 1, ParamsNum>(new RegressCostFunction<Dime, ParamsNum>(elem, targetFun));
                prob.AddResidualBlock(fun, nullptr, params);
            }
            // set options
            ceres::Solver::Options op;
            op.minimizer_progress_to_stdout = false;
            op.linear_solver_type = ceres::DENSE_QR;
            ceres::Solver::Summary sum;
            // solving problem
            ceres::Solve(op, &prob, &sum);
            // assign for param
            if (param != nullptr)
                std::copy_n(params, ParamsNum, param);
            // return the hypothesis function
            return [targetFun, params](const sample_value_type &sample) -> double
            {
                return targetFun(sample, params);
            };
        }
    };
} // namespace ns_ml
