#pragma once

#include "../../sample/include/sampleset.hpp"

namespace ns_ml
{
#pragma region mean
    template <std::size_t Dime, typename LabelType>
    auto mean(const ns_ml::SampleSet<Dime, LabelType> &set)
    {
        auto size = set.samples().size();
        using sample_value_type = typename ns_ml::SampleSet<Dime, LabelType>::sample_type::value_type;
        sample_value_type mean = sample_value_type::Zero();
        for (const auto &sample : set.samples())
            mean += sample.values();
        mean /= size;
        return mean;
    }

    auto mean(const Eigen::VectorXd &vec)
    {
        double mean = 0.0;
        for (int i = 0; i != vec.rows(); ++i)
            mean += vec(i);
        mean /= vec.rows();
        return mean;
    }
#pragma endregion

#pragma region variance
    template <std::size_t Dime, typename LabelType>
    auto variance(const ns_ml::SampleSet<Dime, LabelType> &set)
    {
        auto m = mean(set);
        using sample_value_type = typename ns_ml::SampleSet<Dime, LabelType>::sample_type::value_type;
        sample_value_type var = sample_value_type::Zero();
        for (const auto &sample : set.samples())
        {
            var += decltype(var)(Eigen::pow((sample.values() - m).array(), 2));
        }
        var /= set.samples().size();
        return var;
    }

    template <std::size_t Dime, typename LabelType>
    auto standardDev(const ns_ml::SampleSet<Dime, LabelType> &set)
    {
        auto var = variance(set);
        return decltype(var)(Eigen::sqrt(var.array()));
    }

    auto variance(const Eigen::VectorXd &vec)
    {
        double m = mean(vec);
        double var = 0.0;
        for (int i = 0; i != vec.rows(); ++i)
            var += std::pow(vec(i) - m, 2);
        var /= vec.rows();
        return var;
    }

    auto standardDev(const Eigen::VectorXd &vec)
    {
        return std::sqrt(variance(vec));
    }

#pragma endregion

#pragma region distance
    auto distance(const Eigen::VectorXd &vec1, const Eigen::VectorXd &vec2)
    {
        Eigen::VectorXd dis = vec1 - vec2;
        dis = decltype(dis)(Eigen::pow(dis.array(), 2));
        return std::sqrt(dis.sum());
    }
#pragma endregion

#pragma region Performance metrics

    /**
     * \brief Used to measure the performance of regression results
     */
    template <std::size_t Dime, typename Hypothesis>
    auto meanSquaredError(const ns_ml::SampleSet<Dime, double> &set, const Hypothesis &hypothesis)
    {
        double result = 0.0;
        for (const auto &sample : set.samples())
            result += std::pow(hypothesis(sample.values()) - sample.label(), 2);
        result /= set.samples().size();
        return result;
    }

    /**
     * \brief Used to measure the performance of classification results
     */
    template <std::size_t Dime, typename Hypothesis>
    auto errorRate(const ns_ml::SampleSet<Dime, int> &set, const Hypothesis &hypothesis)
    {
        double result = 0.0;
        for (const auto &sample : set.samples())
            if (hypothesis(sample.values()) != sample.label())
                result += 1.0;
        result /= set.samples().size();
        return result;
    }

    /**
     * \brief Used to measure the performance of classification results
     */
    template <std::size_t Dime, typename Hypothesis>
    auto correctRate(const ns_ml::SampleSet<Dime, int> &set, const Hypothesis &hypothesis)
    {
        return 1.0 - errorRate(set, hypothesis);
    }

#pragma endregion

} // namespace ns_ml
