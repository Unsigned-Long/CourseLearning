#pragma once

#include "../../sample/include/sampleset.hpp"
#include "../../statistics/include/statistic.hpp"
#include <iostream>

namespace ns_ml
{
    template <std::size_t Dime, std::size_t resDime, typename LabelType>
    class PCA
    {
    public:
        using origin_sampleset_type = ns_ml::SampleSet<Dime, LabelType>;

        using origin_sample_type = typename origin_sampleset_type::sample_type;

        using res_sampleset_type = ns_ml::SampleSet<resDime, LabelType>;

        using res_sample_type = typename res_sampleset_type::sample_type;

    private:
        PCA() = delete;

    public:
        /**
         * \brief Principal component analysis is one of the most widely used data dimensionality reduction algorithms
         * \param set the sample set
         * \return the result sample set
         */
        static auto process(origin_sampleset_type set)
        {
            // step 1 : calculate the mean vector
            auto mean = ns_ml::mean(set);
            // step 2 : De averaging
            for (auto &elem : set.samples())
                elem.values() -= mean;
            // setp 3 : Finding eigenvalues and eigenvectors of covariance matrix
            auto size = set.samples().size();
            Eigen::MatrixXd mat(Dime, size);
            for (int c = 0; c != size; ++c)
                for (int r = 0; r != Dime; ++r)
                    mat(r, c) = set.samples().at(c).valueAt(r);
            Eigen::MatrixXd covMat = (1.0 / size) * mat * mat.transpose();
            Eigen::EigenSolver<decltype(covMat)> eigen_solver(covMat);
            auto values = eigen_solver.eigenvalues();
            auto vectors = eigen_solver.eigenvectors();
            std::vector<std::pair<double, Eigen::Vector<double, Dime>>> record;
            for (int i = 0; i != Dime; ++i)
            {
                auto val = values(i).real();
                Eigen::Vector<double, Dime> vec;
                for (int j = 0; j != Dime; ++j)
                    vec(j) = vectors(j, i).real();
                record.push_back(std::make_pair(val, vec));
            }
            // step 4 : sort the vaules and vectors
            std::sort(record.begin(), record.end(), [](const auto &elem1, const auto &elem2)
                      { return elem1.first > elem2.first; });
            // construct the transport matrix
            Eigen::MatrixXd trans(resDime, Dime);
            for (int i = 0; i != resDime; ++i)
                for (int j = 0; j != Dime; ++j)
                    trans(i, j) = record.at(i).second(j);
            // step 5 : get the result sample set
            Eigen::MatrixXd resultMat = trans * mat;
            res_sampleset_type resuleSet;
            for (int i = 0; i != size; ++i)
            {
                res_sample_type resSample;
                auto &orgSample = set.samples().at(i);
                resSample.label() = orgSample.label();
                for (int j = 0; j != resDime; ++j)
                    resSample.valueAt(j) = resultMat(j, i);
                resuleSet.samples().push_back(resSample);
            }
            return resuleSet;
        }
    };
} // namespace ns_ml
