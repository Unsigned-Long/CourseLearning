#pragma once

#include <map>
#include <set>
#include "../../thirdparty/include/gaussian.hpp"
#include "../../statistics/include/statistic.hpp"

namespace ns_ml
{
    template <std::size_t Dime>
    class NaiveBayes
    {
    public:
        // the type of the sample
        using sample_type = ns_ml::ClassifySample<Dime>;

        using sample_value_type = typename ns_ml::ClassifySample<Dime>::value_type;

        // the type of the sample set
        using sampleset_type = typename ns_ml::SampleSet<Dime, int>;

    public:
        /**
         * \brief Supervised learning based on Naive Bayes
         * \param set the sample set
         * \return the Decision function
         */
        static auto process(const sampleset_type &set)
        {
            // dimeIndex, class, type, P
            std::map<int, std::map<int, std::map<int, double>>> discrete;
            // diemIndex, valueCounter
            std::map<int, std::set<int>> valueCounter;
            // diemIndex, class, [mean, std-dev]
            std::map<int, std::map<int, std::pair<double, double>>> continuous;
            int dime = set.samples().front().dime();
            int setSize = set.samples().size();
            // type : std::array<FeatuerProp, DIME>
            auto props = set.properties();
            // classID, number
            std::map<int, int> classCounter;
            for (const auto &sample : set.samples())
                ++classCounter[sample.label()];
            for (int i = 0; i != dime; ++i)
            {
                if (props.at(i) == ns_ml::FeatuerProp::CONTINUOUS)
                {
                    std::map<int, std::pair<double, double>> meanStdev;
                    std::map<int, int> counter;
                    // get mean
                    for (int j = 0; j != setSize; ++j)
                    {
                        auto &sample = set.samples().at(j);
                        meanStdev[sample.label()].first += sample.valueAt(i);
                        ++counter[sample.label()];
                    }
                    for (auto &elem : meanStdev)
                        elem.second.first /= counter[elem.first];
                    // get std-dev
                    for (int j = 0; j != setSize; ++j)
                    {
                        auto &sample = set.samples().at(j);
                        meanStdev[sample.label()].second += std::pow(meanStdev[sample.label()].first - sample.valueAt(i), 2);
                    }
                    for (auto &elem : meanStdev)
                    {
                        elem.second.second /= counter[elem.first] - 1;
                        elem.second.second = std::sqrt(elem.second.second);
                        continuous[i][elem.first] = elem.second;
                    }
                }
                else
                {
                    for (int j = 0; j != setSize; ++j)
                    {
                        auto &sample = set.samples().at(j);
                        discrete[i][sample.label()][int(sample.valueAt(i))] += 1.0;
                        valueCounter[i].insert(int(sample.valueAt(i)));
                    }
                }
            }

            // dimeIndex, class, type, P
            for (auto &[diemIndex, other1] : discrete)
                for (auto &[classID, other2] : other1)
                    for (auto &[type, P] : other2)
                        P = (P + 1.0) / (classCounter[classID] + valueCounter[diemIndex].size());

            return [discrete, continuous, classCounter, setSize](const sample_value_type &val)
            {
                // classID, P
                std::map<int, double> result;
                for (const auto &[classID, num] : classCounter)
                    result[classID] = double(num + 1) / (setSize + classCounter.size());
                // for discrete featuer
                for (const auto &[dimeIndex, other1] : discrete)
                    for (const auto &[classID, other2] : other1)
                        result[classID] *= other2.at(int(val(dimeIndex)));

                //for continuous featuer
                for (const auto &[dimeIndex, other1] : continuous)
                    for (const auto &[classID, meanStd] : other1)
                        result[classID] *= ns_gaussian::Gaussian<double>::gaussian(val(dimeIndex), meanStd.first, meanStd.second);

                return std::max_element(result.cbegin(), result.cend(), [](const auto &p1, const auto &p2)
                                        { return p1.second < p2.second; })
                    ->first;
            };
        }
    };
} // namespace ns_ml
