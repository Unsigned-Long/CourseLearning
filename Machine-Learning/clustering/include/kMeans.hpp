#pragma once

#include "maxMinDistance.hpp"

namespace ns_ml
{
    template <std::size_t Dime>
    class KMeans
    {
    public:
        // the type of the sample
        using sample_type = ns_ml::ClassifySample<Dime>;

        using sample_value_type = typename ns_ml::ClassifySample<Dime>::value_type;

        // the type of the sample set
        using sampleset_type = typename ns_ml::SampleSet<Dime, int>;

    private:
        KMeans() = delete;

    public:
        /**
         * \brief Based on the max-min distance algorithm, the clusters are divided.
         * \param set the sample set[ClassifySample<Dime>]
         * \param classNum the number of the classes
         * \param thresold the condition to break
         * \return the classCenters and the Decision function
         *         std::pair(classCenter,decisionFun)
         */
        static auto process(sampleset_type &set, std::size_t classNum, double thresold = 1E-3)
        {
            // do the max-min distance for the initial value
            auto [maxmin, h] = ns_ml::MaxMinDistance<Dime>::process(set, classNum);
            // constuct the class center[id, center vector]
            std::map<int, sample_value_type> classCenters;
            std::map<int, int> count;
            for (const auto &[key, value] : maxmin)
            {
                classCenters[key] = sample_value_type::Zero();
                count[key] = 0;
            }
            // calculate the gravity center
            for (const auto &sample : set.samples())
            {
                classCenters[sample.label()] += sample.values();
                ++count[sample.label()];
            }
            for (auto &[key, value] : classCenters)
                value /= count[key];

            double E = 0.0;
            for (const auto &sample : set.samples())
                E += ns_ml::distance(sample.values(), classCenters[sample.label()]);

            while (true)
            {
                for (auto &sample : set.samples())
                {
                    const auto &val = sample.values();
                    auto &label = sample.label();
                    double minDis = ns_ml::distance(val, classCenters.cbegin()->second);
                    int minClass = classCenters.cbegin()->first;
                    for (auto iter = ++classCenters.cbegin(); iter != classCenters.cend(); ++iter)
                    {
                        auto dis = ns_ml::distance(val, iter->second);
                        if (dis < minDis)
                        {
                            minDis = dis;
                            minClass = iter->first;
                        }
                    }
                    label = minClass;
                }
                // calculate the gravity center
                for (const auto &[key, value] : maxmin)
                {
                    classCenters[key] = sample_value_type::Zero();
                    count[key] = 0;
                }
                for (const auto &sample : set.samples())
                {
                    classCenters[sample.label()] += sample.values();
                    ++count[sample.label()];
                }
                for (auto &[key, value] : classCenters)
                    value /= count[key];

                // calculate the new E
                double new_E = 0.0;
                for (const auto &sample : set.samples())
                    new_E += ns_ml::distance(sample.values(), classCenters[sample.label()]);
                auto change = std::abs(E - new_E);
                if (change < thresold)
                    break;
                E = new_E;
            }
            return std::make_pair(classCenters, [classCenters](const sample_value_type &sampleValue)
                                  {
                                      double minDis = ns_ml::distance(sampleValue, classCenters.cbegin()->second);
                                      int minClass = classCenters.cbegin()->first;
                                      for (auto iter = ++classCenters.cbegin(); iter != classCenters.cend(); ++iter)
                                      {
                                          auto dis = ns_ml::distance(sampleValue, iter->second);
                                          if (dis < minDis)
                                          {
                                              minDis = dis;
                                              minClass = iter->first;
                                          }
                                      }
                                      return minClass;
                                  });
        }
    };
} // namespace ns_ml
