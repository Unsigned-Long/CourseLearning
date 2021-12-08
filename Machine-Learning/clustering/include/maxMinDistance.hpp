#pragma once

#include "../../sample/include/sampleset.hpp"
#include "../../statistics/include/statistic.hpp"
#include <map>
#include <algorithm>

namespace ns_ml
{
    template <std::size_t Dime>
    class MaxMinDistance
    {
    public:
        // the type of the sample
        using sample_type = ns_ml::ClassifySample<Dime>;

        using sample_value_type = typename ns_ml::ClassifySample<Dime>::value_type;

        // the type of the sample set
        using sampleset_type = typename ns_ml::SampleSet<Dime, int>;

    private:
        MaxMinDistance() = delete;

    public:
        /**
         * \brief Based on the given number of categories, the clusters are roughly divided.
         *        This function marks the class to which each sample belongs and the name of each class.
         * \param set the sample set[ClassifySample<Dime>]
         * \param classNum the number of the classes
         * \return the classCenters and the Decision function
         *         std::pair(classCenter,decisionFun)
         */
        static auto process(sampleset_type &set, std::size_t classNum)
        {
            // find the first center
            auto center = std::min_element(set.samples().begin(), set.samples().end(),
                                           [](const sample_type &s1, const sample_type &s2)
                                           {
                                               return s1.valueAt(0) < s2.valueAt(0);
                                           });
            // [classType, center]
            std::map<int, sample_value_type> classCenters;
            // record the first center
            classCenters[0] = center->values();
            // create center map
            int classCount = 0;
            std::map<int, sample_value_type> centers;
            centers[classCount++] = center->values();
            // create the vector to record the distance
            std::size_t sampleSize = set.samples().size();
            std::vector<std::map<int, double>> distance(sampleSize, std::map<int, double>());
            // find new centers
            for (int i = 0; i != classNum - 1; ++i)
            {
                auto lastCenter = *(--centers.end());
                auto lastCenterClass = lastCenter.first;
                auto lastCenterVec = lastCenter.second;
                std::vector<double> minDis(sampleSize, 0.0);
                for (int j = 0; j != sampleSize; ++j)
                {
                    auto &sample = set.samples().at(j);
                    auto &curMap = distance.at(j);
                    curMap[lastCenterClass] = ns_ml::distance(sample.values(), lastCenterVec);
                    // find min distance from cur sample to each center
                    minDis.at(j) = std::min_element(curMap.cbegin(), curMap.cend(), [](const auto &p1, const auto &p2)
                                                    { return p1.second < p2.second; })
                                       ->second;
                }

                // find the max distance from samples' min distance
                auto maxDisIter = std::max_element(minDis.cbegin(), minDis.cend());
                auto curCenterVec = set.samples().at(maxDisIter - minDis.cbegin()).values();
                auto curCenterClass = classCount++;
                // record new center
                centers[curCenterClass] = curCenterVec;
                classCenters[curCenterClass] = set.samples().at((maxDisIter - minDis.cbegin())).values();
            }
            auto lastCenter = *(--centers.end());
            auto lastCenterClass = lastCenter.first;
            auto lastCenterVec = lastCenter.second;
            for (int j = 0; j != sampleSize; ++j)
            {
                auto &sample = set.samples().at(j);
                auto &curMap = distance.at(j);
                curMap[lastCenterClass] = ns_ml::distance(sample.values(), lastCenterVec);
                auto minIter = std::min_element(curMap.cbegin(), curMap.cend(), [](const auto &p1, const auto &p2)
                                                { return p1.second < p2.second; });
                // mark each sample according the min distance
                sample.label() = minIter->first;
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
