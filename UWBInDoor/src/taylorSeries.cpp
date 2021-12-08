#include "taylorSeries.h"
#include <math.h>
#include <iomanip>

using namespace Eigen;

namespace ns_uwb
{
    ns_point::Point3d taylorSeries(std::vector<UWBContext::DataItem>::const_iterator start,
                                   std::vector<UWBContext::DataItem>::const_iterator end)
    {
        UWBContext::checkSolveCondition(start, end);
        ns_timer::DurationTimer<std::chrono::milliseconds> timer;
        timer.init();
        auto &center = UWBContext::getInitPosition();
        /**
         * \attention Assign the calculation result to 'params'
         */
        double params[3] = {center.x(), center.y(), center.z()};
        /**
         * \attention writing the new algorithm
         */

        Eigen::MatrixXd B(end - start, 2), L(end - start, 1), X(2, 1), D(end - start, 1), D_0(end - start, 1);
        Eigen::MatrixXd V(end - start, 1), N(end - start, end - start);
        int i = 0;
        for (auto iter = start; iter != end; ++iter)
        {
            auto range = iter->_range;
            auto id = iter->_refStationID;
            auto stationPos = (UWBContext::baseStation()).at(id);
            auto x = stationPos.x();
            auto y = stationPos.y();
            auto z = stationPos.z();
            B(i, 0) = 2 * (params[0] - x);
            B(i, 1) = 2 * (params[1] - y);
            D(i, 0) = range * range;
            D_0(i, 0) = pow(params[0] - x, 2) + pow(params[1] - y, 2);
            i++;
        }
        L = D - D_0;
        N = B.transpose() * B;
        X = pseudoInverse(N) * B.transpose() * L;
        V = B * X - L;
        params[0] += X(0, 0);
        params[1] += X(1, 0);
        while (1)
        {
            N = B.transpose() * B;
            X = pseudoInverse(N) * B.transpose() * L;
            V = B * X - L;
            params[0] += X(0, 0);
            params[1] += X(1, 0);
            if (fabs(X(0, 0)) + fabs(X(1, 0)) < 0.001)
                break;
            int n = 0;
            for (auto iter = start; iter != end; ++iter)
            {
                auto range = iter->_range;
                auto id = iter->_refStationID;
                auto stationPos = (UWBContext::baseStation()).at(id);
                auto x = stationPos.x();
                auto y = stationPos.y();
                auto z = stationPos.z();
                B(n, 0) = 2 * (params[0] - x);
                B(n, 1) = 2 * (params[1] - y);
                D(n, 0) = range * range;
                D_0(n, 0) = pow(params[0] - x, 2) + pow(params[1] - y, 2);
                n++;
            }
            L = D - D_0;
        }
        /**
         * \attention end
         */
        output(timer.lastDurStr("solve cost time"));
        center.x() = params[0];
        center.y() = params[1];
        center.z() = params[2];
        return ns_point::Point3d(params[0], params[1], params[2]);
    }
} // namespace ns_uwb