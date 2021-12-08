#include "sequential.h"
#include "eigen3/Eigen/Dense"

namespace ns_uwb
{
    ns_point::Point3d sequential(std::vector<UWBContext::DataItem>::const_iterator start,
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
        Eigen::MatrixXd B0(4, 3), l0(4, 1), x0(3, 1), Q0(3, 3);
        Eigen::MatrixXd x1(3, 1), B1(1, 3), J(3, 1);
        while (1)
        {
            int i = 0;
            for (auto iter = start; iter != start + 4; iter++)
            {
                auto range = iter->_range;
                auto id = iter->_refStationID;
                auto stationPos = (UWBContext::baseStation()).at(id);
                auto x = stationPos.x();
                auto y = stationPos.y();
                auto z = stationPos.z();
                B0(i, 0) = 2 * (params[0] - x);
                B0(i, 1) = 2 * (params[1] - y);
                B0(i, 2) = 2 * (params[2] - z);
                l0(i, 0) = range * range - (pow(params[0] - x, 2) + pow(params[1] - y, 2) + pow(params[2] - z, 2));
                i++;
            }
            x0 = (B0.transpose() * B0).inverse() * B0.transpose() * l0;
            params[0] += x0(0, 0);
            params[1] += x0(1, 0);
            params[2] += x0(2, 0);
            if ((abs(x0(0, 0)) + abs(x0(1, 0))) < 0.01)
            {
                break;
            }
        }
        double params0[3] = {params[0], params[1], params[2]};
        int i = 0;
        for (auto iter = start + 4; iter != start + 8; iter++)
        {
            auto range = iter->_range;
            auto id = iter->_refStationID;
            auto stationPos = (UWBContext::baseStation()).at(id);
            auto x = stationPos.x();
            auto y = stationPos.y();
            auto z = stationPos.z();
            B0(i, 0) = 2 * (params[0] - x);
            B0(i, 1) = 2 * (params[1] - y);
            B0(i, 2) = 2 * (params[2] - z);
            l0(i, 0) = range * range - (pow(params[0] - x, 2) + pow(params[1] - y, 2) + pow(params[2] - z, 2));
            i++;
        }
        x0 = (B0.transpose() * B0).inverse() * B0.transpose() * l0;
        params[0] = params0[0] + x0(0, 0);
        params[1] = params0[1] + x0(1, 0);
        params[2] = params0[2] + x0(2, 0);
        Q0 = (B0.transpose() * B0).inverse();
        for (auto iter = start + 8; iter != end; iter++)
        {
            auto range = iter->_range;
            auto id = iter->_refStationID;
            auto stationPos = (UWBContext::baseStation()).at(id);
            auto x = stationPos.x();
            auto y = stationPos.y();
            auto z = stationPos.z();
            B1(0, 0) = 2 * (params[0] - x);
            B1(0, 1) = 2 * (params[1] - y);
            B1(0, 2) = 2 * (params[2] - z);
            double l1 = range * range - (pow(params[0] - x, 2) + pow(params[1] - y, 2) + pow(params[2] - z, 2));
            double l_ = l1 - (B1 * x0).determinant();
            J = Q0 * B1.transpose() / (1 + (B1 * Q0 * B1.transpose()).determinant());
            x1 = x0 + J * l_;
            params[0] = params0[0] + x1(0, 0);
            params[1] = params0[1] + x1(1, 0);
            params[2] = params0[2] + x1(2, 0);
            x0 = x1;
            Q0 = Q0 - Q0 * B1.transpose() / (1 + (B1 * Q0 * B1.transpose()).determinant()) * B1 * Q0;
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