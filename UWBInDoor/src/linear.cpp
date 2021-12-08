#include "linear.h"
#include "eigen3/Eigen/Dense"
#include <math.h>

namespace ns_uwb
{
    ns_point::Point3d linear(std::vector<UWBContext::DataItem>::const_iterator start,
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
        auto len = end - start;
        auto &station = UWBContext::baseStation();
        double z0 = center.z();
        Eigen::MatrixXd A(len - 1, 2), l(len - 1, 1), v(len - 1, 1), par(2, 1);
        int iter_count = 0;
        Eigen::MatrixXd delta_pa = Eigen::MatrixXd::Zero(len - 1, 1);
        par(0, 0) = center.x();
        par(1, 0) = center.y();
        auto id_1 = start->_refStationID;
        double range_1 = start->_range;
        double x1 = station.at(id_1).x();
        double y1 = station.at(id_1).y();
        double z1 = station.at(id_1).z();
        while (iter_count == 0 || delta_pa.norm() > 1e-2)
        {
            for (int i = 1; i < len; i++)
            {
                int id = (start + i)->_refStationID;
                double range_i = (start + i)->_range;
                double xi = station.at(id).x();
                double yi = station.at(id).y();
                double zi = station.at(id).z();
                double diffdis_Square = std::pow(range_1, 2) - (z1 - z0) * (z1 - z0) - (std::pow(range_i, 2) - (zi - z0) * (zi - z0));
                A(i - 1, 0) = 2 * (xi - x1);
                A(i - 1, 1) = 2 * (yi - y1);
                l(i - 1, 0) = diffdis_Square - 2 * (xi - x1) * par(0, 0) - 2 * (yi - y1) * par(1, 0) + (xi * xi + yi * yi) - (x1 * x1 + y1 * y1); //par(0,0)=initial x0, par(1,0)=initial y0
            }
            delta_pa = (A.transpose() * A).inverse() * A.transpose() * l;
            par += delta_pa;
            v = A * delta_pa - l;
            iter_count++;
        }

        params[0] = par(0);
        params[1] = par(1);
        params[2] = z0;
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