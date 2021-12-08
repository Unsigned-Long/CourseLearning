#include "newtonLS.h"
#include "eigen3/Eigen/Dense"
#include <math.h>

namespace ns_uwb
{
    ns_point::Point3d newtonLS(std::vector<UWBContext::DataItem>::const_iterator start,
                               std::vector<UWBContext::DataItem>::const_iterator end)
    {
        UWBContext::checkSolveCondition(start, end);
        ns_timer::DurationTimer<std::chrono::milliseconds> timer;
        timer.init();
        auto &cen = UWBContext::getInitPosition();
        /**
         * \attention Assign the calculation result to 'params'
         */
        double params[3] = {cen.x(), cen.y(), cen.z()};
        /**
         * \attention writing the new algorithm
         */
        auto &station = UWBContext::baseStation();
        auto len = end - start;
        Eigen::MatrixXd l(len, 1), v(len, 1), vec_pa(2, 1);                                                                                  //l=inconsistent value,vec_pa=vectorization of parameters(cricle point coors x,y,z)
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(len, 2), vec_pa_old = Eigen::MatrixXd::Ones(2, 1), delta_pa = Eigen::MatrixXd::Zero(2, 1); //A=coefficient matrix
        vec_pa(0, 0) = cen.x();
        vec_pa(1, 0) = cen.y();
        double z0 = cen.z();
        int iter_count = 0;
        while ((vec_pa - vec_pa_old).norm() > 1e-4)
        {
            vec_pa_old = vec_pa;
            double x0 = vec_pa(0, 0);
            double y0 = vec_pa(1, 0);
            double z0 = cen.z();
            for (int i = 0; i < len; i++)
            {
                int id = (start + i)->_refStationID;
                double xi = station.at(id).x();
                double yi = station.at(id).y();
                double zi = station.at(id).z();
                double plane_dis = std::sqrt(std::pow((start + i)->_range, 2) - (z0 - zi) * (z0 - zi));
                double coors_dis = std::sqrt((xi - x0) * (xi - x0) + (yi - y0) * (yi - y0));
                l(i, 0) = plane_dis - coors_dis;
                A(i, 0) = (x0 - xi) / coors_dis;
                A(i, 1) = (y0 - yi) / coors_dis;
            }
            if (iter_count == 0)
                v = -l;
            //gradient vector
            Eigen::MatrixXd g = 2 * A.transpose() * v;
            Eigen::MatrixXd H = 2 * A.transpose() * A;
            //delta_pa = ((A.transpose()*A).inverse())*A.transpose()*l;
            delta_pa = -H.inverse() * g; //correction of vec_pa
            v = A * delta_pa - l;
            vec_pa(0, 0) += delta_pa(0, 0);
            vec_pa(1, 0) += delta_pa(1, 0);
        }
        //iteration ending
        for (int k = 0; k < 2; k++)
            params[k] = vec_pa(k, 0);

        /**
         * \attention end
         */
        output(timer.lastDurStr("solve cost time"));
        cen.x() = params[0];
        cen.y() = params[1];
        cen.z() = params[2];
        return ns_point::Point3d(params[0], params[1], params[2]);
    }
} // namespace ns_uwb