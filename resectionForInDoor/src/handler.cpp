
#include "handler.h"

namespace ns_res
{
    bool Ceres_RemoveDistortion::operator()(const double *const undisPoint, double *out) const
    {
        auto und_x = (undisPoint[0] - ConfigInit::cx) / ConfigInit::f;
        auto und_y = (undisPoint[1] - ConfigInit::cy) / ConfigInit::f;
        auto r_2 = und_x * und_x + und_y * und_y;
        auto r_4 = r_2 * r_2;
        auto r_6 = r_4 * r_2;
        auto dk = (ConfigInit::k1 * r_2 + ConfigInit::k2 * r_4 + ConfigInit::k3 * r_6);
        auto dp_x = 2 * ConfigInit::p1 * und_y + ConfigInit::p2 * (r_2 + 2 * und_x * und_x);
        auto dp_y = ConfigInit::p1 * (r_2 + 2 * und_y * und_y) + 2 * ConfigInit::p2 * und_x;
        und_x += und_x * dk + dp_x;
        und_y += und_y * dk + dp_y;
        out[0] = this->_point->x() - (und_x * ConfigInit::f + ConfigInit::cx);
        out[1] = this->_point->y() - (und_y * ConfigInit::f + ConfigInit::cy);
        return true;
    }

    bool Ceres_CalHMatrix::operator()(const double *const h, double *out) const
    {
        auto x = this->_objPoint->x();
        auto y = this->_objPoint->y();

        auto temp_x = h[0] * x + h[1] * y + h[2] - h[6] * this->_imgPoint->x() * x - h[7] * this->_imgPoint->x() * y;
        auto temp_y = h[3] * x + h[4] * y + h[5] - h[6] * this->_imgPoint->y() * x - h[7] * this->_imgPoint->y() * y;

        out[0] = this->_imgPoint->x() - temp_x;
        out[1] = this->_imgPoint->y() - temp_y;
        return true;
    }

    void Handler::process()
    {
        removeDistortion();
        cvtImgCoor();
        calculateH();
        constructOtherMats();
        return;
    }

    void Handler::removeDistortion()
    {
        console("remove distortion", true, false);
        for (auto &imgPoint : ConfigInit::imgPoints)
        {
            double pos[2] = {imgPoint.x(), imgPoint.y()};
            ceres::Problem prob;
            ceres::CostFunction *fun =
                new ceres::NumericDiffCostFunction<Ceres_RemoveDistortion, ceres::CENTRAL, 2, 2>(new Ceres_RemoveDistortion(&imgPoint));
            prob.AddResidualBlock(fun, nullptr, pos);

            ceres::Solver::Options op;
            op.minimizer_progress_to_stdout = false;
            op.linear_solver_type = ceres::DENSE_QR;

            ceres::Solver::Summary sum;
            ceres::Solve(op, &prob, &sum);
            imgPoint.x() = pos[0];
            imgPoint.y() = pos[1];
        }
    }

    void Handler::cvtImgCoor()
    {
        console("Convert to image coordinates", true, false);
        for (auto &elem : ConfigInit::imgPoints)
        {
            elem.x() = (elem.x() - ConfigInit::cx) / ConfigInit::f;
            elem.y() = (elem.y() - ConfigInit::cy) / ConfigInit::f;
        }
        return;
    }

    void Handler::calculateH()
    {
        console("calculate the H matrix");
        double h[8] = {};
        std::fill(h, h + 8, 1.0);
        ceres::Problem prob;

        for (int i = 0; i != ConfigInit::itemNum; ++i)
        {
            ceres::CostFunction *fun =
                new ceres::NumericDiffCostFunction<Ceres_CalHMatrix, ceres::CENTRAL, 2, 8>(new Ceres_CalHMatrix(&ConfigInit::imgPoints.at(i), &ConfigInit::objPoints.at(i)));
            prob.AddResidualBlock(fun, nullptr, h);
        }

        ceres::Solver::Options op;
        op.minimizer_progress_to_stdout = false;
        op.linear_solver_type = ceres::DENSE_QR;

        ceres::Solver::Summary sum;
        ceres::Solve(op, &prob, &sum);
        for (int i = 0; i != 8; ++i)
            ConfigInit::H(i / 3, i % 3) = h[i];
        std::cout << ConfigInit::H << std::endl;

        return;
    }

    void Handler::constructOtherMats()
    {
        console("construct other mats");
        auto r1 = ConfigInit::H.col(0);
        auto r2 = ConfigInit::H.col(1);
        auto lambda = (r1.norm() + r2.norm()) / 2.0;
        r1.normalize();
        r2.normalize();
        auto r3 = r1.cross(r2);
        r3.normalize();
        r2 = r3.cross(r1);
        ConfigInit::Rh.col(0) = r1;
        ConfigInit::Rh.col(1) = r2;
        ConfigInit::Rh.col(2) = r3;
        console("Rh mat[R * R_pl]", false, true);
        std::cout << ConfigInit::Rh << std::endl;

        ConfigInit::R = ConfigInit::Rh * (ConfigInit::Rt_pl.block(0, 0, 3, 3).transpose());
        console("R mat[Rh * R_pl.inv()]");
        std::cout << ConfigInit::R << std::endl;

        ConfigInit::t = ConfigInit::H.col(2) / lambda - ConfigInit::R * ConfigInit::Rt_pl.block(0, 3, 3, 1);
        ConfigInit::C = -ConfigInit::R.transpose() * ConfigInit::t;
        console("t mat[translate from plane Coor to camera Coor]");
        std::cout << ConfigInit::C << std::endl;

        return;
    }

} // namespace ns_res