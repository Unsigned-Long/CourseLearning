#include "ceresSolver.h"

namespace ns_uwb
{

    ns_point::Point3d ceresPosition(std::vector<UWBContext::DataItem>::const_iterator start,
                                    std::vector<UWBContext::DataItem>::const_iterator end)
    {
        UWBContext::checkSolveCondition(start, end);
        ns_timer::DurationTimer<std::chrono::milliseconds> timer;
        timer.init();
        auto &center = UWBContext::getInitPosition();
        double params[3] = {0.0, 0.0, 0.0};
        ceres::Problem prob;
        auto iter = start;
        while (iter != end)
        {
            ceres::CostFunction *fun = new ceres::AutoDiffCostFunction<CeresPosition, 1, 3>(new CeresPosition(&(*iter)));
            prob.AddResidualBlock(fun, nullptr, params);
            ++iter;
        }
        ceres::Solver::Options op;
        op.gradient_tolerance = 1E-10;
        op.function_tolerance = 1E-10;
        op.minimizer_progress_to_stdout = false;
        op.linear_solver_type = ceres::DENSE_QR;

        ceres::Solver::Summary sum;
        ceres::Solve(op, &prob, &sum);
        output(timer.lastDurStr("solve cost time"));
        center.x() = params[0];
        center.y() = params[1];
        center.z() = params[2];
        return ns_point::Point3d(params[0], params[1], params[2]);
    }
} // namespace ns_uwb