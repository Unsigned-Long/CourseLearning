#pragma once

#include "UWBContext.h"
#include <Eigen/Dense>
#include <Eigen/Core>

namespace ns_uwb
{
    ns_point::Point3d taylorSeries(std::vector<UWBContext::DataItem>::const_iterator start,
                                   std::vector<UWBContext::DataItem>::const_iterator end);

    template <typename _Matrix_Type_>
    _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a,
                                double epsilon = std::numeric_limits<double>::epsilon())
    {
        Eigen::JacobiSVD<_Matrix_Type_> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
        return svd.matrixV() *
               (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
               svd.matrixU().adjoint();
    }
} // namespace ns_uwb