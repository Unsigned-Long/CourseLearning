#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include "surpharmonic.h"

namespace ns_pygeo
{
#pragma region class SurPharmonic

    void SurPharmonic::harmonic(int index_n, int order_m, float min, float max,
                                SurPharmonic::tri_fun tri_fun, const ns_clp::Color::ColorType &colorModel,
                                int classify)
    {
        // create the pointcloud of pcl::PointXYZRGB to record the points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointXYZRGB point;
        // the viewer to display the pointcloud
        pcl::visualization::CloudViewer viewer("win");
        // create points from theta[-90.0,90.0], lambda[0.0,360.0]
        for (float theta = -90.0; theta <= 90.0; theta += 0.5)
        {
            for (float lambda = 0.0; lambda <= 360.0; lambda += 0.5)
            {
                auto theta_radian = theta * ns_pygeo::params::PI / 180.0;
                auto lambda_radian = lambda * ns_pygeo::params::PI / 180.0;
                // the value calculated by Legendre function
                auto val = ns_pygeo::Legendre::forwardColumn(index_n, order_m, (theta + 90.0) * ns_pygeo::params::PI / 180.0) * tri_fun(order_m * lambda_radian);
                auto r = 10 + val;
                point.x = r * std::cos(theta_radian) * std::cos(lambda_radian);
                point.y = r * std::cos(theta_radian) * std::sin(lambda_radian);
                point.z = r * std::sin(theta_radian);
                auto [red, g, b] = ns_clp::ColorPrj::project(val, min, max, false, classify, colorModel);
                point.r = red;
                point.g = g;
                point.b = b;
                cloud->push_back(point);
            }
        }
        // diplay the point cloud while the viewer was not stopped
        viewer.showCloud(cloud);
        while (!viewer.wasStopped())
        {
        }
        return;
    }

    void SurPharmonic::harmonic_Rnm(int index_n, int order_m,
                                    const ns_clp::Color::ColorType &colorModel,
                                    int classify, float min, float max)
    {
        // the R_nm with std::cos() function
        return SurPharmonic::harmonic(index_n, order_m, min, max, std::cos, colorModel, classify);
    }

    void SurPharmonic::harmonic_Rnm(const IndexOrder &i,
                                    const ns_clp::Color::ColorType &colorModel,
                                    int classify, float min, float max)
    {
        return SurPharmonic::harmonic(i._n, i._m, min, max, std::cos, colorModel, classify);
    }

    void SurPharmonic::harmonic_Snm(int index_n, int order_m,
                                    const ns_clp::Color::ColorType &colorModel,
                                    int classify, float min, float max)
    {
        // the R_nm with std::sin() function
        return SurPharmonic::harmonic(index_n, order_m, min, max, std::sin, colorModel, classify);
    }

    void SurPharmonic::harmonic_Snm(const IndexOrder &i,
                                    const ns_clp::Color::ColorType &colorModel,
                                    int classify, float min, float max)
    {
        return SurPharmonic::harmonic(i._n, i._m, min, max, std::sin, colorModel, classify);
    }

#pragma endregion

} // namespace ns_ns_pygeo
