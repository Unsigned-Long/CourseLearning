#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include "eigen3/Eigen/Dense"
#include <vector>
#include "point.h"

namespace ns_res
{
    /**
     * \brief a simple function to print info to the console
     */
    void console(const std::string &str, bool upLine = true, bool downLine = true);

    /**
     * \brief a static class to init the configure files,
     *        it's a part to contain the params.
     */
    class ConfigInit
    {
    public:
        static Eigen::Matrix4d Rt_pl;

        /**
         * \brief point desc
         * [0] lower_left
         * [1] upper_left
         * [2] upper_right
         * [3] lower_right
         */
        static std::vector<ns_point::Point2d> objPoints;
        static std::vector<ns_point::Point2d> imgPoints;

        static double f, cx, cy;
        static double k1, k2, k3;
        static double p1, p2;

        // the num of the points pair
        static int itemNum;

        /**
         * \brief here are the params we need to calculate.
         */
        static Eigen::Matrix3d H;

        static Eigen::Matrix3d Rh;

        static Eigen::Matrix3d R;

        static Eigen::Vector3d t;
        // our pos refering to the world coor
        static Eigen::Vector3d C;

    public:
        static void init(const std::string &config_directory);

        static void info();

        // translate matrix from world coor to camera [or the camera position in the world coor]
        static const Eigen::Vector3d &curPosition() { return ConfigInit::C; }

        // rotation matrix from world coor to camera
        static const Eigen::Matrix3d &curPosture() { return ConfigInit::R; }

    private:
        ConfigInit() = delete;

        // read all chars in the file
        static std::string readString(std::fstream &file);

        // init the Rt[plane2world] matrix
        static void init_Rt_pl(const std::string &filename);

        // init the point pairs[obj points and img points]
        static void init_points(const std::string &filename, std::vector<ns_point::Point2d> &vec);

        // init the params of f, cx, cy, k[1, 2, 3], p[1, 2]
        static void init_inner_params(const std::string &filename);
    };

} // namespace ns_res
