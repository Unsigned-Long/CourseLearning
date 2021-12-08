#pragma once

/**
 * @file handler.h
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2021-12-07
 * @copyright Copyright (c) 2021
 */

#include "point.hpp"
#include "angle.h"
#include <iomanip>

namespace ns_cc
{
    /**
     * @brief express the direction of a curve
     */
    enum class CornerDir
    {
        LEFT,
        RIGHT
    };
    /**
     * \brief a function to output info to the std::ostream
     * \param str the target string
     * \param ceil whether output the ceil line or not
     * \param floor whether output the floor line or not
     * \param symbol the char type to construct the lin
     * \param os the output stream type
     * \return void
     */
    void output(const std::string &str, bool ceil = false, bool floor = true,
                char symbol = '-', std::ostream &os = std::cout);
    /**
     * @brief the main class to handle with the calculation a curve
     */
    class Handler
    {
    private:
        std::ostream *_pos;
        /**
         * @brief initialized values
         */
        float _kjd;
        ns_geo::Point2f _pjd;
        ns_angle::Radian _corRad;
        CornerDir _corDir;
        float _r;
        float _ls;
        ns_angle::Radian _tanRad;
        /**
         * @brief middle values
         */
        float _m;
        float _P;
        ns_angle::Radian _beta_0;
        ns_angle::Radian _alpha_ZH_JD;
        ns_angle::Radian _alpha_JD_ZH;
        ns_angle::Radian _alpha_HZ_JD;
        ns_angle::Radian _alpha_JD_HZ;
        /**
         * @brief feature values
         */
        float _T_H;
        float _L_H;
        float _E_H;
        float _L_T;
        float _q;
        float _K_ZH;
        float _K_HY;
        float _K_QZ;
        float _K_YH;
        float _K_HZ;

    public:
        Handler() = delete;
        /**
         * @brief Construct a new Handler object
         * 
         * @param K_JD the mileage of the 'JiaoDian' point
         * @param P_JD the position of the 'JiaoDian' point
         * @param cornerDeg the angle[Degree] for the curve's corner
         * @param cornerDir the direction[LEFT|RIGHT] for the curve's corner
         * @param radius the radius for the curve's corner
         * @param Ls the length for the curve's easement curve
         * @param tangentDeg the tangrnt angle of this curve
         * @param os the output stream
         */
        Handler(float K_JD, const ns_geo::Point2f &P_JD, const ns_angle::Degree &cornerDeg,
                CornerDir cornerDir, float radius, float Ls, const ns_angle::Degree &tangentDeg, std::ostream &os = std::cout)
            : _kjd(K_JD), _pjd(P_JD), _corRad(static_cast<ns_angle::Radian>(cornerDeg)),
              _corDir(cornerDir), _r(radius), _ls(Ls), _tanRad(static_cast<ns_angle::Radian>(tangentDeg)), _pos(&os)
        {
            this->info();
            this->init();
            output("Main Points", true, true, '-', *_pos);
            *_pos << "[ P_JD ] : " << _pjd << std::endl;
            *_pos << "[ P_ZH ] : " << calculate(_K_ZH, false) << std::endl;
            *_pos << "[ P_HY ] : " << calculate(_K_HY, false) << std::endl;
            *_pos << "[ P_QZ ] : " << calculate(_K_QZ, false) << std::endl;
            *_pos << "[ P_YH ] : " << calculate(_K_YH, false) << std::endl;
            *_pos << "[ P_HZ ] : " << calculate(_K_HZ, false) << std::endl;
        }

    protected:
        void info() const;
        void init();
        void outputHelper(float K, const ns_geo::Point2f &pos) const;
        ns_geo::Point2f calculate_BZH(float K) const;
        ns_geo::Point2f calculate_ZH_HY(float K) const;
        ns_geo::Point2f calculate_HY_YH(float K) const;
        ns_geo::Point2f calculate_YH_HZ(float K) const;
        ns_geo::Point2f calculate_AHZ(float K) const;
        void checkAngleRange(ns_angle::Radian &r);

    public:
        /**
         * @brief calculate the position for the mileage 'K'
         * 
         * @param K the mileage
         * @param log whether to output the log info
         * @return ns_geo::Point2f the position
         */
        ns_geo::Point2f calculate(float K, bool log = true) const;

        ns_geo::PointSet3f calculate(float start_K, float end_K, float stride) const;

        void outputReport(float start_K, float end_K, float stride, std::ostream &os) const;

        inline float K_ZH() const { return _K_ZH; }
        inline float K_HZ() const { return _K_HZ; }
        inline float K_QZ() const { return _K_QZ; }
        inline float K_HY() const { return _K_HY; }
        inline float K_YH() const { return _K_YH; }
        
        inline ns_geo::Point2f P_ZH() const { return calculate(_K_ZH, false); }
        inline ns_geo::Point2f P_HZ() const { return calculate(_K_HZ, false); }
        inline ns_geo::Point2f P_QZ() const { return calculate(_K_QZ, false); }
        inline ns_geo::Point2f P_HY() const { return calculate(_K_HY, false); }
        inline ns_geo::Point2f P_YH() const { return calculate(_K_YH, false); }
    };

} // namespace ns_cc
