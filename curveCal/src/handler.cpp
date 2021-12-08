/**
 * @file handler.cpp
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2021-12-07
 * @copyright Copyright (c) 2021
 */

#include "handler.h"

namespace ns_cc
{
    void Handler::info() const
    {
        *_pos << std::setiosflags(std::ios::fixed) << std::setprecision(3);
        output("Details for This Curve[m]", true, true, '-', *_pos);
        *_pos << "[ K_JD ] : " << _kjd << std::endl;
        *_pos << "[ P_JD ] : " << _pjd << std::endl;
        *_pos << "[ CorA ] : " << ns_angle::toAngleStrExp(_corRad) << std::endl;
        *_pos << "[ CorD ] : " << (_corDir == CornerDir::LEFT ? "LEFT" : "RIGHT") << std::endl;
        *_pos << "[ CurR ] : " << _r << std::endl;
        *_pos << "[ C_Ls ] : " << _ls << std::endl;
        *_pos << "[ TanA ] : " << ns_angle::toAngleStrExp(_tanRad) << std::endl;
        return;
    }

    void Handler::init()
    {
        _m = _ls * 0.5f - std::pow(_ls, 3) / (240.0f * _r * _r);
        _P = _ls * _ls / (24.0f * _r);
        _beta_0 = ns_angle::Radian(_ls * 0.5f / _r);

        _alpha_ZH_JD = _tanRad;
        checkAngleRange(_alpha_ZH_JD);
        _alpha_JD_ZH = _tanRad - ns_angle::Radian(M_PI);
        checkAngleRange(_alpha_JD_ZH);
        _alpha_JD_HZ = _corDir == CornerDir::RIGHT ? _tanRad + _corRad : _tanRad - _corRad;
        checkAngleRange(_alpha_JD_HZ);
        _alpha_HZ_JD = _alpha_JD_HZ - ns_angle::Radian(M_PI);
        checkAngleRange(_alpha_HZ_JD);

        _T_H = (_r + _P) * std::tan(_corRad * 0.5f) + _m;
        _L_H = float(_corRad - 2.0f * _beta_0) * _r + 2.0f * _ls;
        _L_T = float(_corRad - 2.0f * _beta_0) * _r;
        _E_H = (_r + _P) / std::cos(_corRad * 0.5f) - _r;
        _q = 2.0f * _T_H - _L_H;
        _K_ZH = _kjd - _T_H;
        _K_HY = _K_ZH + _ls;
        _K_QZ = _K_ZH + _L_H * 0.5f;
        _K_YH = _K_HY + _L_T;
        _K_HZ = _K_YH + _ls;

        output("Features for This Curve[m]", true, true, '-', *_pos);
        *_pos << "[  m   ] : " << _m << std::endl;
        *_pos << "[  P   ] : " << _P << std::endl;
        *_pos << "[beta_0] : " << ns_angle::toAngleStrExp(_beta_0) << std::endl;
        *_pos << "[ T_H  ] : " << _T_H << std::endl;
        *_pos << "[ L_H  ] : " << _L_H << std::endl;
        *_pos << "[ L_T  ] : " << _L_T << std::endl;
        *_pos << "[ E_H  ] : " << _E_H << std::endl;
        *_pos << "[  q   ] : " << _q << std::endl;
        *_pos << "[ K_ZH ] : " << _K_ZH << std::endl;
        *_pos << "[ K_HY ] : " << _K_HY << std::endl;
        *_pos << "[ K_QZ ] : " << _K_QZ << std::endl;
        *_pos << "[ K_YH ] : " << _K_YH << std::endl;
        *_pos << "[ K_HZ ] : " << _K_HZ << std::endl;
        *_pos << "[ alpha ZH-JD ] : " << ns_angle::toAngleStrExp(_alpha_ZH_JD) << std::endl;
        *_pos << "[ alpha JD-ZH ] : " << ns_angle::toAngleStrExp(_alpha_JD_ZH) << std::endl;
        *_pos << "[ alpha HZ-JD ] : " << ns_angle::toAngleStrExp(_alpha_HZ_JD) << std::endl;
        *_pos << "[ alpha JD-HZ ] : " << ns_angle::toAngleStrExp(_alpha_JD_HZ) << std::endl;
        output("Check for This Curve[m]", true, true, '-', *_pos);
        *_pos << "[     K_JD     ] = " << _kjd << std::endl;
        *_pos << "[K_QZ + 0.5 * q] = " << _K_QZ + 0.5 * _q << std::endl;
        return;
    }

    ns_geo::Point2f Handler::calculate(float K, bool log) const
    {
        ns_geo::Point2f pos;
        if (K <= _K_ZH)
            pos = calculate_BZH(K);
        else if (K <= _K_HY)
            pos = calculate_ZH_HY(K);
        else if (K <= _K_YH)
            pos = calculate_HY_YH(K);
        else if (K < _K_HZ)
            pos = calculate_YH_HZ(K);
        else
            pos = calculate_AHZ(K);
        if (log)
            outputHelper(K, pos);
        return pos;
    }

    ns_geo::PointSet3f Handler::calculate(float start_K, float end_K, float stride) const
    {
        ns_geo::PointSet3f ps;
        for (float K = start_K; K < end_K; K += stride)
        {
            auto pos = calculate(K, false);
            ps.push_back({pos.x(), pos.y(), K});
        }
        return ps;
    }

    void Handler::outputReport(float start_K, float end_K, float stride, std::ostream &os) const
    {
        os << std::setiosflags(std::ios::fixed) << std::setprecision(3);
        output("曲线要素", true, true, '-', os);
        os << " 切线长T：" << _T_H << "\t曲线长L：" << _L_H << std::endl;
        os << "外矢距E0：" << _E_H << "\t切曲差q：" << _q << std::endl;
        output("主点       里程         X            Y", true, true, '-', os);
        int w = 13;
        os << std::setw(4) << " ZH" << std::setw(w) << _K_ZH << std::setw(w) << P_ZH().x() << std::setw(w) << P_ZH().y() << std::endl;
        os << std::setw(4) << " HZ" << std::setw(w) << _K_HZ << std::setw(w) << P_HZ().x() << std::setw(w) << P_HZ().y() << std::endl;
        os << std::setw(4) << " QZ" << std::setw(w) << _K_QZ << std::setw(w) << P_QZ().x() << std::setw(w) << P_QZ().y() << std::endl;
        os << std::setw(4) << " HY" << std::setw(w) << _K_HY << std::setw(w) << P_HY().x() << std::setw(w) << P_HY().y() << std::endl;
        os << std::setw(4) << " YH" << std::setw(w) << _K_YH << std::setw(w) << P_YH().x() << std::setw(w) << P_YH().y() << std::endl;
        output("加密点     里程         X            Y", true, true, '-', os);
        int index = 0;
        for (float K = start_K; K < end_K; K += stride)
        {
            auto pos = calculate(K, false);
            os << std::setw(4) << index++ << std::setw(w) << K << std::setw(w) << pos.x() << std::setw(w) << pos.y() << std::endl;
        }
        return;
    }

    ns_geo::Point2f Handler::calculate_BZH(float K) const
    {
        ns_geo::Point2f pos;
        float l_i = _kjd - K;
        auto phi = _alpha_JD_ZH;
        pos.x() = _pjd.x() + l_i * std::cos(phi);
        pos.y() = _pjd.y() + l_i * std::sin(phi);
        return pos;
    }

    ns_geo::Point2f Handler::calculate_ZH_HY(float K) const
    {
        ns_geo::Point2f pos;
        float l_i = K - _K_ZH;
        float x_i = l_i - std::pow(l_i, 5) / (40.0 * _r * _r * l_i * l_i);
        float y_i = std::pow(l_i, 3) / (6.0 * _r * l_i);
        auto alpha_ZH = _alpha_ZH_JD;
        float X_ZH = _pjd.x() - _T_H * std::cos(alpha_ZH);
        float Y_ZH = _pjd.y() - _T_H * std::sin(alpha_ZH);
        if (_corDir == CornerDir::RIGHT)
        {
            pos.x() = X_ZH + x_i * std::cos(alpha_ZH) - y_i * std::sin(alpha_ZH);
            pos.y() = Y_ZH + x_i * std::sin(alpha_ZH) + y_i * std::cos(alpha_ZH);
        }
        else
        {
            pos.x() = X_ZH + x_i * std::cos(alpha_ZH) + y_i * std::sin(alpha_ZH);
            pos.y() = Y_ZH + x_i * std::sin(alpha_ZH) - y_i * std::cos(alpha_ZH);
        }
        return pos;
    }

    ns_geo::Point2f Handler::calculate_HY_YH(float K) const
    {
        ns_geo::Point2f pos;
        float l_i = K - _K_ZH;
        auto phi = _beta_0 + ns_angle::Radian((l_i - _ls) / _r);
        float x_i = _m + _r * std::sin(phi);
        float y_i = _P + _r * (1 - std::cos(phi));
        auto alpha_ZH = _alpha_ZH_JD;
        float X_ZH = _pjd.x() - _T_H * std::cos(alpha_ZH);
        float Y_ZH = _pjd.y() - _T_H * std::sin(alpha_ZH);
        if (_corDir == CornerDir::RIGHT)
        {
            pos.x() = X_ZH + x_i * std::cos(alpha_ZH) - y_i * std::sin(alpha_ZH);
            pos.y() = Y_ZH + x_i * std::sin(alpha_ZH) + y_i * std::cos(alpha_ZH);
        }
        else
        {
            pos.x() = X_ZH + x_i * std::cos(alpha_ZH) + y_i * std::sin(alpha_ZH);
            pos.y() = Y_ZH + x_i * std::sin(alpha_ZH) - y_i * std::cos(alpha_ZH);
        }
        return pos;
    }

    ns_geo::Point2f Handler::calculate_YH_HZ(float K) const
    {
        ns_geo::Point2f pos;
        float l_i = _K_HZ - K;
        float x_i = l_i - std::pow(l_i, 5) / (40.0 * _r * _r * l_i * l_i);
        float y_i = std::pow(l_i, 3) / (6.0 * _r * l_i);
        auto alpha_HZ = _alpha_HZ_JD;
        float X_HZ = _pjd.x() - _T_H * std::cos(alpha_HZ);
        float Y_HZ = _pjd.y() - _T_H * std::sin(alpha_HZ);
        if (_corDir == CornerDir::RIGHT)
        {
            pos.x() = X_HZ + x_i * std::cos(alpha_HZ) + y_i * std::sin(alpha_HZ);
            pos.y() = Y_HZ + x_i * std::sin(alpha_HZ) - y_i * std::cos(alpha_HZ);
        }
        else
        {
            pos.x() = X_HZ + x_i * std::cos(alpha_HZ) - y_i * std::sin(alpha_HZ);
            pos.y() = Y_HZ + x_i * std::sin(alpha_HZ) + y_i * std::cos(alpha_HZ);
        }
        return pos;
    }

    ns_geo::Point2f Handler::calculate_AHZ(float K) const
    {
        ns_geo::Point2f pos;
        float l_i = K - _kjd + _q;
        auto phi = _alpha_JD_HZ;
        pos.x() = _pjd.x() + l_i * std::cos(phi);
        pos.y() = _pjd.y() + l_i * std::sin(phi);
        return pos;
    }

    void Handler::checkAngleRange(ns_angle::Radian &r)
    {
        if (float(r) < 0.0)
            r += ns_angle::Radian(2 * M_PI);
        if (float(r) > 2 * M_PI)
            r -= ns_angle::Radian(2 * M_PI);
        return;
    }

    void Handler::outputHelper(float K, const ns_geo::Point2f &pos) const
    {
        output("Calculation[K = " + std::to_string(K) + "]", true, true, '-', *_pos);
        *_pos << pos << std::endl;
        return;
    }

    void output(const std::string &str, bool ceil, bool floor,
                char symbol, std::ostream &os)
    {
        if (ceil)
            os << std::string(str.length(), symbol) << std::endl;
        os << str << std::endl;
        if (floor)
            os << std::string(str.length(), symbol) << std::endl;
        return;
    }
} // namespace ns_cc
