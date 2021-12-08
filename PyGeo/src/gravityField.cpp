#include "gravityField.h"
#include <fstream>
#include <string>
#include <algorithm>
#include "timer.h"

namespace ns_pygeo
{

#pragma region public methods
    void GFCHandler::calculation(double lat_lower, double lat_upper, double lat_sep,
                                 double lon_lower, double lon_upper, double lon_sep,
                                 int end_n, const ns_pygeo::params::Ellipsoid &e)
    {
        lat_lower = ns_pygeo::params::degree2radian(lat_lower);
        lat_upper = ns_pygeo::params::degree2radian(lat_upper);
        lon_lower = ns_pygeo::params::degree2radian(lon_lower);
        lon_upper = ns_pygeo::params::degree2radian(lon_upper);
        lat_sep = ns_pygeo::params::degree2radian(lat_sep);
        lon_sep = ns_pygeo::params::degree2radian(lon_sep);
        ns_timer::DurationTimer<std::chrono::milliseconds> timer;
        timer.init();
        /**
         * \attention here we need remove p.z() to 0.0 at the end
         */
        ns_point::Point3d p(0.0, 0.0, 0.0);

        // it'is for [Cnm * Cos(m * lambda) + Snm * sin(m * lambda)] * Pnm(Cos)
        std::vector<double> param4(end_n - 2 + 1, 0.0);

#pragma region for test[cur : no testing]
        // ns_point::Point3d p(ns_pygeo::params::degree2radian(30 + 19 / 60.0 + 38.543230 / 3600.0),
        //                     ns_pygeo::params::degree2radian(103 + 9 / 60.0 + 30.881650 / 3600.0), 39.7790);
        // auto tempLat = ns_pygeo::params::degree2radian(20.0);
        // auto tempLon = ns_pygeo::params::degree2radian(118.5);
        // lat_lower = tempLat;
        // lat_upper = tempLat;
        // lon_lower = tempLon;
        // lon_upper = tempLon;
#pragma endregion

        for (auto lat = lat_lower; lat <= lat_upper; lat += lat_sep)
        {
            p.x() = lat;
            auto dataset = Legendre::legendre_dataset(end_n, ns_pygeo::params::BLH2SphCoor(p, e).y());
            auto normalGravity = e.normalGravity(lat);
            for (auto lon = lon_lower; lon <= lon_upper; lon += lon_sep)
            {
                /**
                 * \attention this part is just for speed calculation
                 */
                auto iter_param4 = param4.begin();
                p.x() = lat;
                p.y() = lon;
                auto sphCoor = ns_pygeo::params::BLH2SphCoor(p, e);
                auto lambda = sphCoor.z();
                auto iter = this->_data.cbegin() + 3;
                for (int n = 2; n <= end_n; ++n, ++iter_param4)
                {
                    auto temp = 0.0;
                    for (int m = 0; m <= n; ++m, ++iter)
                    {
                        auto cu = this->getCU(n, m, e);
                        auto m_lambda = m * lambda;
                        temp += ((iter->_C - this->getCU(n, m, e)) * std::cos(m_lambda) +
                                 iter->_S * std::sin(m_lambda)) *
                                dataset.at(IndexOrder(n, m));
                    }
                    *iter_param4 = temp;
                }
                /**
                 * \brief end
                 */
                auto a_r = e.a() / sphCoor.x();
                auto a_r_2 = a_r * a_r;
                auto a_r_3 = a_r_2 * a_r;
                auto GM_r_2 = e.GM() / std::pow(sphCoor.x(), 2);
                auto GM_r = GM_r_2 * sphCoor.x();

                auto disturbancePotential = _helpFunction(sphCoor, end_n, e, GM_r, _disturbancePotential(), a_r_2, param4);

                auto geoidHeight = _helpFunction(sphCoor, end_n, e, e.a(), _geoidHeight(), a_r_3, param4);

                auto gravityAnomal = 1E05 * _helpFunction(sphCoor, end_n, e, GM_r_2, _gravityAnomal(), a_r_2, param4);

                auto gravityDisturbance = 1E05 * _helpFunction(sphCoor, end_n, e, GM_r_2, _gravityDisturbance(), a_r_2, param4);

                p.x() = params::radian2degree(p.x());
                p.y() = params::radian2degree(p.y());
                this->_result.push_back(CalResult{
                    p,
                    disturbancePotential,
                    geoidHeight,
                    gravityAnomal,
                    gravityDisturbance});
            }
            outputFormat(timer.lastDurStr("lat at " + std::to_string(p.x()) + ", cost time"));
        }
        outputFormat(timer.totalDurStr("total cost"));
        return;
    }

    void GFCHandler::writeResult(const std::string &filename)
    {
        std::fstream file(filename, std::ios::out);
        if (!file.is_open())
            throw std::runtime_error("file '" + filename + "' open failed");
        file << "lat,lon,hieight,disturbancePotential,geoidHeight,gravityAnomal,gravityDisturbance" << std::endl;
        for (const auto &elem : this->_result)
            file
                << elem._p.x() << ','
                << elem._p.y() << ','
                << elem._p.z() << ','
                << elem._disturbancePotential << ','
                << elem._geoidHeight << ','
                << elem._gravityAnomal << ','
                << elem._gravityDisturbance << '\n';
        file.close();
        return;
    }

#pragma endregion

#pragma region private methods
    void GFCHandler::readData(const std::string &filename)
    {
        std::fstream file(filename, std::ios::in);
        if (!file.is_open())
            throw std::runtime_error("the file '" + filename + "' open failed");
        std::string strLine;
        while (std::getline(file, strLine))
        {
            if (strLine.size() <= 13)
                continue;
            if (std::string(strLine.cbegin(), strLine.cbegin() + 13) == "begin_of_head")
                break;
        }
        // read header
        ns_timer::DurationTimer<std::chrono::milliseconds> timer;
        // initialize the timer
        timer.init();
        outputFormat("reading header", true);
        while (std::getline(file, strLine))
        {
            this->_header += strLine;
            if (strLine.empty())
                break;
        }
        std::getline(file, strLine);
        std::getline(file, strLine);
        outputFormat(timer.lastDurStr("cost time"));
        // read data
        outputFormat("reading data");
        while (std::getline(file, strLine))
        {
            if (strLine.empty())
                continue;
            auto vec = split(strLine, ' ', true);
            this->_data.push_back(DataItem{
                ns_pygeo::IndexOrder(std::stoi(vec.at(1)), std::stoi(vec.at(2))),
                std::stod(vec.at(3)),
                std::stod(vec.at(4))});
        }
        outputFormat(timer.lastDurStr("cost time"));
        outputFormat("item num [" + std::to_string(this->_data.size()) + ']');
        file.close();
        return;
    }

    double GFCHandler::_helpFunction(const ns_point::Point3d &sphCoor, int end_n, const ns_pygeo::params::Ellipsoid &e,
                                     double param1, parma2_getfun param2, double params3, const std::vector<double> &param4)
    {

        auto r = sphCoor.x();
        auto theta = sphCoor.y();
        auto lambda = sphCoor.z();
        auto result = 0.0;
        /**
         * \brief params to speed calculation
         */
        auto iter = this->_data.cbegin() + 3;
        double m_lambda = 0.0;
        double factor = e.a() / r;
        double accuFactor = params3;
        auto iter_param4 = param4.cbegin();
        for (int n = 2; n <= end_n; ++n, ++iter_param4)
        {
            auto temp = *iter_param4;
            temp *= accuFactor;
            if (param2 != nullptr)
                temp *= param2(n);
            result += temp;
            accuFactor *= factor;
        }
        result *= param1;
        return result;
    }

    double GFCHandler::__helpFunction(const ns_point::Point3d &sphCoor, int end_n, const ns_pygeo::params::Ellipsoid &e,
                                      double param1, parma2_getfun param2, double params3, Legendre::legendre_map *dt)
    {

        auto r = sphCoor.x();
        auto theta = sphCoor.y();
        auto lambda = sphCoor.z();
        auto &dataset = *dt;
        auto result = 0.0;
        /**
         * \brief params to speed calculation
         */
        auto iter = this->_data.cbegin() + 3;
        double m_lambda = 0.0;
        double factor = e.a() / r;
        double accuFactor = params3;
        for (int n = 2; n <= end_n; ++n)
        {
            auto temp = 0.0;
            for (int m = 0; m <= n; ++m, ++iter)
            {
                auto cu = this->getCU(n, m, e);
                m_lambda = m * lambda;
                temp += ((iter->_C - this->getCU(n, m, e)) * std::cos(m_lambda) +
                         iter->_S * std::sin(m_lambda)) *
                        dataset.at(IndexOrder(n, m));
            }
            temp *= accuFactor;
            if (param2 != nullptr)
                temp *= param2(n);
            result += temp;
            accuFactor *= factor;
        }
        result *= param1;
        return result;
    }

    double GFCHandler::getCU(int n, int m, const ns_pygeo::params::Ellipsoid &e)
    {
        if (m != 0)
            return 0.0;
        if (n == 2)
            return -e.J2() / std::sqrt(2 * n + 1);
        else if (n == 4)
            return -e.J4() / std::sqrt(2 * n + 1);
        else if (n == 6)
            return -e.J6() / std::sqrt(2 * n + 1);
        else if (n == 8)
            return -e.J8() / std::sqrt(2 * n + 1);
        else
            return 0.0;
    }

    std::vector<std::string> split(const std::string &str, char splitor, bool ignoreEmpty)
    {
        std::vector<std::string> vec;
        auto iter = str.cbegin();
        while (true)
        {
            auto pos = std::find(iter, str.cend(), splitor);
            auto elem = std::string(iter, pos);
            if ((!ignoreEmpty) || (ignoreEmpty && !elem.empty()))
                vec.push_back(elem);
            if (pos == str.cend())
                break;
            iter = ++pos;
        }
        return vec;
    }

    void outputFormat(const std::string &str, bool ceil, bool floor,
                      char symbol, std::ostream &os)
    {
        if (ceil)
            os << std::string(str.length(), symbol) << std::endl;
        os << str << std::endl;
        if (floor)
            os << std::string(str.length(), symbol) << std::endl;
        return;
    }

#pragma endregion
} // namespace ns_pygeo