#include "gravityAnomal.h"
#include <fstream>

namespace ns_pygeo
{
    std::ostream &operator<<(std::ostream &os, const BGIReader::DataItem &item)
    {
        os << item._source << ','
           << item._lat << "[D]" << ','
           << item._long << "[D]" << ','
           << item._alt << "[m]" << ','
           << item._g << "[mGal]";
        return os;
    }

    std::ostream &operator<<(std::ostream &os, const GravityAnomal::GAItem &item)
    {
        os << *item._dataItem << ','
           << item._originalGA << "[mGal]" << ','
           << item._spatialCorr << "[mGal]" << ','
           << item._layerCorr << "[mGal]";
        return os;
    }
    void BGIReader::readData(const std::string &fileName)
    {
        std::fstream file(fileName, std::ios::in);
        if (!file.is_open())
            exit(-1);
        // hearder
        std::string strLine;
        while (true)
        {
            std::getline(file, strLine);
            if (strLine.front() == '#')
                this->_header += strLine;
            else
                break;
        }
        // data
        do
        {
            auto vec = BGIReader::split(strLine, ' ');
            this->_data.push_back(DataItem());
            this->_data.back()._source = std::stoi(vec[0]);
            this->_data.back()._lat = std::stod(vec[1]);
            this->_data.back()._long = std::stod(vec[2]);
            this->_data.back()._alt = std::stod(vec[3]);
            this->_data.back()._g = std::stod(vec[4]);
        } while (std::getline(file, strLine));
        file.close();
        return;
    }

    std::vector<std::string> BGIReader::split(const std::string &str, char splitor)
    {
        std::vector<std::string> vec;
        auto iter = str.cbegin();
        while (true)
        {
            auto pos = std::find(iter, str.cend(), splitor);
            vec.push_back(std::string(iter, pos));
            if (pos == str.cend())
                break;
            iter = ++pos;
        }
        return vec;
    }

    void GravityAnomal::init(const BGIReader *reader)
    {
        auto &readerData = reader->data();
        this->_data.resize(readerData.size());
        auto readerIter = readerData.begin();
        auto dataIter = this->_data.begin();
        while (dataIter != this->_data.end())
        {
            dataIter->_dataItem = &(*readerIter);
            dataIter->_originalGA =
                this->originalGA(readerIter->_g, this->_ellipsoid, readerIter->_lat * params::PI / 180.0);
            dataIter->_spatialCorr = this->spatialCorrection(readerIter->_alt);
            dataIter->_layerCorr = this->layerCorrection(readerIter->_alt);
            ++readerIter;
            ++dataIter;
        }
    }

    double GravityAnomal::originalGA(double g, const ns_pygeo::params::Ellipsoid *ellipsoid, double radian_lat)
    {
        return g - 1E5 * ellipsoid->normalGravity(radian_lat);
    }
    double GravityAnomal::spatialCorrection(double alt)
    {
        return 0.3086 * alt - 7.2 * 1E-8 * alt * alt;
    }

    double GravityAnomal::layerCorrection(double alt)
    {
        return -0.1119 * alt;
    }
} // namespace ns_pygeo
