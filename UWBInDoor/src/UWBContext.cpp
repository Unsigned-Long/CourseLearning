#include "UWBContext.h"
#include <fstream>
#include <string>
#include <set>
#include <rapidjson/document.h>

namespace ns_uwb
{
#pragma region init for the context members
    int UWBContext::STATION_COUNT = 0;

    int UWBContext::CALCUL_RANGE = 0;

    int UWBContext::CALCUL_RANGE_STEP = 0;

    std::string UWBContext::_fileHeader = std::string("");

    std::vector<UWBContext::DataItem> UWBContext::_data = std::vector<UWBContext::DataItem>();

    std::map<int, ns_point::Point3d> UWBContext::_baseStations = std::map<int, ns_point::Point3d>();

    ns_point::Point3d UWBContext::_initPosition = ns_point::Point3d();
#pragma endregion
    void UWBContext::UWBInit(const std::string &dataFilename, const std::string &configFilename)
    {
        UWBContext::loadConfigFile(configFilename);
        UWBContext::loadDataFile(dataFilename);
        UWBContext::calculateInitValue();
        return;
    }

    void UWBContext::checkSolveCondition(std::vector<UWBContext::DataItem>::const_iterator start,
                                         std::vector<UWBContext::DataItem>::const_iterator end)
    {
        output("data from '" + start->_time.stringExpr() + "' to '" +
               (end - 1)->_time.stringExpr() + "' dur {" +
               std::to_string(ns_dt::distance(start->_time, (end - 1)->_time, ns_dt::TimeUnit::SECOND)) + " s}");
        std::string range = ". start[" + std::to_string(start - UWBContext::data().cbegin()) +
                            "] end[" + std::to_string(end - UWBContext::data().cbegin()) + ']';
        if (std::distance(start, end) < 4)
            throw std::runtime_error("item num less than necessary needs" + range);
        std::set<int> ids;
        for (auto iter = start; iter != end; ++iter)
        {
            ids.insert(iter->_refStationID);
            if (ids.size() >= STATION_COUNT)
                break;
        }
        if (ids.size() < STATION_COUNT)
            throw std::runtime_error("loss info of some necessary station info" + range);
        return;
    }

    ns_point::Point3d &UWBContext::getInitPosition()
    {
        return UWBContext::_initPosition;
    }

    void UWBContext::calculateInitValue()
    {
        ns_point::Point3d &center = UWBContext::_initPosition;
        for (const auto &elem : UWBContext::_baseStations)
        {
            center.x() += elem.second.x();
            center.y() += elem.second.y();
            center.z() += elem.second.z();
        }
        center.x() /= STATION_COUNT;
        center.y() /= STATION_COUNT;
        center.z() /= STATION_COUNT;
        return;
    }

    void UWBContext::loadDataFile(const std::string &dataFilename)
    {
        std::fstream file(dataFilename, std::ios::in);
        if (!file.is_open())
            throw std::runtime_error("the file of '" + dataFilename + "' open failed");
        output("data file name : " + dataFilename, true);
        ns_timer::DurationTimer<std::chrono::milliseconds> timer;
        timer.init();
        std::string strLine;
        /**
         * read the file head info
         */
        output("read file head info");
        for (int i = 0; i != 3; ++i)
        {
            std::getline(file, strLine, '\r');
            UWBContext::_fileHeader += strLine;
        }
        for (int i = 0; i != STATION_COUNT; ++i)
        {
            std::getline(file, strLine, '\r');
            auto vec = split(strLine, ':');
            auto id = std::stoi(vec.at(3));
            ns_point::Point3d station;
            station.x() = std::stod(vec.at(4));
            station.y() = std::stod(vec.at(5));
            station.z() = std::stod(vec.at(6));
            UWBContext::_baseStations.insert(std::make_pair(id, station));
        }
        /**
         * read the file data
         */
        int invalid = 0;
        int total = 0;
        output("read file data");
        while (getline(file, strLine, '\r'))
        {
            if (strLine == "\n")
                continue;
            ++total;
            if (split(strLine, ':').at(2) != "RR")
            {
                ++invalid;
                continue;
            }
            UWBContext::_data.push_back(DataItem{0, 0.0, ns_dt::Time(0, 0, 0, 0)});
            auto vec = split(strLine, ':');
            auto timeStr = vec.at(1);
            ns_dt::Time t(
                std::stoi(std::string(timeStr.begin(), timeStr.begin() + 2)),
                std::stoi(std::string(timeStr.begin() + 2, timeStr.begin() + 4)),
                std::stoi(std::string(timeStr.begin() + 4, timeStr.begin() + 6)),
                std::stoi(std::string(timeStr.begin() + 6, timeStr.begin() + 9)));
            UWBContext::_data.back()._time = t;
            UWBContext::_data.back()._refStationID = std::stoi(vec.at(4));
            /**
             * \brief here we get the range from base station to current position
             * \attention the unit is m [mm -> m]
             */
            UWBContext::_data.back()._range = std::stod(vec.at(6)) / 1000.0;
        }
        output("total item [" + std::to_string(total) + ']' +
               ", invaild count [" + std::to_string(invalid) + ']' +
               timer.lastDurStr(", cost time"));
        file.close();
        return;
    }

    void UWBContext::loadConfigFile(const std::string &jsonFilename)
    {
        std::fstream file(jsonFilename, std::ios::in);
        if (!file.is_open())
            throw std::runtime_error("the file of '" + jsonFilename + "' open failed");
        output("config file name : " + jsonFilename, false);
        ns_timer::DurationTimer<std::chrono::milliseconds> timer;
        timer.init();

        auto jsonStr = readString(file);
        rapidjson::Document doc;
        doc.Parse(jsonStr.c_str());
        UWBContext::STATION_COUNT = doc["STATION_COUNT"].GetInt();
        UWBContext::CALCUL_RANGE = doc["CALCUL_RANGE"].GetInt();
        UWBContext::CALCUL_RANGE_STEP = doc["CALCUL_RANGE_STEP"].GetInt();

        output(timer.lastDurStr("cost time"));
        return;
    }
#pragma region helping functions

    std::ostream &operator<<(std::ostream &os, const UWBContext::DataItem &i)
    {
        os << "TIME[" << i._time << "], REF_AP[" << i._refStationID << "], RANGE[" << i._range << ']';
        return os;
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

    std::string readString(std::fstream &file)
    {
        file.seekg(0, std::ios::end);
        auto size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::string str(size, ' ');
        file.read(const_cast<char *>(str.c_str()), size);
        return str;
    }

#pragma endregion
} // namespace ns_uwb