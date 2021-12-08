#pragma once

#include <iostream>
#include <vector>
#include <map>
#include "point.h"
#include <array>
#include "timer.h"
#include "datatime.h"

namespace ns_uwb
{

    /**
     * \brief A static context class used to store the state of the system, 
     *        such as various parameters.
     */
    class UWBContext
    {
    public:
        /**
         * \param STATION_COUNT Number of base stations
         */
        static int STATION_COUNT;

        static int CALCUL_RANGE;

        static int CALCUL_RANGE_STEP;

    public:
        /**
         * \brief It is used to organize the data of 
         *        each UWB equipment measurement record
         */
        struct DataItem
        {
            // the id of reference station
            int _refStationID;
            // the distance
            double _range;
            // the time stamp
            ns_dt::Time _time;
        };

    private:
        // the header info of the file
        static std::string _fileHeader;

        // the data in the file
        static std::vector<DataItem> _data;

        // the info of the stations
        static std::map<int, ns_point::Point3d> _baseStations;

        // the init pos
        static ns_point::Point3d _initPosition;

    public:
        UWBContext() = delete;

        // load the header info and data in the file
        static void UWBInit(const std::string &dataFilename, const std::string &configFilename);

        // get the base stations
        static const std::map<int, ns_point::Point3d> &baseStation() { return UWBContext::_baseStations; }

        // get the data
        static std::vector<DataItem> &data() { return UWBContext::_data; }

        // get the header info
        static const std::string &fileHeader() { return UWBContext::_fileHeader; }

        // Check the solution conditions, such as whether the amount of data is sufficient
        // and whether the solution state is good
        static void checkSolveCondition(std::vector<UWBContext::DataItem>::const_iterator start,
                                        std::vector<UWBContext::DataItem>::const_iterator end);

        static ns_point::Point3d &getInitPosition();

    private:
        static void loadDataFile(const std::string &dataFilename);

        static void loadConfigFile(const std::string &jsonFilename);

        static void calculateInitValue();
    };

#pragma region helping functions
    // overload for operator '<<'
    std::ostream &operator<<(std::ostream &os, const UWBContext::DataItem &i);

    /**
    * \brief a function to output info to the std::ostream
    * \param str the target string
    * \param ceil whether output the ceil line or not
    * \param floor whether output the floor line or not
    * \param symbol the char type to construct the lin
    * \param os the output stream type
    */
    void output(const std::string &str, bool ceil = false, bool floor = true,
                char symbol = '-', std::ostream &os = std::cout);

    /**
    * \brief a function to split a string to some string elements according the splitor
    * \param str the string to be splited
    * \param splitor the splitor char
    * \param ignoreEmpty whether ignoring the empty string element or not
    */
    std::vector<std::string> split(const std::string &str, char splitor, bool ignoreEmpty = true);

    /**
     * \brief a function to read all chars in the file and return a string
     * \param file the opened oouput file stream reference
     * \return the char string in the file
     */
    std::string readString(std::fstream &file);
#pragma endregion
} // namespace ns_uwb
