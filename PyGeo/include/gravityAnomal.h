#pragma once

#include "parameter.h"
#include <string>
#include <list>
#include <algorithm>

namespace ns_pygeo
{
#pragma region class BGIReader
    /**
     * \brief a class serves for the \class GravityAnomal
     */ 
    class BGIReader
    {
    public:
        struct DataItem
        {
            int _source;
            // latitude unit[degree]
            double _lat;
            // Longitude unit[degree]
            double _long;
            // unit[m]
            double _alt;
            // Gravity observation unit[mGal]
            double _g;

            DataItem() = default;
        };

    private:
        // the header string for the BGI file
        std::string _header;
        // the data organized by DataItem
        std::list<DataItem> _data;

    public:
        BGIReader() = delete;
        /**
         * \brief pass a file name and init the class object
         */ 
        BGIReader(const std::string &fileName) { this->readData(fileName); }

    public:
        // get the header
        inline const std::string &header() const { return this->_header; }
        // get the data items
        inline const std::list<DataItem> &data() const { return this->_data; }

    private:
        // read the data and save them
        void readData(const std::string &fileName);
        // split a string into some small strings based on the splitor
        static std::vector<std::string> split(const std::string &str, char splitor);
    };
#pragma endregion

#pragma region class GravityAnomal
    /**
     * \brief the main class to calculate the gravity anomality
     */ 
    class GravityAnomal
    {
    public:
        struct GAItem
        {
            const BGIReader::DataItem *_dataItem;
            // [g - gamma] unit[mGal]
            double _originalGA;
            // [deta_1_g] unit[mGal]
            double _spatialCorr;
            // [deta_2_g] unit[mGal]
            double _layerCorr;

            GAItem() = default;
            // calculate and get the spatial gravity anomality
            double spatialGA() const { return this->_originalGA + this->_spatialCorr; }
            // calculate and get the simple Bouguer gravity anomality
            double simpleBouguerGA() const { return this->_originalGA + this->_spatialCorr + this->_layerCorr; }
        };

    private:
        // the data
        std::list<GAItem> _data;
        // the reference ellipsoid
        const ns_pygeo::params::Ellipsoid *_ellipsoid;

    public:
        GravityAnomal() = delete;
        /**
         * \brief construct a gravity anomality calculator
         * \param reader a BGIReader  pointer contains the data
         * \param ellipsoid the reference ellipsoid
         */
        GravityAnomal(const BGIReader *reader, const ns_pygeo::params::Ellipsoid *ellipsoid)
            : _ellipsoid(ellipsoid) { this->init(reader); };

    public:
        const std::list<GAItem> &data() const { return this->_data; }

    private:
        // calculate the values of  different gravity anomality correction
        void init(const BGIReader *reader);
        // calculate the spatial correction
        static double spatialCorrection(double alt);
        // calculate the layer correction
        static double layerCorrection(double alt);
        /**
         * \brief calculate the original gravity anomality
         * \param g the Gravity observations
         * \param ellipsoid the reference ellipsoid
         * \param radian_lat the latitude [radian]
         */
        static double originalGA(double g, const ns_pygeo::params::Ellipsoid *ellipsoid, double radian_lat);
    };
#pragma endregion

#pragma region operators overload
    // operator overload [BGIReader::DataItem]
    std::ostream &operator<<(std::ostream &os, const BGIReader::DataItem &item);

    // operator overload [GravityAnomal::GAItem]
    std::ostream &operator<<(std::ostream &os, const GravityAnomal::GAItem &item);
#pragma endregion

} // namespace ns_pygeo
