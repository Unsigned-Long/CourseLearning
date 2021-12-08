#pragma once

#include "legendre.h"
#include <vector>
#include "point.h"

namespace ns_pygeo
{
    /**
     * \brief a cpp class to handle the GFC data calculation
     */
    class GFCHandler
    {
    public:
        using parma2_getfun = int (*)(int n);

    public:
        /**
         * \brief the type and members of each data item
         */
        struct DataItem
        {
            ns_pygeo::IndexOrder _nm;

            double _C;
            double _S;

            DataItem() = default;
        };
        /**
         * \brief the members of each calculation result
         */
        struct CalResult
        {
            ns_point::Point3d _p;
            double _disturbancePotential;
            double _geoidHeight;
            double _gravityAnomal;
            double _gravityDisturbance;
        };

    private:
        /**
         * \brief members for reading
         */
        std::string _header;
        std::vector<DataItem> _data;
        /**
         * \brief member for recording calculating result
         */
        std::vector<CalResult> _result;

    public:
        GFCHandler() = delete;

        GFCHandler(const std::string &filename) { this->readData(filename); }

        /**
         *  \attention the unit is [degree]
         *  \param the calculation range, max index and reference ellipsoid
         */
        void calculation(double lat_lower, double lat_upper, double lat_sep,
                         double long_lower, double long_upper, double long_sep,
                         int end_n, const ns_pygeo::params::Ellipsoid &e);

        /**
         * \brief write the calculation result to the file
         */
        void writeResult(const std::string &filename);

    private:
        /**
         * \brief a help function
         * \param sphCoor the Spherical coordinates
         * \param end_n the index
         * \param e the reference Ellipsoid
         * \param param1 it's for [GM / r]||[GM / r ^ 2]||[a]
         * \param param2 a function to get [1]||[n - 1]||[n + 1]
         * \param param3 it's for [a / r]||[(a / r) ^ 2]
         * \param param4 it'is for [Cnm * Cos(m * lambda) + Snm * sin(m * lambda)] * Pnm(Cos)
         */
        double _helpFunction(const ns_point::Point3d &sphCoor, int end_n, const ns_pygeo::params::Ellipsoid &e,
                             double param1, parma2_getfun param2, double params3, const std::vector<double> &param4);

        // just used during designe the algorithm
        double __helpFunction(const ns_point::Point3d &sphCoor, int end_n, const ns_pygeo::params::Ellipsoid &e,
                              double param1, parma2_getfun param2, double params3, Legendre::legendre_map *dt);

        // return function pointer for different problem
        inline parma2_getfun _disturbancePotential() { return nullptr; }

        inline parma2_getfun _geoidHeight() { return nullptr; }

        inline parma2_getfun _gravityAnomal()
        {
            return [](int n) -> int
            {
                return n - 1;
            };
        }

        inline parma2_getfun _gravityDisturbance()
        {
            return [](int n) -> int
            {
                return n + 1;
            };
        }

        // read the gfc data
        void readData(const std::string &filename);

        // get param 'Cu' when calculation
        double getCU(int n, int m, const ns_pygeo::params::Ellipsoid &e);
    };
    /**
     * \brief a function to split a string to some string elements according the splitor
     * \param str the string to be splited
     * \param splitor the splitor char
     * \param ignoreEmpty whether ignoring the empty string element or not
     */
    std::vector<std::string> split(const std::string &str, char splitor, bool ignoreEmpty = true);

    /**
     * \brief a function to output info to the std::ostream
     * \param str the target string
     * \param ceil whether output the ceil line or not
     * \param floor whether output the floor line or not
     * \param symbol the char type to construct the lin
     * \param os the output stream type
     */
    void outputFormat(const std::string &str, bool ceil = false, bool floor = true,
                      char symbol = '-', std::ostream &os = std::cout);

} // namespace ns_pygeo
