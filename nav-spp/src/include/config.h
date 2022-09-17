//
// Created by csl on 9/12/22.
//

#ifndef NAV_SPP_CONFIG_H
#define NAV_SPP_CONFIG_H

#include <ostream>
#include "string"
#include "memory"

namespace ns_spp {
    struct ModJulianDay;
    struct RefEllipsoid;

    /**
     * a configure struct using in all program
     * @attention the members are all static
     */
    struct Config {
    public:
        /**
         * the author's information
         */
        struct Author {
            static std::string name;
            static std::string eMail;
            static std::string address;
        };

        /**
         * the threshold for algorithm or values
         */
        struct Threshold {
            // the threshold to judge whether two doubles are equal
            static long double DOUBLE_EQ;
            // the coordinate transformation accuracy threshold
            static long double POSITION;
            // the threshold for iterate algorithm
            static long double ITERATE;
        };

        /**
         * the origin for the specific global position systems
         * @attention the origins are expressed in the form of modify julian day
         */
        struct TimeSystem {
            static ModJulianDay GPSTOrigin;
            static ModJulianDay BDTOrigin;
;        };

        /**
         * the reference ellipsoids
         */
        struct RefEllipsoid {
            // for GPS
            static ns_spp::RefEllipsoid WGS1984;
            // for BDS
            static ns_spp::RefEllipsoid CGCS2000;
        };

    public:

        /**
         * @brief load the configure file from the path
         *
         * @param configPath the file path
         */
        static void loadConfigure(const std::string &configPath);

    private:
        static std::string readString(const std::string &filePath);

        Config() = delete;
    };

}

#endif //NAV_SPP_CONFIG_H
