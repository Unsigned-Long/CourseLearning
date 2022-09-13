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

    struct Config {
    public:
        struct Author {
            static std::string name;
            static std::string e_mail;
        };

        struct Threshold {
            static long double DOUBLE_EQ;
            static long double ITERATE;
        };
        struct TimeSystem {
            static ModJulianDay GPSTOrigin;
            static ModJulianDay BDTOrigin;
        };

        struct RefEllipsoid {
            static ns_spp::RefEllipsoid WGS1984;
            static ns_spp::RefEllipsoid CGCS2000;
        };

    public:

        static void loadConfigure(const std::string &configPath);

    private:
        static std::string readString(const std::string &filePath);

        Config() = delete;
    };

}

#endif //NAV_SPP_CONFIG_H
