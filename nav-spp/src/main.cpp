#include "config.h"
#include "datetime.h"
#include "coordinate.h"
#include "artwork/logger/logger.h"
#include "utils/base_cast.hpp"
#include "data_parser.h"

int main(int argc, char *argv[]) {
    ns_spp::Config::loadConfigure("../config/spp.json");
    LOG_VAR(ns_spp::EnumCast::enumToInteger(ns_spp::PortIdentifier::COM3_31));
    return 0;
}
