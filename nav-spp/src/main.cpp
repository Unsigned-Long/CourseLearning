#include "config.h"
#include "datetime.h"
#include "coordinate.h"
#include "artwork/logger/logger.h"
#include "utils/base_cast.hpp"
#include "data_parser.h"

int main(int argc, char *argv[]) {
    ns_spp::Config::loadConfigure("../config/spp.yaml");
    ns_spp::NovAtelOEM("../data/202209051200.oem719");
    return 0;
}



