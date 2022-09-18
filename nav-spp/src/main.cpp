#include "config.h"
#include "datetime.h"
#include "coordinate.h"
#include "artwork/logger/logger.h"
#include "utils/base_cast.hpp"

int main(int argc, char *argv[]) {
    ns_spp::Config::loadConfigure("../config/spp.json");
    return 0;
}
