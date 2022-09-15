#include "config.h"
#include "datetime.h"
#include "coordinate.h"
#include "artwork/logger/logger.h"

int main(int argc, char *argv[]) {
    ns_spp::Config::loadConfigure("../config/spp.json");
    return 0;
}
