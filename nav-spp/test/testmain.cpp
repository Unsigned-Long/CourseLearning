#include "test_datetime.hpp"
#include "test_config.hpp"
#include "test_coordinate.hpp"

int main(int argc, char *argv[]) {
    ns_spp::Config::loadConfigure("../config/spp.json");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}