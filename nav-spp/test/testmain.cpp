#include "test_datetime.hpp"
#include "test_config.hpp"
#include "test_coordinate.hpp"
#include "test_enum_cast.hpp"
#include "test_base_cast.hpp"
#include "test_data_parser.hpp"

int main(int argc, char *argv[]) {
    ns_spp::Config::loadConfigure("../config/spp.yaml");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}