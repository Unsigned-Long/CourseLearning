#include "ceresSolver.h"
#include "newtonLS.h"
#include "taylorSeries.h"
#include "linear.h"
#include "sequential.h"
/**
 * \brief this option control the solve method
 * options : [1] RANGE : calculate the position use adaptive sliding range data items
 *           [2] ALL : calculate the position use all data items
 *           [3] ... : future options 
 */
#define CAL_MODE_RANGE true
#define CAL_MODE_ALL false

int main(int argc, char const *argv[])
{
    // use try-catch structure to improve program robustness
    try
    {
        std::fstream file("../pyDrawer/temp.txt", std::ios::out);
        // init the uwb solver system
        ns_uwb::UWBContext::UWBInit("../data/move2_RTLS_log.txt", "../configs/UWBParams.json");
        // get the data
        auto &data = ns_uwb::UWBContext::data();
        // Select a certain range of data for solution
#if CAL_MODE_RANGE
        ns_uwb::output("CAL_MODE_RANGE");
        // here we use range data to solve the problem
        auto range_start = data.cbegin();
        auto range_end = range_start + ns_uwb::UWBContext::CALCUL_RANGE;
        while (range_end < data.cend())
        {
            try
            {
                auto p = ns_uwb::newtonLS(range_start, range_end);
                file << p.x() << ',' << p.y() << ',' << p.z() << std::endl;
                range_end += ns_uwb::UWBContext::CALCUL_RANGE_STEP;
                range_start = range_end - ns_uwb::UWBContext::CALCUL_RANGE;
            }
            catch (const std::runtime_error &e)
            {
                ns_uwb::output(std::string("error happend : ") + e.what(), true, true, '*');
                range_end += ns_uwb::UWBContext::CALCUL_RANGE_STEP;
            }
        }
#endif

#if CAL_MODE_ALL
        ns_uwb::output("CAL_MODE_ALL");
        // here we use range data to solve the problem
        std::cout << ns_uwb::ceresPosition(data.cbegin(), data.cend()) << std::endl;
#endif
    }
    catch (const std::runtime_error &e)
    {
        ns_uwb::output(std::string("error happend : ") + e.what(), true, true, '+');
    }
    return 0;
}
