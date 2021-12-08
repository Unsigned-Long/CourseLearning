#include <chrono>
#include <iomanip>
#include <fstream>
#include "gravityField.h"

// file << std::fixed << std::setprecision(5);

int main(int argc, char const *argv[])
{
       ns_pygeo::GFCHandler handler("../data/SGG-UGM-1.gfc");

       handler.calculation(-89.5, 89.5, 1.0,
                           0.0, 360.0, 1.0,
                           360, ns_pygeo::params::GRS_80);

       handler.writeResult("../data/temp.csv");
       return 0;
}