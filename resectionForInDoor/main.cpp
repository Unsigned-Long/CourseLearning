#include "handler.h"

int main(int argc, char const *argv[])
{
    // load the directory og the configure files and read the info
    ns_res::ConfigInit::init("../configs");
    // output the info
    ns_res::ConfigInit::info();
    // calculate the pos
    ns_res::Handler::process();
    return 0;
}
