//
// Created by csl on 9/18/22.
//

#ifndef SPP_TYPE_DEF_HPP
#define SPP_TYPE_DEF_HPP

#include <boost/multiprecision/cpp_dec_float.hpp>

namespace ns_spp {

    using BigDouble = boost::multiprecision::cpp_dec_float_50;
    using Byte = unsigned char;

    using Char = char;
    using UChar = unsigned char;
    using Short = short;
    using UShort = unsigned short;
    using Long = int;
    using ULong = unsigned int;
    using Double = double;
    using Float = float;
    using String = std::string;
}

#endif //SPP_TYPE_DEF_HPP
