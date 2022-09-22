//
// Created by csl on 9/22/22.
//

#ifndef SPP_BUFFER_HPP
#define SPP_BUFFER_HPP

#include "type_def.hpp"
#include "fstream"

namespace ns_spp {

    struct BufferHelper {

        template<class Type>
        static Type fromByte(const Byte *buffer) {
            Type val = *((Type *) buffer);
            return val;
        }

        static std::size_t readBuffer(Byte **buffer, const std::string &binFilePath) {
            std::ifstream file(binFilePath, std::ios::in | std::ios::binary);
            if (!file.is_open()) {
                return 0;
            }
            std::size_t bufferSize;
            // count the float values' num
            file.seekg(0, std::ios::end);
            bufferSize = file.tellg() / sizeof(Byte);
            file.seekg(0, std::ios::beg);

            delete[](*buffer);
            *buffer = new Byte[bufferSize];

            // read float buffer
            file.read(reinterpret_cast<char *>(*buffer), bufferSize * sizeof(Byte));

            file.close();

            return bufferSize;
        }
    };

}

#endif //SPP_BUFFER_HPP
