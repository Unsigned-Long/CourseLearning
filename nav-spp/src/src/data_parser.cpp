//
// Created by csl on 9/18/22.
//
#include "data_parser.h"

unsigned int ns_spp::CRC32::calBuffCRC32(const ns_spp::Byte *buff, std::size_t len) {
    int i, j;
    unsigned int crc = 0;
    for (i = 0; i < len; i++) {
        crc ^= buff[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ CRC32_POLYNOMIAL;
            else crc >>= 1;
        }
    }
    return crc;
}

bool ns_spp::CRC32::checkBuff(const ns_spp::Byte *buff, std::size_t len, unsigned int tarCRC32) {
    return calBuffCRC32(buff, len) == tarCRC32;
}

ns_spp::ASCIIMessageHeader ns_spp::ASCIIMessageHeader::parsing(const ns_spp::Byte *buff, std::size_t len) {
    return ns_spp::ASCIIMessageHeader();
}

ns_spp::BinaryMessageHeader ns_spp::BinaryMessageHeader::parsing(const ns_spp::Byte *buff, std::size_t len) {
    return ns_spp::BinaryMessageHeader();
}
