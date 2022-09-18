//
// Created by csl on 9/18/22.
//
#include "data_parser.h"

unsigned int ns_spp::CRC32::calBuffCRC32(const ns_spp::Byte *buff, int len) {
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

bool ns_spp::CRC32::checkBuff(const ns_spp::Byte *buff, int len, unsigned int tarCRC32) {
    return calBuffCRC32(buff, len) == tarCRC32;
}
