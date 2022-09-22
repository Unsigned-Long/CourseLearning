//
// Created by csl on 9/18/22.
//
#include "data_parser.h"
#include "fstream"
#include "artwork/logger/logger.h"
#include "utils/buffer.hpp"

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

ns_spp::BinaryMessageHeader ns_spp::BinaryMessageHeader::parsing(const ns_spp::Byte *buffer, std::size_t len) {
    using namespace ns_spp;

    ns_spp::BinaryMessageHeader header{};

    header.firSync = buffer[0], header.sedSync = buffer[1], header.trdSync = buffer[2];

    header.headerLength = buffer[3];

    header.messageID = static_cast<MessageID>(BufferHelper::fromByte<UShort>(buffer + 4));

    header.messageType = BufferHelper::fromByte<Char>(buffer + 6);

    header.portAddress = EnumCast::integerToEnum<PortIdentifier>(BufferHelper::fromByte<UChar>(buffer + 7));

    header.messageLength = BufferHelper::fromByte<UShort>(buffer + 8);

    header.sequence = BufferHelper::fromByte<UShort>(buffer + 10);

    header.idleTime = BufferHelper::fromByte<UChar>(buffer + 12);

    header.timeStatus = EnumCast::integerToEnum<TimeStatus>(BufferHelper::fromByte<UChar>(buffer + 13));

    header.week = BufferHelper::fromByte<UShort>(buffer + 14);

    header.ms = BufferHelper::fromByte<Long>(buffer + 16);

    header.receiverStatus = BufferHelper::fromByte<ULong>(buffer + 20);

    header.reserved = BufferHelper::fromByte<UShort>(buffer + 24);

    header.receiverVersion = BufferHelper::fromByte<UShort>(buffer + 26);

    return header;
}


ns_spp::NovAtelOEM::~NovAtelOEM() {
    delete[] this->buffer;
}

const ns_spp::Byte *ns_spp::NovAtelOEM::getBuffer() const {
    return buffer;
}

ns_spp::NovAtelOEM::NovAtelOEM(const std::string &binFilePath) : buffer(nullptr) {
    using namespace ns_spp;
    this->bufferSize = BufferHelper::readBuffer(&this->buffer, binFilePath);

    for (int i = 0; i != bufferSize - 3;) {
        LOG_VAR(i);
        auto firByte = buffer[i + 0], sedByte = buffer[i + 1], trdByte = buffer[i + 2];
        if (firByte == firSync && sedByte == sedSync && trdByte == trdSync) {
            BinaryMessageHeader header = BinaryMessageHeader::parsing(buffer + i, buffer[i + 3]);
            LOG_VAR(header);
            auto dataSize = header.headerLength + header.messageLength;
            auto crcCode = BufferHelper::fromByte<ULong>(buffer + i + dataSize);
            if (!CRC32::checkBuff(buffer + i, dataSize, crcCode)) { continue; }
            LOG_VAR("check pass!");
            i += dataSize + 4;
        } else {
            ++i;
        }
        std::cin.get();
    }
}

ns_spp::MessageItem::MessageItem(const ns_spp::BinaryMessageHeader &header) {

}

ns_spp::RANGEMessage::RANGEMessage(const ns_spp::BinaryMessageHeader &header)
        : MessageItem(header) {}

void ns_spp::RANGEMessage::parseMessage(const ns_spp::Byte *buffer, std::size_t len) {

}

ns_spp::GPSEPHEMMessage::GPSEPHEMMessage(const ns_spp::BinaryMessageHeader &header)
        : MessageItem(header) {}

void ns_spp::GPSEPHEMMessage::parseMessage(const ns_spp::Byte *buffer, std::size_t len) {

}

ns_spp::BDSEPHEMERISMessage::BDSEPHEMERISMessage(const ns_spp::BinaryMessageHeader &header)
        : MessageItem(header) {}

void ns_spp::BDSEPHEMERISMessage::parseMessage(const ns_spp::Byte *buffer, std::size_t len) {

}

ns_spp::BESTPOSMessage::BESTPOSMessage(const ns_spp::BinaryMessageHeader &header)
        : MessageItem(header) {}

void ns_spp::BESTPOSMessage::parseMessage(const ns_spp::Byte *buffer, std::size_t len) {

}
