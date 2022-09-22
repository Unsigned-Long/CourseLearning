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

ns_spp::MessageHeader ns_spp::MessageHeader::parsing(const ns_spp::Byte *buffer, std::size_t len) {
    using namespace ns_spp;

    ns_spp::MessageHeader header{};

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

/*
 * MessageItem
 */
ns_spp::MessageItem::MessageItem(const ns_spp::MessageHeader &header)
        : header(header) {}

std::shared_ptr<ns_spp::MessageItem>
ns_spp::MessageItem::tryParseMessage(const ns_spp::Byte *buffer,
                                     std::size_t *bytesUsed) {
    MessageHeader header = MessageHeader::parsing(buffer, buffer[3]);

    auto dataSize = header.headerLength + header.messageLength;
    LOG_VAR(header);
    if (bytesUsed != nullptr) {
        *bytesUsed = dataSize + 4;
    }

    auto crcCode = BufferHelper::fromByte<ULong>(buffer + dataSize);
    if (!CRC32::checkBuff(buffer, dataSize, crcCode)) {
        return nullptr;
    }
    std::shared_ptr<MessageItem> messageItem = nullptr;

    switch (header.messageID) {
        case RANGE:
            messageItem = std::make_shared<RANGEMessage>(header);
            break;
        case GPSEPHEM:
            messageItem = std::make_shared<GPSEPHEMMessage>(header);
            break;
        case BDSEPHEMERIS:
            messageItem = std::make_shared<BDSEPHEMERISMessage>(header);
            break;
        case BESTPOS:
            messageItem = std::make_shared<BESTPOSMessage>(header);
            break;
        default:
            break;
    }
    if (messageItem != nullptr) {
        messageItem->parseMessage(buffer + header.headerLength, header.messageLength);
    }
    return messageItem;
}

std::size_t ns_spp::MessageItem::messageItemSize() const {
    return this->header.headerLength + header.messageLength + 4;
}

/*
 * RANGEMessage
 */
ns_spp::RANGEMessage::RANGEMessage(const ns_spp::MessageHeader &header)
        : MessageItem(header) {}

void ns_spp::RANGEMessage::parseMessage(const ns_spp::Byte *buffer, std::size_t len) {

}

/*
 * GPSEPHEMMessage
 */
ns_spp::GPSEPHEMMessage::GPSEPHEMMessage(const ns_spp::MessageHeader &header)
        : MessageItem(header) {}

void ns_spp::GPSEPHEMMessage::parseMessage(const ns_spp::Byte *buffer, std::size_t len) {

}

/*
 * BDSEPHEMERISMessage
 */
ns_spp::BDSEPHEMERISMessage::BDSEPHEMERISMessage(const ns_spp::MessageHeader &header)
        : MessageItem(header) {}

void ns_spp::BDSEPHEMERISMessage::parseMessage(const ns_spp::Byte *buffer, std::size_t len) {

}

/*
 * BESTPOSMessage
 */
ns_spp::BESTPOSMessage::BESTPOSMessage(const ns_spp::MessageHeader &header)
        : MessageItem(header) {}

void ns_spp::BESTPOSMessage::parseMessage(const ns_spp::Byte *buffer, std::size_t len) {

}

