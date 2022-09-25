//
// Created by csl on 9/18/22.
//
#include "data_parser.h"
#include "fstream"
#include "artwork/logger/logger.h"
#include "utils/buffer.hpp"
#include "bitset"

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
            LOG_VAR("--RANGE--");
            break;
        case GPSEPHEM:
            messageItem = std::make_shared<GPSEPHEMMessage>(header);
            LOG_VAR("--GPSEPHEM--");
            break;
        case BDSEPHEMERIS:
            messageItem = std::make_shared<BDSEPHEMERISMessage>(header);
            LOG_VAR("--BDSEPHEMERIS--");
            break;
        case PSRPOS:
            messageItem = std::make_shared<PSRPOSMessage>(header);
            LOG_VAR("--PSRPOS--");
            break;
        default:
            LOG_VAR("Unknown");
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
    using namespace ns_spp;
    this->obsNum = BufferHelper::fromByte<ULong>(buffer);
    this->obsVec.resize(this->obsNum);
    for (int i = 0; i < obsNum; ++i) {
        auto &obs = this->obsVec[i];
        obs.PRN = BufferHelper::fromByte<UShort>(buffer + i * 44 + 4);
        obs.gloFreq = BufferHelper::fromByte<UShort>(buffer + i * 44 + 6);
        obs.psr = BufferHelper::fromByte<Double>(buffer + i * 44 + 8);
        obs.psrSigma = BufferHelper::fromByte<Float>(buffer + i * 44 + 16);
        obs.adr = BufferHelper::fromByte<Double>(buffer + i * 44 + 20);
        obs.adrSigma = BufferHelper::fromByte<Float>(buffer + i * 44 + 28);
        obs.dopp = BufferHelper::fromByte<Float>(buffer + i * 44 + 32);
        obs.C_No = BufferHelper::fromByte<Float>(buffer + i * 44 + 36);
        obs.lockTime = BufferHelper::fromByte<Float>(buffer + i * 44 + 40);
        obs.channelTrackingStatus = ChannelTrackingStatus(
                BufferHelper::fromByte<ULong>(buffer + i * 44 + 44)
        );
    }
}

/*
 * GPSEPHEMMessage
 */
ns_spp::GPSEPHEMMessage::GPSEPHEMMessage(const ns_spp::MessageHeader &header)
        : MessageItem(header) {}

void ns_spp::GPSEPHEMMessage::parseMessage(const ns_spp::Byte *buffer, std::size_t len) {
    this->ephem.PRN = BufferHelper::fromByte<ULong>(buffer);
    this->ephem.tow = BufferHelper::fromByte<Double>(buffer + 4);
    this->ephem.health = BufferHelper::fromByte<ULong>(buffer + 12);

    this->ephem.IODE1 = BufferHelper::fromByte<ULong>(buffer + 16);
    this->ephem.IODE2 = BufferHelper::fromByte<ULong>(buffer + 20);

    this->ephem.week = BufferHelper::fromByte<ULong>(buffer + 24);
    this->ephem.zWeek = BufferHelper::fromByte<ULong>(buffer + 28);

    this->ephem.toe = BufferHelper::fromByte<Double>(buffer + 32);
    this->ephem.A = BufferHelper::fromByte<Double>(buffer + 40);
    this->ephem.DeltaN = BufferHelper::fromByte<Double>(buffer + 48);
    this->ephem.M_0 = BufferHelper::fromByte<Double>(buffer + 56);
    this->ephem.ecc = BufferHelper::fromByte<Double>(buffer + 64);
    this->ephem.omega = BufferHelper::fromByte<Double>(buffer + 72);

    this->ephem.cuc = BufferHelper::fromByte<Double>(buffer + 80);
    this->ephem.cus = BufferHelper::fromByte<Double>(buffer + 88);

    this->ephem.crc = BufferHelper::fromByte<Double>(buffer + 96);
    this->ephem.crs = BufferHelper::fromByte<Double>(buffer + 104);

    this->ephem.cic = BufferHelper::fromByte<Double>(buffer + 112);
    this->ephem.cis = BufferHelper::fromByte<Double>(buffer + 120);

    this->ephem.i_0 = BufferHelper::fromByte<Double>(buffer + 128);
    this->ephem.i_u0 = BufferHelper::fromByte<Double>(buffer + 136);

    this->ephem.omega_0 = BufferHelper::fromByte<Double>(buffer + 144);
    this->ephem.omegaDot = BufferHelper::fromByte<Double>(buffer + 152);

    this->ephem.iodc = BufferHelper::fromByte<ULong>(buffer + 160);

    this->ephem.toc = BufferHelper::fromByte<Double>(buffer + 164);
    this->ephem.tgd = BufferHelper::fromByte<Double>(buffer + 172);

    this->ephem.a_f0 = BufferHelper::fromByte<Double>(buffer + 180);
    this->ephem.a_f1 = BufferHelper::fromByte<Double>(buffer + 188);
    this->ephem.a_f2 = BufferHelper::fromByte<Double>(buffer + 196);

    this->ephem.AS = BufferHelper::fromByte<Bool>(buffer + 204);
    this->ephem.N = BufferHelper::fromByte<Double>(buffer + 208);
    this->ephem.URA = BufferHelper::fromByte<Double>(buffer + 216);
}

/*
 * BDSEPHEMERISMessage
 */
ns_spp::BDSEPHEMERISMessage::BDSEPHEMERISMessage(const ns_spp::MessageHeader &header)
        : MessageItem(header) {}

void ns_spp::BDSEPHEMERISMessage::parseMessage(const ns_spp::Byte *buffer, std::size_t len) {
    this->ephem.satID = BufferHelper::fromByte<ULong>(buffer);
    this->ephem.week = BufferHelper::fromByte<ULong>(buffer + 4);
    this->ephem.URA = BufferHelper::fromByte<Double>(buffer + 8);
    this->ephem.health = BufferHelper::fromByte<ULong>(buffer + 16);

    this->ephem.tgd1 = BufferHelper::fromByte<Double>(buffer + 20);
    this->ephem.tgd2 = BufferHelper::fromByte<Double>(buffer + 28);

    this->ephem.AODC = BufferHelper::fromByte<ULong>(buffer + 36);
    this->ephem.toc = BufferHelper::fromByte<ULong>(buffer + 40);

    this->ephem.a_0 = BufferHelper::fromByte<Double>(buffer + 44);
    this->ephem.a_1 = BufferHelper::fromByte<Double>(buffer + 52);
    this->ephem.a_2 = BufferHelper::fromByte<Double>(buffer + 60);

    this->ephem.AODE = BufferHelper::fromByte<ULong>(buffer + 68);
    this->ephem.toe = BufferHelper::fromByte<ULong>(buffer + 72);

    this->ephem.rootA = BufferHelper::fromByte<Double>(buffer + 76);
    this->ephem.ecc = BufferHelper::fromByte<Double>(buffer + 84);
    this->ephem.omega = BufferHelper::fromByte<Double>(buffer + 92);
    this->ephem.DeltaN = BufferHelper::fromByte<Double>(buffer + 100);
    this->ephem.M_0 = BufferHelper::fromByte<Double>(buffer + 108);

    this->ephem.Omega_0 = BufferHelper::fromByte<Double>(buffer + 116);
    this->ephem.OmegaDot = BufferHelper::fromByte<Double>(buffer + 124);

    this->ephem.i_0 = BufferHelper::fromByte<Double>(buffer + 132);
    this->ephem.IDOT = BufferHelper::fromByte<Double>(buffer + 140);

    this->ephem.cuc = BufferHelper::fromByte<Double>(buffer + 148);
    this->ephem.cus = BufferHelper::fromByte<Double>(buffer + 156);

    this->ephem.crc = BufferHelper::fromByte<Double>(buffer + 164);
    this->ephem.crs = BufferHelper::fromByte<Double>(buffer + 172);

    this->ephem.cic = BufferHelper::fromByte<Double>(buffer + 180);
    this->ephem.cis = BufferHelper::fromByte<Double>(buffer + 188);
}

/*
 * PSRPOSMessage
 */
ns_spp::PSRPOSMessage::PSRPOSMessage(const ns_spp::MessageHeader &header)
        : MessageItem(header) {}

void ns_spp::PSRPOSMessage::parseMessage(const ns_spp::Byte *buffer, std::size_t len) {
    this->psrpos.solStatus = BufferHelper::fromByte<ULong>(buffer);
    this->psrpos.posType = BufferHelper::fromByte<ULong>(buffer + 4);

    this->psrpos.lat = BufferHelper::fromByte<Double>(buffer + 8);
    this->psrpos.lon = BufferHelper::fromByte<Double>(buffer + 16);
    this->psrpos.height = BufferHelper::fromByte<Double>(buffer + 24);

    this->psrpos.undulation = BufferHelper::fromByte<Float>(buffer + 32);
    this->psrpos.datumID = BufferHelper::fromByte<ULong>(buffer + 36);

    this->psrpos.latSigma = BufferHelper::fromByte<Float>(buffer + 40);
    this->psrpos.lonSigma = BufferHelper::fromByte<Float>(buffer + 44);
    this->psrpos.heightSigma = BufferHelper::fromByte<Float>(buffer + 48);

    this->psrpos.stnID[0] = BufferHelper::fromByte<Char>(buffer + 52);
    this->psrpos.stnID[1] = BufferHelper::fromByte<Char>(buffer + 53);
    this->psrpos.stnID[2] = BufferHelper::fromByte<Char>(buffer + 54);
    this->psrpos.stnID[3] = BufferHelper::fromByte<Char>(buffer + 55);

    this->psrpos.diffAge = BufferHelper::fromByte<Float>(buffer + 56);
    this->psrpos.solAge = BufferHelper::fromByte<Float>(buffer + 60);

    this->psrpos.SVs = BufferHelper::fromByte<UChar>(buffer + 64);
    this->psrpos.solnSvs = BufferHelper::fromByte<UChar>(buffer + 65);

    this->psrpos.reserved.field0 = BufferHelper::fromByte<UChar>(buffer + 66);
    this->psrpos.reserved.field1 = BufferHelper::fromByte<UChar>(buffer + 67);
    this->psrpos.reserved.field2 = BufferHelper::fromByte<Byte>(buffer + 68);

    this->psrpos.EXT_SOL_STATUS = BufferHelper::fromByte<Byte>(buffer + 69);
    this->psrpos.GALILEO_BEIDOU_SIG_MASK = BufferHelper::fromByte<Byte>(buffer + 70);
    this->psrpos.GPS_GLONASS_SIG_MASK = BufferHelper::fromByte<Byte>(buffer + 71);

}


ns_spp::ChannelTrackingStatus::ChannelTrackingStatus(ns_spp::ULong data) : data(data) {
    auto val = (data << 27) >> 27;

    this->trackingState = EnumCast::integerToEnum<TrackingState>(val);

    val = ((data << 22) >> 27);
    this->SVChannelNumber = val;

    val = ((data << 21) >> 31);
    this->phaseLock = EnumCast::integerToEnum<LockFlag>(val);

    val = ((data << 20) >> 31);
    this->parityKnown = EnumCast::integerToEnum<KnownFlag>(val);

    val = ((data << 19) >> 31);
    this->codeLock = EnumCast::integerToEnum<LockFlag>(val);

    val = ((data << 16) >> 29);
    this->correlatorType = EnumCast::integerToEnum<CorrelatorType>(val);

    val = ((data << 13) >> 29);
    this->satSystem = EnumCast::integerToEnum<SatSystem>(val);

    val = ((data << 11) >> 31);
    this->grouping = EnumCast::integerToEnum<GroupingFlag>(val);

    val = ((data << 6) >> 27);
    this->signalType = val;

    val = ((data << 4) >> 31);
    this->primaryL1Channel = EnumCast::integerToEnum<PrimaryFlag>(val);

    val = ((data << 3) >> 31);
    this->phaseLock = EnumCast::integerToEnum<LockFlag>(val);

    val = ((data << 2) >> 31);
    this->digitalFiltering = EnumCast::integerToEnum<DigitalFilteringFlag>(val);

    val = ((data << 1) >> 31);
    this->PRNLock = EnumCast::integerToEnum<LockFlag>(val);

    val = (data >> 31);
    this->channelAssignment = EnumCast::integerToEnum<ChannelAssignmentFlag>(val);

}

ns_spp::ChannelTrackingStatus::ChannelTrackingStatus() {}
