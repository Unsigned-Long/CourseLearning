//
// Created by csl on 9/18/22.
//

#ifndef SPP_DATA_PARSER_H
#define SPP_DATA_PARSER_H

#include <ostream>
#include "config.h"
#include "datetime.h"
#include "boost/preprocessor.hpp"
#include "utils/base_cast.hpp"

namespace ns_spp {
// enum to tuple
#define TUPLE_WITHOUT_ZERO(count, name) \
BOOST_PP_TUPLE_POP_FRONT((BOOST_PP_ENUM_PARAMS(count, name),name##count))

// tuple to seq
#define SEQ_WITHOUT_ZERO(count, name) \
BOOST_PP_TUPLE_TO_SEQ(TUPLE_WITHOUT_ZERO(count,name))

// seq to enum
#define ENUM_WITHOUT_ZERO(count, name) \
BOOST_PP_SEQ_ENUM(SEQ_WITHOUT_ZERO(count, name))

// append
#define APPEND(r, data, elem) \
BOOST_PP_CAT(elem, data)

// add tail
#define SEQ_WITH_TAIL_WITHOUT_ZERO(count, name, tail) \
BOOST_PP_SEQ_TRANSFORM(APPEND, tail, SEQ_WITHOUT_ZERO(count, name))

// seq to enum
#define ENUM_WITH_TAIL_WITHOUT_ZERO(count, name, tail) \
BOOST_PP_SEQ_ENUM(SEQ_WITH_TAIL_WITHOUT_ZERO(count, name, tail))

#define CRC32_POLYNOMIAL 0xEDB88320U

    struct CRC32 {
        /*
         * calculates the CRC-32 of a block of data all at once
         */
        static unsigned int calBuffCRC32(const Byte *buff, std::size_t len);

        static bool checkBuff(const Byte *buff, std::size_t len, unsigned int tarCRC32);
    };

    enum class TimeStatus : UChar {
        // Time validity is unknown
        APPROXIMATE = 60,

        // Time is set approximately
        UNKNOWN = 20,

        // Time is approaching coarse precision
        COARSE = 100,

        // This time is valid to coarse precision
        COARSEADJUSTING = 80,

        // Time is coarse set and is being steered
        COARSESTEERING = 120,

        // Position is lost and the range bias cannot be calculated
        FREEWHEELING = 130,

        // Time is adjusting to fine precision
        FINEADJUSTING = 140,

        // Time has fine precision
        FINE = 160,

        // Time is fine set and is being steered by the backup system
        FINEBACKUPSTEERING = 170,

        // Time is fine set and is being steered
        FINESTEERING = 180,

        // Time from satellite. Only used in logs containing satellite data such as ephemeris and almanac
        SATTIME = 200
    };

    static std::ostream &operator<<(std::ostream &os, const TimeStatus &timeStatus) {
        os << EnumCast::enumToString(timeStatus);
        return os;
    }

    enum class PortIdentifier : UChar {
        NO_PORTS = 0,
        ENUM_WITH_TAIL_WITHOUT_ZERO(3, COM, _ALL),
        THISPORT_ALL = 6,
        FILE_ALL,
        ALL_PORTS,
        USB1_ALL = 13,
        USB2_ALL,
        USB3_ALL,
        AUX_ALL,
        COM4_ALL = 19,
        ETH1_ALL,
        IMU_ALL,
        ICOM1_ALL = 23,
        ICOM2_ALL,
        ICOM3_ALL,
        ENUM_WITH_TAIL_WITHOUT_ZERO(3, NCOM, _ALL),
        ICOM4_ALL,
        WCOM1_ALL,
        COM1 = 32,
        ENUM_WITHOUT_ZERO(31, COM1_),
        COM2,
        ENUM_WITHOUT_ZERO(31, COM2_),
        COM3,
        ENUM_WITHOUT_ZERO(31, COM3_),
        SPECIAL = 160,
        ENUM_WITHOUT_ZERO(31, SPECIAL_),
        THISPORT = 192,
        ENUM_WITHOUT_ZERO(31, THISPORT_),
        FILE = 224,
        ENUM_WITHOUT_ZERO(31, FILE_)
    };

    static std::ostream &operator<<(std::ostream &os, const PortIdentifier &portIdentifier) {
        os << EnumCast::enumToString(portIdentifier);
        return os;
    }

    enum MessageID : UShort {
        // Satellite range information
        RANGE = 43,
        // Decoded GPS L1 C/A ephemerides
        GPSEPHEM = 7,
        // Decoded BDS ephemeris
        BDSEPHEMERIS = 1696,

        // Best position
        BESTPOS = 42,
        // Position averaging
        AVEPOS = 172,
        // PDP filter position
        PDPPOS = 469,
        // Pseudorange position
        PSRPOS = 47
    };

    struct BinaryMessageHeader {
    public:
        UChar firSync;
        UChar sedSync;
        UChar trdSync;
        UChar headerLength;
        MessageID messageID;
        Char messageType;
        PortIdentifier portAddress;
        UShort messageLength;
        UShort sequence;
        UChar idleTime;
        TimeStatus timeStatus;
        UShort week;
        Long ms;
        ULong receiverStatus;
        UShort reserved;
        UShort receiverVersion;

    public:
        BinaryMessageHeader() {}

        static BinaryMessageHeader parsing(const Byte *buffer, std::size_t len);

        friend std::ostream &operator<<(std::ostream &os, const BinaryMessageHeader &header) {
            os << "firSync: " << BaseCast::decTo<16>(header.firSync)
               << " sedSync: " << BaseCast::decTo<16>(header.sedSync)
               << " trdSync: " << BaseCast::decTo<16>(header.trdSync)
               << " headerLength: " << static_cast<unsigned int>(header.headerLength)
               << " messageID: " << header.messageID
               << " messageType: " << static_cast<unsigned int>(header.messageType)
               << " portAddress: " << header.portAddress
               << " messageLength: " << header.messageLength
               << " sequence: " << header.sequence
               << " idleTime: " << static_cast<unsigned int>(header.idleTime)
               << " timeStatus: " << header.timeStatus
               << " week: " << header.week << " ms: " << header.ms
               << " receiverStatus: " << header.receiverStatus
               << " reserved: " << header.reserved
               << " receiverVersion: " << header.receiverVersion;
            return os;
        }
    };


    struct MessageItem {
    public:
        BinaryMessageHeader header;

        explicit MessageItem(const BinaryMessageHeader &header);

        virtual void parseMessage(const Byte *buffer, std::size_t len) = 0;
    };

    struct RANGEMessage : public MessageItem {
    public:
        explicit RANGEMessage(const BinaryMessageHeader &header);

    protected:
        void parseMessage(const Byte *buffer, std::size_t len) override;
    };

    struct GPSEPHEMMessage : public MessageItem {
    public:
        explicit GPSEPHEMMessage(const BinaryMessageHeader &header);

    protected:
        void parseMessage(const Byte *buffer, std::size_t len) override;
    };

    struct BDSEPHEMERISMessage : public MessageItem {
    public:
        explicit BDSEPHEMERISMessage(const BinaryMessageHeader &header);

    protected:
        void parseMessage(const Byte *buffer, std::size_t len) override;
    };

    struct BESTPOSMessage : public MessageItem {
    public:
        explicit BESTPOSMessage(const BinaryMessageHeader &header);

    protected:
        void parseMessage(const Byte *buffer, std::size_t len) override;
    };

    class NovAtelOEM {
    private:
        Byte *buffer;
        std::size_t bufferSize;

        const static Byte firSync = 0xAA;
        const static Byte sedSync = 0x44;
        const static Byte trdSync = 0x12;

    public:
        explicit NovAtelOEM(const std::string &binFilePath);

        virtual ~NovAtelOEM();

        [[nodiscard]] const Byte *getBuffer() const;

    };

}

#endif //SPP_DATA_PARSER_H
