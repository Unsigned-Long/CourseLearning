//
// Created by csl on 9/18/22.
//

#ifndef SPP_DATA_PARSER_H
#define SPP_DATA_PARSER_H

#include "config.h"
#include "datetime.h"
#include "boost/preprocessor.hpp"

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

    enum class TimeStatus {
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

    enum class PortIdentifier {
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
        ENUM_WITHOUT_ZERO(31, COM3_)
    };

    enum class MessageID : ushort {
        RANGE = 43,
        RANGECMP = 140,

        // The data is stored in the format of three sub-frames in the GPS navigation message,
        // and the navigation message format needs to be referred to when decoding
        RAWEPHEM = 41,
        RAWGPSSUBFRAME = 25,

        // NovAtel decodes and recodes all sub-frames of the navigation message
        GPSEPHEM = 7,
        BDSEPHEMERIS = 1696,

        // Four navigation and positioning results
        BESTPOS = 42,
        AVEPOS = 172,
        PDPPOS = 469,
        PSRPOS = 47
    };

    struct ASCIIMessageHeader {
    public:
        Char sync;
        String message;
        PortIdentifier port;
        Long sequence;
        Float idleTime;
        TimeStatus timeStatus;
        ULong week;
        Float seconds;
        ULong receiverStatus;
        ULong reserved;
        ULong receiverVersion;

    public:
        static ASCIIMessageHeader parsing(const Byte *buff, std::size_t len);

        ASCIIMessageHeader() {}
    };

    struct BinaryMessageHeader {
    public:
        Char firSync;
        Char sedSync;
        Char trdSync;
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

        static BinaryMessageHeader parsing(const Byte *buff, std::size_t len);
    };


}

#endif //SPP_DATA_PARSER_H
