//
// Created by csl on 9/18/22.
//

#ifndef SPP_DATA_PARSER_H
#define SPP_DATA_PARSER_H

#include "config.h"
#include "datetime.h"

namespace ns_spp {

#define CRC32_POLYNOMIAL 0xEDB88320U

    struct CRC32 {
        /*
         * calculates the CRC-32 of a block of data all at once
         */
        static unsigned int calBuffCRC32(const Byte *buff, int len);

        static bool checkBuff(const Byte *buff, int len, unsigned int tarCRC32);
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

    struct ASCIIMessageHeader {
    public:
        Char message;
        Char port;
        Long sequence;
        Float idleTime;
        TimeStatus timeStatus;
        ULong week;
        Float seconds;
        ULong receiverStatus;
        ULong reserved;
        ULong receiverVersion;
    };

    struct BinaryMessageHeader {
    public:
        UChar headerLength;
        UShort messageID;
        Char messageType;
        UChar portAddress;
        UShort messageLength;
        UShort sequence;
        UChar idleTime;
        TimeStatus timeStatus;
        UShort week;
        Long ms;
        ULong receiverStatus;
        UShort reserved;
        UShort receiverVersion;
    };


}

#endif //SPP_DATA_PARSER_H
