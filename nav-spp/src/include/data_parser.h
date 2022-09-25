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
        // BESTPOS = 42,
        // Position averaging
        // AVEPOS = 172,
        // PDP filter position
        // PDPPOS = 469,
        // Pseudorange position
        PSRPOS = 47
    };

    struct MessageHeader {
    public:
        const static Byte firSync = 0xAA;
        const static Byte sedSync = 0x44;
        const static Byte trdSync = 0x12;

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
        MessageHeader() {}

        static MessageHeader parsing(const Byte *buffer, std::size_t len);

        friend std::ostream &operator<<(std::ostream &os, const MessageHeader &header) {
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

    /*
     * MessageItem
     */

    struct MessageItem {
    public:
        MessageHeader header;

        MessageItem(const MessageHeader &header);

        virtual void parseMessage(const Byte *buffer, std::size_t len) = 0;

        std::size_t messageItemSize() const;

        static std::shared_ptr<MessageItem>
        tryParseMessage(const Byte *buffer, std::size_t *bytesUsed = nullptr);

    };

    struct ChannelTrackingStatus {
    public:
        enum class TrackingState {
            IDLE = 0,
            SKY_SEARCH,
            WIDE_FREQUENCY_BAND_PULL_IN,
            NARROW_FREQUENCY_BAND_PULL_IN,
            PHASE_LOCK_LOOP,
            CHANNEL_STEERING = 6,
            FREQUENCY_LOCK_LOOP,
            CHANNEL_ALIGNMENT = 9,
            CODE_SEARCH,
            AIDED_PHASE_LOCK_LOOP,
            SIDE_PEAK_DETECTION = 23,
            FFT_SKY_SEARCH
        };
        enum class LockFlag {
            NOT_LOCKED = 0,
            LOCKED
        };
        enum class KnownFlag {
            NOT_KNOWN = 0,
            KNOWN
        };
        enum class CorrelatorType {
            NA = 0,

            // spacing = 1 chip
            STANDARD_CORRELATOR,

            // spacing < 1 chip
            NARROW_CORRELATOR,
            NONE_1,
            // Pulse Aperture Correlator
            PAC,
            NARROW_PAC,
            NONE_2
        };
        enum class SatSystem {
            GPS = 0, GLONASS, SBAS, GALILEO, BEI_DOU, QZSS, NAV_IC, OTHER
        };
        enum class GroupingFlag {
            NOT_GROUPED = 0, GROUPED
        };
        enum class PrimaryFlag {
            NOT_PRIMARY = 0, PRIMARY
        };
        enum class CarrierPhaseHalfCycleFlag {
            NOT_ADDED = 0, ADDED
        };
        enum class DigitalFilteringFlag {
            NO_DIGITAL_FILTER = 0, DIGITAL_FILTER
        };
        enum class ChannelAssignmentFlag {
            AUTOMATIC = 0, FORCED
        };
    public:
        ULong data;

        TrackingState trackingState;
        UShort SVChannelNumber;
        LockFlag phaseLock;
        KnownFlag parityKnown;
        LockFlag codeLock;
        CorrelatorType correlatorType;
        SatSystem satSystem;

        GroupingFlag grouping;
        UShort signalType;
        PrimaryFlag primaryL1Channel;
        CarrierPhaseHalfCycleFlag carrierPhaseHalfCycle;
        DigitalFilteringFlag digitalFiltering;
        LockFlag PRNLock;
        ChannelAssignmentFlag channelAssignment;

        ChannelTrackingStatus();

        ChannelTrackingStatus(ULong data);

        friend std::ostream &operator<<(std::ostream &os, const ChannelTrackingStatus &status) {
            os << "data: " << status.data
               << " trackingState: " << EnumCast::enumToString(status.trackingState)
               << " SVChannelNumber: " << status.SVChannelNumber
               << " phaseLock: " << EnumCast::enumToString(status.phaseLock)
               << " parityKnown: " << EnumCast::enumToString(status.parityKnown)
               << " codeLock: " << EnumCast::enumToString(status.codeLock)
               << " correlatorType: " << EnumCast::enumToString(status.correlatorType)
               << " satSystem: " << EnumCast::enumToString(status.satSystem)
               << " grouping: " << EnumCast::enumToString(status.grouping)
               << " signalType: " << status.signalType
               << " primaryL1Channel: " << EnumCast::enumToString(status.primaryL1Channel)
               << " carrierPhaseHalfCycle: " << EnumCast::enumToString(status.carrierPhaseHalfCycle)
               << " digitalFiltering: " << EnumCast::enumToString(status.digitalFiltering)
               << " PRNLock: " << EnumCast::enumToString(status.PRNLock)
               << " channelAssignment: " << EnumCast::enumToString(status.channelAssignment);
            return os;
        }
    };

    struct RANGEMessage : public MessageItem {
    public:

        struct Observation {
            // Satellite PRN number of range measurement
            UShort PRN;
            // GLONASS Frequency + 7
            UShort gloFreq;
            // Pseudorange measurement (m)
            Double psr;
            // Pseudorange measurement standard deviation (m)
            Float psrSigma;
            // Carrier phase, in cycles (accumulated Doppler range)
            Double adr;
            // Estimated carrier phase standard deviation (cycles)
            Float adrSigma;
            // Instantaneous carrier Doppler frequency (Hz)
            Float dopp;
            // Carrier to noise density ratio C/No = 10[log10(S/N0)] (dB-Hz)
            Float C_No;
            // Number of seconds of continuous tracking (no cycle slipping)
            Float lockTime;
            // Tracking status
            ChannelTrackingStatus channelTrackingStatus;

            friend std::ostream &operator<<(std::ostream &os, const Observation &observation) {
                os << "PRN: " << observation.PRN << " gloFreq: " << observation.gloFreq
                   << " psr: " << observation.psr << " psrSigma: " << observation.psrSigma
                   << " adr: " << observation.adr << " adrSigma: " << observation.adrSigma
                   << " dopp: " << observation.dopp << " C_No: " << observation.C_No
                   << " lockTime: " << observation.lockTime << " ch_tr_status: " << observation.channelTrackingStatus;
                return os;
            }
        };

        // Number of observations with information
        ULong obsNum;
        std::vector<Observation> obsVec;

    public:
        explicit RANGEMessage(const MessageHeader &header);

    protected:
        void parseMessage(const Byte *buffer, std::size_t len) override;
    };

    struct GPSEPHEMMessage : public MessageItem {
    public:
        struct Ephemeris {
            // Satellite PRN number
            ULong PRN;
            // Time stamp of subframe 1 (s)
            Double tow;
            // Health status - a 6-bit health code
            ULong health;
            // Issue of ephemeris data 1
            ULong IODE1;
            // Issue of ephemeris data 2
            ULong IODE2;
            // toe week number (computed from Z count week)
            ULong week;
            // Z count week number, This is the week number from subframe 1 of the ephemeris
            ULong zWeek;
            // Reference time for ephemeris (s)
            Double toe;
            // Semi-major axis (m)
            Double A;
            // Mean motion difference (radians/s)
            Double DeltaN;
            // Mean anomaly of reference time (radians)
            Double M_0;
            // Eccentricity, dimensionless
            Double ecc;
            // Argument of perigee (radians)
            Double omega;
            // Amplitude of cosine harmonic correction term to the argument of latitude (radians)
            Double cuc;
            // Amplitude of sine harmonic correction term to the argument of latitude (radians)
            Double cus;
            // Amplitude of cosine harmonic correction term to the orbit radius (m)
            Double crc;
            // Amplitude of sine harmonic correction term to the orbit radius (m)
            Double crs;
            // Amplitude of cosine harmonic correction term to the angle of inclination (radians)
            Double cic;
            // Amplitude of sine harmonic correction term to the angle of inclination (radians)
            Double cis;
            // Inclination angle at reference time (radians)
            Double i_0;
            // Rate of inclination angle (radians/s)
            Double i_u0;
            // Right ascension (radians)
            Double omega_0;
            // Rate of right ascension (radians/s)
            Double omegaDot;
            // Issue of data clock
            ULong iodc;
            // SV clock correction term (s)
            Double toc;
            // Estimated group delay difference (s)
            Double tgd;
            // Clock aging parameter (s)
            Double a_f0;
            // Clock aging parameter (s/s)
            Double a_f1;
            // Clock aging parameter (s/s/s)
            Double a_f2;
            // Anti-spoofing on: FALSE(0); TRUE(1);
            Bool AS;
            // Corrected mean motion (radians/s)
            Double N;
            // User Range Accuracy variance (m)^2
            Double URA;

            friend std::ostream &operator<<(std::ostream &os, const Ephemeris &ephemeris) {
                os << "PRN: " << ephemeris.PRN << " tow: " << ephemeris.tow
                   << " health: " << ephemeris.health << " IODE1: " << ephemeris.IODE1
                   << " IODE2: " << ephemeris.IODE2 << " week: " << ephemeris.week
                   << " zWeek: " << ephemeris.zWeek << " toe: " << ephemeris.toe
                   << " A: " << ephemeris.A << " DeltaN: " << ephemeris.DeltaN
                   << " M_0: " << ephemeris.M_0 << " ecc: " << ephemeris.ecc << " omega: " << ephemeris.omega
                   << " cuc: " << ephemeris.cuc << " cus: " << ephemeris.cus
                   << " crc: " << ephemeris.crc << " crs: " << ephemeris.crs
                   << " cic: " << ephemeris.cic << " cis: " << ephemeris.cis
                   << " i_0: " << ephemeris.i_0 << " i_u0: " << ephemeris.i_u0
                   << " omega_0: " << ephemeris.omega_0 << " omegaDot: " << ephemeris.omegaDot
                   << " iodc: " << ephemeris.iodc << " toc: " << ephemeris.toc
                   << " tgd: " << ephemeris.tgd << " a_f0: " << ephemeris.a_f0
                   << " a_f1: " << ephemeris.a_f1 << " a_f2: " << ephemeris.a_f2
                   << " AS: " << ephemeris.AS << " N: " << ephemeris.N << " URA: " << ephemeris.URA;
                return os;
            }
        } ephem;

    public:
        explicit GPSEPHEMMessage(const MessageHeader &header);

    protected:
        void parseMessage(const Byte *buffer, std::size_t len) override;
    };

    struct BDSEPHEMERISMessage : public MessageItem {
    public:
        struct Ephemeris {
            // ID/ranging code
            ULong satID;
            // BeiDou week number
            ULong week;
            // User range accuracy (m). This is the evaluated URAI/URA lookup-table value.
            Double URA;
            // Autonomous satellite health flag. 0 means broadcasting satellite is good and 1 means not.
            ULong health;
            // Equipment group delay differential for the B1 signal (s)
            Double tgd1;
            // Equipment group delay differential for the B2 signal (s)
            Double tgd2;
            // Age of data, clock
            ULong AODC;
            // Reference time of clock parameters (s)
            ULong toc;
            // Constant term of clock correction polynomial (s)
            Double a_0;
            // Linear term of clock correction polynomial (s/s)
            Double a_1;
            // Quadratic term of clock correction polynomial (s/s^2)
            Double a_2;
            // Age of data, ephemeris
            ULong AODE;
            // Reference time of ephemeris parameters (s)
            ULong toe;
            // Square root of semi-major axis (sqrt(m))
            Double rootA;
            // Eccentricity (dimensionless)
            Double ecc;
            // Argument of perigee (radians)
            Double omega;
            // Mean motion difference from computed value (radians/s)
            Double DeltaN;
            // Mean anomaly at reference time (radians)
            Double M_0;
            // Longitude of ascending node of orbital of plane computed according to reference time (radians)
            Double Omega_0;
            // Rate of right ascension (radians/s)
            Double OmegaDot;
            // Inclination angle at reference time (radians)
            Double i_0;
            // Rate of inclination angle (radians/second)
            Double IDOT;
            // Amplitude of cosine harmonic correction term to the argument of latitude (radians)
            Double cuc;
            // Amplitude of sine harmonic correction term to the argument of latitude (radians)
            Double cus;
            // Amplitude of cosine harmonic correction term to the orbit radius (m)
            Double crc;
            // Amplitude of sine harmonic correction term to the orbit radius (m)
            Double crs;
            // Amplitude of cosine harmonic correction term to the angle of inclination (radians)
            Double cic;
            // Amplitude of sine harmonic correction term to the angle of inclination (radians)
            Double cis;

            friend std::ostream &operator<<(std::ostream &os, const Ephemeris &ephemeris) {
                os << "satID: " << ephemeris.satID << " week: " << ephemeris.week
                   << " URA: " << ephemeris.URA << " health: " << ephemeris.health
                   << " tgd1: " << ephemeris.tgd1 << " tgd2: " << ephemeris.tgd2
                   << " AODC: " << ephemeris.AODC << " toc: " << ephemeris.toc
                   << " a_0: " << ephemeris.a_0 << " a_1: " << ephemeris.a_1
                   << " a_2: " << ephemeris.a_2 << " AODE: " << ephemeris.AODE
                   << " toe: " << ephemeris.toe << " rootA: " << ephemeris.rootA
                   << " ecc: " << ephemeris.ecc << " omega: " << ephemeris.omega
                   << " DeltaN: " << ephemeris.DeltaN << " M_0: " << ephemeris.M_0
                   << " Omega_0: " << ephemeris.Omega_0 << " OmegaDot: " << ephemeris.OmegaDot
                   << " i_0: " << ephemeris.i_0 << " IDOT: " << ephemeris.IDOT
                   << " cuc: " << ephemeris.cuc << " cus: " << ephemeris.cus
                   << " crc: " << ephemeris.crc << " crs: " << ephemeris.crs
                   << " cic: " << ephemeris.cic << " cis: " << ephemeris.cis;
                return os;
            }
        } ephem;

    public:
        explicit BDSEPHEMERISMessage(const MessageHeader &header);

    protected:
        void parseMessage(const Byte *buffer, std::size_t len) override;
    };

    struct PSRPOSMessage : public MessageItem {
    public:
        struct PSRPOS {
            // Solution status
            ULong solStatus;
            // Position type
            ULong posType;
            // Latitude (degrees)
            Double lat;
            // Longitude (degrees)
            Double lon;
            // Height above mean sea level (m)
            Double height;
            // Undulation - the relationship between the geoid and the WGS84 ellipsoid (m) 1
            Float undulation;
            // Datum ID number: WGS84(61); USER(63)
            ULong datumID;
            // Latitude standard deviation (m)
            Float latSigma;
            // Longitude standard deviation (m)
            Float lonSigma;
            // Height standard deviation (m)
            Float heightSigma;
            // Base station ID
            Char stnID[4];
            // Differential age in seconds
            Float diffAge;
            // Solution age in seconds
            Float solAge;
            // Number of satellites tracked
            UChar SVs;
            // Number of satellites used in solution
            UChar solnSvs;

            struct Reserved {
                UChar field0;
                UChar field1;
                Byte field2;

                friend std::ostream &operator<<(std::ostream &os, const Reserved &reserved) {
                    os << "field0: " << static_cast<unsigned int>(reserved.field0)
                       << " field1: " << static_cast<unsigned int>(reserved.field1)
                       << " field2: " << BaseCast::decTo<16>(reserved.field2);
                    return os;
                }
            } reserved;

            // Extended solution status
            Byte EXT_SOL_STATUS;
            // Galileo and BeiDou signals used mask
            Byte GALILEO_BEIDOU_SIG_MASK;
            // GPS and GLONASS signals used mask
            Byte GPS_GLONASS_SIG_MASK;

            friend std::ostream &operator<<(std::ostream &os, const PSRPOS &psrpos) {
                os << "solStatus: " << psrpos.solStatus << " posType: " << psrpos.posType
                   << " lat: " << psrpos.lat << " lon: " << psrpos.lon << " height: " << psrpos.height
                   << " undulation: " << psrpos.undulation << " datumID: " << psrpos.datumID
                   << " latSigma: " << psrpos.latSigma << " lonSigma: " << psrpos.lonSigma
                   << " heightSigma: " << psrpos.heightSigma
                   << " stnID: " << static_cast<int>(psrpos.stnID[0]) << ':' << static_cast<int>(psrpos.stnID[1])
                   << ':' << static_cast<int>(psrpos.stnID[2]) << ':' << static_cast<int>(psrpos.stnID[3])
                   << " diffAge: " << psrpos.diffAge << " solAge: " << psrpos.solAge
                   << " SVs: " << static_cast<unsigned int>(psrpos.SVs)
                   << " solnSvs: " << static_cast<unsigned int>(psrpos.solnSvs)
                   << " reserved: " << psrpos.reserved
                   << " EXT_SOL_STATUS: " << BaseCast::decTo<16>(psrpos.EXT_SOL_STATUS)
                   << " GALILEO_BEIDOU_SIG_MASK: " << BaseCast::decTo<16>(psrpos.GALILEO_BEIDOU_SIG_MASK)
                   << " GPS_GLONASS_SIG_MASK: " << BaseCast::decTo<16>(psrpos.GPS_GLONASS_SIG_MASK);
                return os;
            }
        } psrpos;

    public:
        explicit PSRPOSMessage(const MessageHeader &header);

    protected:
        void parseMessage(const Byte *buffer, std::size_t len) override;
    };


}

#endif //SPP_DATA_PARSER_H
