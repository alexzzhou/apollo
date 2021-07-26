#pragma once

#include <cstdint>
#include <limits>
#include <string>

#include "modules/drivers/gnss/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {
namespace ublox {

enum MessageId : uint16_t {
  DTM = 0x0A,
  GBQ = 0x44,
  GBS = 0x09,
  GGA = 0x00,
  GLL = 0x01,
  GLQ = 0x43,
  GNQ = 0x42,
  GNS = 0x0D,
  GPQ = 0x40,
  GRS = 0x06,
  GSA = 0x02,
  GST = 0x07,
  GSV = 0x03,
  RMC = 0x04,
  THS = 0x0E,
  TXT = 0x41,
  VLW = 0x0F,
  VTG = 0x05,
  ZDA = 0x08
};

// Every binary message has 32-bit CRC performed on all data including the
// header.
constexpr uint16_t CRC_LENGTH = 4;

#pragma pack(push, 1)  // Turn off struct padding.

struct MessageType {
  enum MessageFormat {
    BINARY = 0b00,
    ASCII = 0b01,
    ABREVIATED_ASCII = 0b10,
    NMEA = 0b11,
  };

  enum ResponseBit {
    ORIGINAL_MESSAGE = 0b0,
    RESPONSE_MESSAGE = 0b1,
  };

  uint8_t reserved : 5;
  MessageFormat format : 2;
  ResponseBit response : 1;
};

struct Header {
    std::string talkerID;
    std::string message;
};

enum class SolutionStatus : uint32_t {
  SOL_COMPUTED = 0,  // solution computed
  INSUFFICIENT_OBS,  // insufficient observations
  NO_CONVERGENCE,    // no convergence
  SINGULARITY,       // singularity at parameters matrix
  COV_TRACE,         // covariance trace exceeds maximum (trace > 1000 m)
  TEST_DIST,   // test distance exceeded (max of 3 rejections if distance > 10
               // km)
  COLD_START,  // not yet converged from cold start
  V_H_LIMIT,   // height or velocity limits exceeded
  VARIANCE,    // variance exceeds limits
  RESIDUALS,   // residuals are too large
  INTEGRITY_WARNING = 13,  // large residuals make position questionable
  PENDING = 18,  // receiver computes its position and determines if the fixed
                 // position is valid
  INVALID_FIX = 19,   // the fixed position entered using the fix position
                      // command is invalid
  UNAUTHORIZED = 20,  // position type is unauthorized
  INVALID_RATE =
      22,  // selected logging rate is not supported for this solution type
  NONE = std::numeric_limits<uint32_t>::max(),
};

enum class SolutionType : uint32_t {
  NONE = 0,
  FIXEDPOS = 1,
  FIXEDHEIGHT = 2,
  FLOATCONV = 4,
  WIDELANE = 5,
  NARROWLANE = 6,
  DOPPLER_VELOCITY = 8,
  SINGLE = 16,
  PSRDIFF = 17,
  WAAS = 18,
  PROPOGATED = 19,
  OMNISTAR = 20,
  L1_FLOAT = 32,
  IONOFREE_FLOAT = 33,
  NARROW_FLOAT = 34,
  L1_INT = 48,
  WIDE_INT = 49,
  NARROW_INT = 50,
  RTK_DIRECT_INS = 51,  // RTK filter is directly initialized
                        // from the INS filter.
  INS_SBAS = 52,
  INS_PSRSP = 53,
  INS_PSRDIFF = 54,
  INS_RTKFLOAT = 55,
  INS_RTKFIXED = 56,
  INS_OMNISTAR = 57,
  INS_OMNISTAR_HP = 58,
  INS_OMNISTAR_XP = 59,
  OMNISTAR_HP = 64,
  OMNISTAR_XP = 65,
  PPP_CONVERGING = 68,
  PPP = 69,
  INS_PPP_CONVERGING = 73,
  INS_PPP = 74,
};

enum class DatumId : uint32_t {
  // We only use WGS-84.
  WGS84 = 61,
};

struct Gbs {
  float time;
  float errLat;
  float errLon;
  float errAlt;
  float svid;
  float prob;
  float bias;
  float stdev;
  uint16_t systemId;
  uint16_t signalId;
  uint16_t cs;
};

struct Gga {
  float time;
  float lat;
  char ns;
  float lon;
  char ew;
  uint8_t quality;
  float numSV;
  float HDOP;
  float alt;
  char altUnit;
  float sep;
  char sepUnit;
  float doffAge;
  float diffStation;
  uint16_t cs;
};

struct Gll {
    double lat;
    char ns;
    double lon;
    char ew;
    float time;
    char status;
    char posMode;
};

struct Gsa {


};

struct Gst {


};

struct Gsv {


};

struct Rmc {


};

struct Vtg {


};

struct Zda {


};

#pragma pack(pop)  // Back to whatever the previous packing mode was.

struct ImuParameter {
  double gyro_scale;
  double accel_scale;
  double sampling_rate_hz;
};

using ::apollo::drivers::gnss::config::ImuType;
inline ImuParameter GetImuParameter(ImuType type) {
  switch (type) {
    case ImuType::IMAR_FSAS:
      // 0.1 * (2 ** -8) * (math.pi / 180 / 3600), (0.05 * (2 ** -15)
      return {1.893803441835e-9, 1.52587890625e-6, 200.0};

    case ImuType::ADIS16488:
      // 720/2**31 deg/LSB, 200/2**31 m/s/LSB
      return {5.8516723170686385e-09, 9.31322574615478515625e-8, 200.0};

    case ImuType::STIM300:
      // 2**-21 deg/LSB, 2**-22 m/s/LSB
      return {8.32237840649762e-09, 2.384185791015625e-07, 125.0};

    case ImuType::ISA100:
    case ImuType::ISA100C:
      // 1.0e-9 rad/LSB, 2.0e-8 m/s/LSB
      return {1.0e-9, 2.0e-8, 200.0};

    case ImuType::ISA100_400HZ:
    case ImuType::ISA100C_400HZ:
      return {1.0e-9, 2.0e-8, 400.0};

    case ImuType::G320N:
      return {1.7044230976507124e-11, 2.3929443359375006e-10, 125.0};

    case ImuType::CPT_XW5651:
      return {1.0850694444444445e-07, 1.52587890625e-06, 100.0};

    case ImuType::UM442:
      return {6.6581059144655048e-6, 2.99127170628e-5, 20.0};

    case ImuType::IAM20680:
      // (1.0/65.5)/125.0 deg/LSB (1.0/8192.0)*9.80665/125.0 m/s/LSB
      return {0.0001221374045, 9.57680664e-06, 125};

    default:
      return {0.0, 0.0, 0.0};
  }
}

}  // namespace ublox
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
