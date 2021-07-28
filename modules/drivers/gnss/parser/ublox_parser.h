#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "cyber/cyber.h"
#include "modules/drivers/gnss/parser/novatel_messages.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/rtcm_decode.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/gnss/util/time_conversion.h"

namespace apollo {
namespace drivers {
namespace gnss {

class UbloxParser : public Parser {

 public:
  UbloxParser() = default;

  virtual ~UbloxParser() = default;

  virtual MessageType GetMessage(MessagePtr *message_ptr);

 private:
  bool verify_checksum();

  Parser::MessageType PrepareMessage(const std::string &message, MessagePtr *message_ptr);

  // bool HandleGSV(std::vector<std::string> contents);
  // The handle_xxx functions return whether a message is ready.
  //bool handle_esf_raw(const ublox::EsfRaw* raw, size_t data_size);
  //bool handle_esf_ins(const ublox::EsfIns* ins);
  //bool handle_hnr_pvt(const ublox::HnrPvt* pvt);
  //bool handle_nav_att(const ublox::NavAtt* att);
  //bool handle_nav_pvt(const ublox::NavPvt* pvt);
  //bool handle_nav_cov(const ublox::NavCov* cov);
  //bool handle_rxm_rawx(const ublox::RxmRawx* raw);

  double gps_seconds_base_ = -1.0;

  double gyro_scale_ = 0.0;

  double accel_scale_ = 0.0;

  float imu_measurement_span_ = 0.0;

  int imu_frame_mapping_ = 5;

  double imu_measurement_time_previous_ = -1.0;

  std::vector<uint8_t> buffer_;

  size_t total_length_ = 0;

  //protobuf messages
  ::apollo::drivers::gnss::Gnss gnss_;
  ::apollo::drivers::gnss::GnssBestPose bestpos_;
  ::apollo::drivers::gnss::Imu imu_;
  ::apollo::drivers::gnss::Ins ins_;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo