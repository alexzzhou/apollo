#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "cyber/cyber.h"
#include "modules/drivers/gnss/parser/novatel_messages.h"
#include "modules/drivers/gnss/parser/ublox_messages.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/rtcm_decode.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/gnss/util/time_conversion.h"

#include "xsens/include/xsensdeviceapi.h"
#include "xsens/include/xstypes.h"

namespace apollo {
namespace drivers {
namespace gnss {

class XsensParser : public Parser {

 public:
  XsensParser() = default;

  virtual ~XsensParser() = default;

  virtual std::vector<std::pair<MessagePtr, Parser::MessageType>> GetMultiMessage(MessagePtr *message_ptr);

  virtual Parser::MessageType GetMessage(MessagePtr *message_ptr);

  // ::apollo::drivers::gnss::Gnss GetGnss();
  // ::apollo::drivers::gnss::GnssBestPose GetBestPos();
  // ::apollo::drivers::gnss::Imu GetImu();
  // ::apollo::drivers::gnss::Ins GetIns();

 private:
  inline void rfu_to_flu(double r, double f, double u,
                         ::apollo::common::Point3D* flu) {
    flu->set_x(f);
    flu->set_y(-r);
    flu->set_z(u);
  }
  bool verify_checksum();

  std::vector<std::pair<MessagePtr, Parser::MessageType>> PrepareMessage(MessagePtr *message_ptr, XsDataPacket packet);

  double gps_seconds_base_ = -1.0;

  double gyro_scale_ = 0.0;

  double accel_scale_ = 0.0;

  float imu_measurement_span_ = 0.0;

  int imu_frame_mapping_ = 5;

  double imu_measurement_time_previous_ = -1.0;

  std::vector<uint8_t> buffer_;

  size_t total_length_ = 0;

  bool HandleBestPos(XsDataPacket packet);
  bool HandleGNSS(XsDataPacket packet);
  bool HandleImu(XsDataPacket packet);
  bool HandleIns(XsDataPacket packet);

  novatel::InsStatus ins_status_ =
      static_cast<novatel::InsStatus>(novatel::InsStatus::NONE);
      
  //protobuf messages
  ::apollo::drivers::gnss::Gnss gnss_;
  ::apollo::drivers::gnss::GnssBestPose bestpos_;
  ::apollo::drivers::gnss::Imu imu_;
  ::apollo::drivers::gnss::Ins ins_;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo