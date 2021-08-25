#include "modules/drivers/gnss/parser/xsens_parser.h"

#include <math.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "modules/common/time/time.h"
#include "modules/drivers/gnss/parser/newtonm2_parser.h"
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

Parser *Parser::CreateXsens(const config::Config &config) {
  return new XsensParser();
}

std::vector<std::pair<Parser::MessageType, MessagePtr>>
XsensParser::GetMultiMessage(MessagePtr *message_ptr) {
  std::vector<std::pair<Parser::MessageType, MessagePtr>> types;
  if (data_ == nullptr) {
    return types;
  }
  uint8_t msg_id;
  uint8_t msg_length;

  while (data_ < data_end_) {
    if (buffer_.empty()) {  // Looking for PREAMBLE
      if (*data_ == 0xFA) {
        buffer_.push_back(*data_);
      }
      ++data_;
    } else if (buffer_.size() == 1) {  // Looking for BID
      if (*data_ == 0xFF) {
        buffer_.push_back(*data_++);
      } else {
        buffer_.clear();
      }
    } else if (buffer_.size() == 2) {  // Looking for MID
      msg_id = *data_;
      if (msg_id != 54) {
        AERROR << "Non-data type message received.";
      }
      buffer_.push_back(*data_++);
    } else if (buffer_.size() == 3) {  // Looking for Message Len
      msg_length = *data_;
      total_length_ = 5 + msg_length;
      buffer_.push_back(*data_++);
    } else if (total_length_ > 0) {
      while (buffer_.size() < total_length_) {  // Working on body.
        buffer_.push_back(*data_++);
      }

      // forming XsDataPacket from raw data
      XsMessage message(buffer_.data(), total_length_);
      XsDataPacket packet(&message);

      std::vector<std::pair<Parser::MessageType, MessagePtr>> types =
          PrepareMessage(message_ptr, packet);
      buffer_.clear();
      total_length_ = 0;

      return types;
    }
  }
  // If no messages in buffer, return a vector with MessageType::NONE
  std::pair<Parser::MessageType, MessagePtr> pack =
      std::make_pair(MessageType::NONE, nullptr);
  types.push_back(pack);
  return types;
}
/*! \brief Parses raw data taken from DataParser and formats into protobuf
         message
    \param message_ptr pointer to protobuf message
    \returns MessageType object, which identifies the type of protobuf
         message pointed to by message_ptr
*/
Parser::MessageType XsensParser::GetMessage(MessagePtr *message_ptr) {
  if (data_ == nullptr) {
    return MessageType::NONE;
  }
  uint8_t msg_id;
  uint8_t msg_length;
  uint16_t ext_msg_length;
  while (data_ < data_end_) {
    if (buffer_.empty()) {  // Looking for PREAMBLE
      if (*data_ == 0xFA) {
        buffer_.push_back(*data_);
      }
      ++data_;
    } else if (buffer_.size() == 1) {  // Looking for BID
      if (*data_ == 0xFF) {
        buffer_.push_back(*data_++);
      } else {
        buffer_.clear();
      }
    } else if (buffer_.size() == 2) {  // Looking for MID
      msg_id = *data_;
      if (msg_id != 54) {
        AERROR << "Non-data type message received.";
      }
      buffer_.push_back(*data_++);
    } else if (buffer_.size() == 3) {  // Looking for Message Len
      msg_length = *data_;
      buffer_.push_back(*data_++);
    } else if (buffer_.size() == 4) {
      ext_msg_length = *data_;
      buffer_.push_back(*data_++);
      buffer_.push_back(*data_++);
      total_length_ = 8 + msg_length + ext_msg_length;
      buffer_.push_back(*data_++);
    } else if (total_length_ > 0) {
      if (buffer_.size() < total_length_) {  // Working on body.
        buffer_.push_back(*data_++);
        continue;
      }

      // forming XsDataPacket from raw data
      XsMessage message(buffer_.data(), total_length_);
      XsDataPacket packet(&message);

      // Parser::MessageType type = PrepareMessage(message_ptr, packet);
      Parser::MessageType type = Parser::MessageType::NONE;
      buffer_.clear();
      total_length_ = 0;

      if (type != Parser::MessageType::NONE) {
        return type;
      }
    }
  }
  return Parser::MessageType::NONE;
}

bool XsensParser::verify_checksum() { return true; }

/*! \brief Uses Handle* functions to set attributes of each protobuf
      message

    \param message_ptr pointer to protobuf message
    \param packet Xsens Data Packet that contains data formatted into
          Xsens standards
    \returns MessageType object, which identifies the type of protobuf
          message pointed to by message_ptr
*/
std::vector<std::pair<Parser::MessageType, MessagePtr>>
XsensParser::PrepareMessage(MessagePtr *message_ptr, XsDataPacket packet) {
  std::vector<std::pair<Parser::MessageType, MessagePtr>> types;

  if (HandleBestPos(packet)) {
    std::pair<Parser::MessageType, MessagePtr> pack =
        std::make_pair(MessageType::BEST_GNSS_POS, &bestpos_);
    types.push_back(pack);
  }

  if (HandleGNSS(packet)) {
    std::pair<Parser::MessageType, MessagePtr> pack =
        std::make_pair(MessageType::GNSS, &gnss_);
    types.push_back(pack);
  }

  if (HandleIns(packet)) {
    std::pair<Parser::MessageType, MessagePtr> pack =
        std::make_pair(MessageType::INS, &ins_);
    types.push_back(pack);
  }

  if (HandleImu(packet)) {
    std::pair<Parser::MessageType, MessagePtr> pack =
        std::make_pair(MessageType::IMU, &imu_);
    types.push_back(pack);
  }

  return types;
}

/*! \brief Formats data into protobuf bestpos_ message

    \param packet Xsens Data Packet that contains data formatted into
          Xsens standards

    \returns true
*/
bool XsensParser::HandleBestPos(XsDataPacket packet) {
  if (packet.containsLatitudeLongitude()) {
    XsVector latlon = packet.latitudeLongitude();
    bestpos_.set_latitude(latlon[0]);
    bestpos_.set_longitude(latlon[1]);
  }

  bestpos_.set_measurement_time(apollo::common::time::Clock::NowInSeconds());
  return true;
}

/*! \brief Formats data into protobuf gnss_ message

    \param packet Xsens Data Packet that contains data formatted into
          Xsens standards

    \returns true
*/
bool XsensParser::HandleGNSS(XsDataPacket packet) {
  if (packet.containsLatitudeLongitude()) {
    XsVector latlon = packet.latitudeLongitude();
    gnss_.mutable_position()->set_lon(latlon[1]);
    gnss_.mutable_position()->set_lat(latlon[0]);
  }
  if (packet.containsAltitude()) {
    gnss_.mutable_position()->set_height(packet.altitude());
  }
  gnss_.set_measurement_time(apollo::common::time::Clock::NowInSeconds());
  gnss_.set_type(apollo::drivers::gnss::Gnss::SINGLE);
  return true;
}

/*! \brief Formats data into protobuf imu_ message

    \param packet Xsens Data Packet that contains data formatted into
          Xsens standards

    \returns true
*/
bool XsensParser::HandleImu(XsDataPacket packet) {
  if (packet.containsCalibratedAcceleration()) {
    XsVector acc = packet.calibratedAcceleration();

    // setting linear acceleration
    imu_.mutable_linear_acceleration()->set_x(acc[0]);
    imu_.mutable_linear_acceleration()->set_y(acc[1]);
    imu_.mutable_linear_acceleration()->set_z(acc[2]);
  }
  if (packet.containsCalibratedGyroscopeData()) {
    XsVector gyr = packet.calibratedGyroscopeData();

    // setting angular velocity
    imu_.mutable_angular_velocity()->set_x(gyr[0]);
    imu_.mutable_angular_velocity()->set_y(gyr[1]);
    imu_.mutable_angular_velocity()->set_z(gyr[2]);
  }

  imu_.set_measurement_time(apollo::common::time::Clock::NowInSeconds());
  return true;
}

/*! \brief Formats data into protobuf ins_ message

    \param packet Xsens Data Packet that contains data formatted into
          Xsens standards

    \returns true
*/
bool XsensParser::HandleIns(XsDataPacket packet) {
  ins_status_ = novatel::InsStatus::SOLUTION_GOOD;

  if (packet.containsOrientation()) {
    XsQuaternion quaternion = packet.orientationQuaternion();

    XsEuler euler = packet.orientationEuler();
    ins_.mutable_euler_angles()->set_x(euler.roll());
    ins_.mutable_euler_angles()->set_y(euler.pitch());
    ins_.mutable_euler_angles()->set_z(euler.yaw());
  }

  if (packet.containsLatitudeLongitude()) {
    XsVector latlon = packet.latitudeLongitude();
    ins_.mutable_position()->set_lon(latlon[1]);
    ins_.mutable_position()->set_lat(latlon[0]);
  }
  if (packet.containsAltitude()) {
    ins_.mutable_position()->set_height(packet.altitude());
  }
  if (packet.containsVelocity()) {
    XsVector vel = packet.velocity(XDI_VelocityXYZ);
    ins_.mutable_linear_velocity()->set_x(vel[0]);
    ins_.mutable_linear_velocity()->set_y(vel[1]);
    ins_.mutable_linear_velocity()->set_z(vel[2]);
  }
  if (packet.containsCalibratedAcceleration()) {
    XsVector acc = packet.calibratedAcceleration();

    // setting linear acceleration
    ins_.mutable_linear_acceleration()->set_x(acc[0]);
    ins_.mutable_linear_acceleration()->set_y(acc[1]);
    ins_.mutable_linear_acceleration()->set_z(acc[2]);
  }
  if (packet.containsCalibratedGyroscopeData()) {
    XsVector gyr = packet.calibratedGyroscopeData();

    // setting angular velocity
    ins_.mutable_angular_velocity()->set_x(gyr[0]);
    ins_.mutable_angular_velocity()->set_y(gyr[1]);
    ins_.mutable_angular_velocity()->set_z(gyr[2]);
  }
  ins_.set_measurement_time(apollo::common::time::Clock::NowInSeconds());
  ins_.set_type(apollo::drivers::gnss::Ins::GOOD);

  return true;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo