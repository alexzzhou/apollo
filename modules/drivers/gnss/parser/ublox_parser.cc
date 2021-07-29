#include "modules/drivers/gnss/parser/ublox_parser.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "modules/drivers/gnss/parser/newtonm2_parser.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/rtcm_decode.h"
#include "modules/drivers/gnss/parser/ublox_messages.h"
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

Parser *Parser::CreateUblox(const config::Config &config) {
  return new UbloxParser();
}

// UbloxParser::UbloxParser() {

// }

Parser::MessageType UbloxParser::GetMessage(MessagePtr *message_ptr) {
  if (data_ == nullptr) {
    return MessageType::NONE;
  }

  while (data_ < data_end_) {
    if (buffer_.empty()) {  // Looking for SYNC0
      if (*data_ == 0xB5) {
        buffer_.push_back(*data_);
      }
      ++data_;
    } else if (buffer_.size() == 1) {  // Looking for SYNC1
      if (*data_ == 0x62) {
        buffer_.push_back(*data_++);
      } else {
        buffer_.clear();
      }
    } else if (buffer_.size() >= 2 && buffer_.size() < 5) {  // Working on header.
      buffer_.push_back(*data_++);
    } else if (buffer_.size() == 5){
      //total includes 2 syncbytes, class and message ID (2 bytes)
      //payload size message (2 bytes), and 2 bytes checksum
      buffer_.push_back(*data_++);
      total_length_ = 8 +
                    reinterpret_cast<ublox::Header*>(buffer_.data())
                        ->message_length;
    } else if (total_length_ > 0) {
      if (buffer_.size() < total_length_) {  // Working on body.
        buffer_.push_back(*data_++);
        continue;
      }
      MessageType type = PrepareMessage(message_ptr);
      buffer_.clear();
      total_length_ = 0;
      if (type != MessageType::NONE) {
        return type;
      }
    }
  }

  return MessageType::NONE;
}

bool UbloxParser::verify_checksum() { return true; }

Parser::MessageType UbloxParser::PrepareMessage(MessagePtr *message_ptr) {

  uint8_t* message = nullptr;
  ublox::ClassId class_id;
  ublox::MessageId message_id;
  uint16_t message_length;

  //getting header information
  auto header = reinterpret_cast<const ublox::Header*>(buffer_.data());
  message = buffer_.data();
  class_id = header->class_id;
  message_id = header->message_id;
  AINFO << "Class ID: " << class_id << " Message ID: " << message_id;
  message_length = header->message_length;

  switch (message_id) {
    case ublox::NAV_POSECEF:
      AINFO << "ECEF position returned.";
      break;
    case ublox::NAV_POSLLH:
      if (message_length != sizeof(ublox::NAVPOSLLH)) {
        AERROR << "Incorrect message_length";
        break;
      }
      // auto payload = reinterpret_cast<const ublox::NAVPOSLLH*>(message);
      // bestpos_.set_measurement_time(payload->itow);
      // bestpos_.set_latitude(payload->lat);
      // bestpos_.set_longitude(payload->lon);
      // *message_ptr = &bestpos_;
      return MessageType::BEST_GNSS_POS;
      break;
    case ublox::NAV_PVT:
      break;
    // case ublox::NAV_PVT:
    //   AINFO << "Position Velocity Time Solution";
    //   if (message_length != sizeof(ublox::NAVPVT)) {
    //     AERROR << "Incorrect message_length";
    //     break;
    //   }
    //   auto payload_pvt = reinterpret_cast<const ublox::NAVPVT*>(message);
    //   bestpos_.set_measurement_time(payload_pvt->itow);
    //   bestpos_.set_latitude(payload_pvt->lat);
    //   bestpos_.set_longitude(payload_pvt->lon);
    //   *message_ptr = &bestpos_;
    //   return MessageType::BEST_GNSS_POS;
    //   break;
    // default:
    //   break;
  }
  return MessageType::NONE;
}

// bool UbloxParser::HandleGSV(std::vector<std::string> contents){

//   size_t total = contents.size();
//   size_t repeats = (total-7)/4;

//   return true;
// }
// The handle_xxx functions return whether a message is ready.

// bool UbloxParser::handle_esf_raw(const ublox::EsfRaw* raw, size_t
// data_size) { return true; } bool UbloxParser::handle_esf_ins() { return
// true; } bool UbloxParser::handle_hnr_pvt(const ublox::HnrPvt* pvt) { return
// true; } bool UbloxParser::handle_nav_att(const ublox::NavAtt* att) { return
// true; } bool UbloxParser::handle_nav_pvt(const ublox::NavPvt* pvt) { return
// true; } bool UbloxParser::handle_nav_cov(const ublox::NavCov* cov) { return
// true; } bool UbloxParser::handle_rxm_rawx(const ublox::RxmRawx* raw) {
// return true; }

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo