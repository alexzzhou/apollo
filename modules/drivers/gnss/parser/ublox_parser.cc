#include "modules/drivers/gnss/parser/ublox_parser.h"

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

Parser* Parser::CreateUblox(const config::Config& config) {
    return new UbloxParser();
}

// UbloxParser::UbloxParser() {
    
// }

Parser::MessageType UbloxParser::GetMessage(MessagePtr *message_ptr) {
   if (data_ == nullptr) {
     return MessageType::NONE;
   }

   while (data_ < data_end_) {
     if (buffer_.empty()) {
       if (*data_ == ) {
         buffer_.push_back(*data_);
       }
      ++data_;
     } 
     } else if (header_length_ > 0) {  // Working on header.
  //     if (buffer_.size() < header_length_) {
  //       buffer_.push_back(*data_++);
  //     } else {
  //       if (header_length_ == ) {
  //         total_length_ = header_length_ + novatel::CRC_LENGTH +
  //                         reinterpret_cast<novatel::LongHeader*>(buffer_.data())
  //                             ->message_length;
  //       } else if (header_length_ == sizeof(novatel::ShortHeader)) {
  //         total_length_ =
  //             header_length_ + novatel::CRC_LENGTH +
  //             reinterpret_cast<novatel::ShortHeader*>(buffer_.data())
  //                 ->message_length;
  //       } else {
  //         AERROR << "Incorrect header_length_. Should never reach here.";
  //         buffer_.clear();
  //       }
  //       header_length_ = 0;
  //     }
  //   } else if (total_length_ > 0) {
  //     if (buffer_.size() < total_length_) {  // Working on body.
  //       buffer_.push_back(*data_++);
  //       continue;
  //     }
  //     MessageType type = PrepareMessage(message_ptr);
  //     buffer_.clear();
  //     total_length_ = 0;
  //     if (type != MessageType::NONE) {
  //       return type;
  //     }
  //   }
  // }
  return MessageType::NONE;

}

bool UbloxParser::verify_checksum() { return true; }

Parser::MessageType UbloxParser::PrepareMessage(MessagePtr& message_ptr) {
  
  // if (!check_crc()) {
  //   AERROR << "CRC check failed.";
    return MessageType::NONE;
  // }

  // uint8_t* message = nullptr;
  // novatel::MessageId message_id;
  // uint16_t message_length;
  // uint16_t gps_week;
  // uint32_t gps_millisecs;

  // switch (message_id) {
  //   case ublox_
  // }
}

// The handle_xxx functions return whether a message is ready.

//bool UbloxParser::handle_esf_raw(const ublox::EsfRaw* raw, size_t data_size) { return true; }
//bool UbloxParser::handle_esf_ins() { return true; }
//bool UbloxParser::handle_hnr_pvt(const ublox::HnrPvt* pvt) { return true; }
//bool UbloxParser::handle_nav_att(const ublox::NavAtt* att) { return true; }
//bool UbloxParser::handle_nav_pvt(const ublox::NavPvt* pvt) { return true; }
//bool UbloxParser::handle_nav_cov(const ublox::NavCov* cov) { return true; }
//bool UbloxParser::handle_rxm_rawx(const ublox::RxmRawx* raw) { return true; }

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo