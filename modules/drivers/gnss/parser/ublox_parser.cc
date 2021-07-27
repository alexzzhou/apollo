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

  std::string submessage(reinterpret_cast<const char *>(data_),
                         static_cast<size_t>(data_end_ - data_));

  size_t iter = 0;

  while (true) {
    size_t start = submessage.find_first_of("$");
    size_t end = submessage.find("\r\n");

    if (start == std::string::npos || end == std::string::npos) {
      AINFO << "break here";
      break;
    } else if (end > start) {
      AINFO << "curmsg: " << submessage.substr(start, end - start);
      data_ += (end - start);
      MessageType type =
          PrepareMessage(submessage.substr(start, end - start), message_ptr);
      return type;

    } else if (end < start) {
      iter = start;
      submessage = submessage.substr(iter);
    }
  }

  return MessageType::NONE;
}

bool UbloxParser::verify_checksum() { return true; }

Parser::MessageType UbloxParser::PrepareMessage(const std::string &message,
                                                MessagePtr *message_ptr) {
  std::string messageID = message.substr(3, 3);
  std::vector<std::string> contents;

  // formatting message into a vector
  std::string message_copy = message;
  size_t pos = 0;
  std::string token;
  while ((pos = message_copy.find(",")) != std::string::npos) {
    token = message_copy.substr(0, pos);
    contents.push_back(token);
    message_copy.erase(0, pos + 1);
  }

  AINFO << contents[3];

  // GNSS satellite fault detection
  if (messageID == "GBS") {
    bestpos_.set_sol_status(SolutionStatus::SOL_COMPUTED);
    bestpos_.set_sol_type(SolutionType::NONE);
    bestpos_.set_measurement_time(std::stod(contents[1]));
    if (contents[2] != "") {
      bestpos_.set_latitude_std_dev(std::stod(contents[2]));
    }
    if (contents[3] != "") {
      bestpos_.set_longitude_std_dev(std::stod(contents[3]));
    }
    if (contents[4] != "") {
      bestpos_.set_height_std_dev(std::stod(contents[4]));
    }
    *message_ptr = &bestpos_;
    return MessageType::BEST_GNSS_POS;
  }

  // Global positioning system fix data
  else if (messageID == "GGA") {
    bestpos_.set_sol_status(SolutionStatus::SOL_COMPUTED);
    bestpos_.set_sol_type(SolutionType::NONE);
    bestpos_.set_measurement_time(std::stod(contents[1]));
    if (contents[2] != "") {
      bestpos_.set_latitude(std::stod(contents[1]));
    }
    if (contents[4] != "") {
      bestpos_.set_longitude(std::stod(contents[3]));
    }
    if (contents[4] != "") {
      bestpos_.set_height_msl(std::stod(contents[9]));
    }
    *message_ptr = &bestpos_;
    return MessageType::BEST_GNSS_POS;
  }

  // Latitude and longitude, with time of position fix and status
  else if (messageID == "GLL") {
    if (contents[6] != "A") {
      bestpos_.set_sol_status(SolutionStatus::SOL_COMPUTED);
    } else {
      bestpos_.set_sol_status(SolutionStatus::INSUFFICIENT_OBS);
    }
    bestpos_.set_sol_type(SolutionType::NONE);
    if (contents[1] != "") {
      bestpos_.set_latitude(std::stod(contents[1]));
    }
    if (contents[3] != "") {
      bestpos_.set_longitude(std::stod(contents[3]));
    }
    bestpos_.set_measurement_time(std::stod(contents[5]));
    *message_ptr = &bestpos_;
    return MessageType::BEST_GNSS_POS;
  }

  // GNSS DOP and active satellites
  else if (messageID == "GSA") {
    double total_sat = 0;

    for (int i = 3; i < 15; i++) {
      if (contents[i] != "") {
        total_sat++;
      }
    }
    bestpos_.set_num_sats_tracked(total_sat);
  }

  // GNSS satellites in view
  else if (messageID == "GSV") {
    if (contents[3] != "") {
      bestpos_.set_num_sats_tracked(std::stod(contents[3]));
    }
  }

  // Recommended minimum data
  else if (messageID == "RMC") {
    gnss_.set_measurement_time(std::stod(contents[1]));

    if (contents[2] == "A") {
      gnss_.set_solution_status(SolutionStatus::SOL_COMPUTED);
    } else {
      bestpos_.set_sol_status(SolutionStatus::INSUFFICIENT_OBS);
    }

    if (contents[3] != "") {
      gnss_.mutable_position()->set_lon(std::stod(contents[3]));
    }
    if (contents[3] != "") {
      gnss_.mutable_position()->set_lat(std::stod(contents[5]));
    }

    double yaw = 400;
    gnss_.set_type(apollo::drivers::gnss::Gnss::INVALID);
    if (contents[8] != "") {
      gnss_.set_type(apollo::drivers::gnss::Gnss::SINGLE);
      yaw = newtonm2::azimuth_deg_to_yaw_rad(std::stod(contents[8]));
    }

    if (contents[7] != "" && yaw != 400) {
      gnss_.mutable_linear_velocity()->set_x(std::stod(contents[7]) * cos(yaw) *
                                             0.514444);
      gnss_.mutable_linear_velocity()->set_y(std::stod(contents[7]) * sin(yaw) *
                                             0.514444);
    }

    gnss_.set_position_type(0);

    *message_ptr = &gnss_;
    return MessageType::GNSS;
  }

  // Course over ground and ground speed
  else if (messageID == "VTG") {
    double yaw = 400;
    gnss_.set_type(apollo::drivers::gnss::Gnss::INVALID);
    if (contents[1] != "") {
      gnss_.set_type(apollo::drivers::gnss::Gnss::SINGLE);
      yaw = newtonm2::azimuth_deg_to_yaw_rad(std::stod(contents[1]));
    }
    if (contents[5] != "" && yaw != 400) {
      gnss_.mutable_linear_velocity()->set_x(std::stod(contents[5]) * cos(yaw) *
                                             0.514444);
      gnss_.mutable_linear_velocity()->set_y(std::stod(contents[5]) * sin(yaw) *
                                             0.514444);
    }

    gnss_.set_solution_status(SolutionStatus::SOL_COMPUTED);
    gnss_.set_position_type(0);
    *message_ptr = &gnss_;
    return MessageType::GNSS;
  }

  // Time and date
  else if (messageID == "ZDA") {
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