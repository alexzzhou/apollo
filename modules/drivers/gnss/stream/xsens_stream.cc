#include "modules/drivers/gnss/stream/xsens_stream.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cinttypes>
#include <iostream>
#include <list>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace gnss {

XsensStream::XsensStream() { AINFO << "XsensStream constructor"; }

XsensStream::~XsensStream() {}

bool XsensStream::Connect() {
  AINFO << "Constructing XsControl Object.";
  control = XsControl::construct();

  // scanning for devices
  AINFO << "Scanning ports for devices.";
  XsPortInfoArray portInfoArray = XsScanner::scanPorts();

  // Finding a MTi device
  for (auto const& portInfo : portInfoArray) {
    if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig()) {
      mtPort = portInfo;
      break;
    }
  }

  if (mtPort.empty()) {
    AERROR << "No MTi device found.";
    status_ = Stream::Status::ERROR;
    return false;
  }

  AINFO << "Found a device with ID: "
        << mtPort.deviceId().toString().toStdString()
        << " @ port: " << mtPort.portName().toStdString()
        << ", baudrate: " << mtPort.baudrate() << std::endl;

  AINFO << "Opening port..." << std::endl;
  if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate())) {
    AERROR << "Could not open port. Aborting.";
    status_ = Stream::Status::ERROR;
    return false;
  }

  // Get the device object
  device = control->device(mtPort.deviceId());
  assert(device != nullptr);

  AINFO << "Device: " << device->productCode().toStdString()
        << ", with ID: " << device->deviceId().toString() << " opened."
        << std::endl;

  // Create and attach callback handler to device
  device->addCallbackHandler(&callback);

  // Put the device into configuration mode before configuring the device
  AINFO << "Putting device into configuration mode..." << std::endl;
  if (!device->gotoConfig()) {
    AERROR << "Could not put device into configuration mode. Aborting.";
    status_ = Stream::Status::ERROR;
    return false;
  }

  AINFO << "Configuring the device..." << std::endl;

  configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
  configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

  if (device->deviceId().isImu()) {
    configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
    configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
    // configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
  } else if (device->deviceId().isVru() || device->deviceId().isAhrs()) {
    // configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
  } else if (device->deviceId().isGnss()) {
    configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
    configArray.push_back(XsOutputConfiguration(XDI_LatLon, 100));
    configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 100));
    configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, 100));
  } else {
    AERROR << "Unknown device while configuring. Aborting.";
    status_ = Stream::Status::ERROR;
    return false;
  }

  AINFO << "Putting device into measurement mode..." << std::endl;
  if (!device->gotoMeasurement()) {
    AERROR "Could not put device into measurement mode. Aborting.";
    status_ = Stream::Status::ERROR;
    return false;
  }
  AINFO << "Stream status connected.";
  status_ = Stream::Status::CONNECTED;
  return true;
}

bool XsensStream::Disconnect() {
  AINFO << "Closing port..." << std::endl;
  control->closePort(mtPort.portName().toStdString());

  AINFO << "Freeing XsControl object..." << std::endl;
  control->destruct();
  return true;
}

size_t XsensStream::read(uint8_t* buffer, size_t max_length) {

  if (status_ != Stream::Status::CONNECTED) {
    AERROR << "Stream connection error. Aborting.";
    return 0;
  }

  // AINFO << "Check whether packet is available";
  int offset = 0;
  while (callback.messageAvailable()) {
    XsMessage message = callback.getNextMessage();
    AINFO << "Message ID: " << message.getMessageId();
    size_t msg_length = message.getTotalMessageSize();
    if (offset + msg_length > max_length) {
      break;
    }
    memcpy(buffer + offset, message.getMessageStart(), message.getTotalMessageSize());
    offset += message.getTotalMessageSize();
  }

  return offset;
}

size_t XsensStream::write(const uint8_t* buffer, size_t length) {
  AINFO << "Gets here too.";
  return 0;
}

Stream* Stream::create_xsens() { return new XsensStream(); }

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
