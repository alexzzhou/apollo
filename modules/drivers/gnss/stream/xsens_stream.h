#pragma once

#include <iostream>
#include <iomanip>
#include <list>
#include <string>
#include <mutex>

#include "xsens/include/xsensdeviceapi.h"
#include "xsens/include/xstypes.h"

#include "modules/drivers/gnss/stream/stream.h"

namespace apollo {
namespace drivers {
namespace gnss {

class CallbackHandler : public XsCallback {
 public:
  CallbackHandler(size_t maxBufferSize = 5)
      : m_maxNumberOfMessagesInBuffer(maxBufferSize),
        m_numberOfMessagesInBuffer(0) {}

  virtual ~CallbackHandler() throw() {}

  bool messageAvailable() const {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    return m_numberOfMessagesInBuffer > 0;
  }

  XsMessage getNextMessage() {
    assert(messageAvailable());
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    XsMessage oldestMessage(m_messageBuffer.front());
    m_messageBuffer.pop_front();
    --m_numberOfMessagesInBuffer;
    return oldestMessage;
  }

 protected:
  virtual void onMessageReceivedFromDevice(XsDevice* dev,
                                           XsMessage const* message) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    assert(message != nullptr);
    while (m_numberOfMessagesInBuffer >= m_maxNumberOfMessagesInBuffer)
      (void)getNextMessage();

    m_messageBuffer.push_back(*message);
    ++m_numberOfMessagesInBuffer;
    assert(m_numberOfMessagesInBuffer <= m_maxNumberOfMessagesInBuffer);
  }

 private:
  mutable std::recursive_mutex m_mutex;

  size_t m_maxNumberOfMessagesInBuffer;
  size_t m_numberOfMessagesInBuffer;
  std::list<XsMessage> m_messageBuffer;
};

class XsensStream : public Stream {

 public:
  XsensStream();
  ~XsensStream();

  virtual bool Connect();
  virtual bool Disconnect();
  virtual size_t read(uint8_t *buffer, size_t max_length);
  virtual size_t write(const uint8_t *data, size_t length);
  
  XsDataPacket packet_;

private:
    XsControl* control = nullptr;
    XsPortInfo mtPort;
    XsDevice* device = nullptr;
    CallbackHandler callback;
    XsOutputConfigurationArray configArray;
    
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
