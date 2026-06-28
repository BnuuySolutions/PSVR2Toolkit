#pragma once

#include "caesar_usb_thread.h"

#include <cstdint>

namespace psvr2_toolkit {

class CaesarUsbThreadGaze : public CaesarUsbThread {
public:
  virtual uint8_t GetInterface() override;
  virtual uint8_t GetEndpoint() override;

  virtual void OnConnected() override;

  virtual int PollAndProcess() override;

  static void Reset();
  static CaesarUsbThreadGaze *Instance();
private:
  static CaesarUsbThreadGaze *m_pInstance;
};

} // namespace psvr2_toolkit