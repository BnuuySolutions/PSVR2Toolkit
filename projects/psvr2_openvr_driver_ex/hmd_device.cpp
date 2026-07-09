#include "hmd_device.h"

namespace sie {
namespace psvr2 {

  HmdDevice::HmdDevice(ServerDriver *serverDriver)
  	: m_unObjectId(vr::k_unTrackedDeviceIndexInvalid)
  {
  	m_serverDriver = serverDriver;

  	m_resolutionWidth = 4000;
  	m_resolutionHeight = 2040;
  }

} // psvr2
} // sie
