#include "slam_relocalizer_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "util.h"

#include <cmath>
#include <cstdint>

namespace psvr2_toolkit {

  constexpr float LIN_SENSITIVITY = 3.0f;
  constexpr float VELOCITY_THRESHOLD = 0.04f;
  constexpr float MIN_MULTIPLIER = 0.5f;
  constexpr float MAX_MULTIPLIER = 1.05f;

#pragma pack(push, 1)
  struct SlamPosePacket {
    uint32_t timestamp;       // 0x00: Microseconds
    uint32_t status;          // 0x04: 1 = Valid, 0 = Lost/Idle
    uint32_t magic;           // 0x08: 0xCD700049
    float posX;               // 0x0C: Position X (Meters)
    float posY;               // 0x10: Position Y (Meters)
    float posZ;               // 0x14: Position Z (Meters)
    float rot[4];             // 0x18: Rotation Quaternion (W, X, Y, Z)
    float reserved[5];        // 0x28: Unknown/Padding (likely secondary pose)
    float covariance[36];     // 0x3C: 6x6 Covariance Matrix (Uncertainty)
  };
#pragma pack(pop)

  static uint32_t g_last_time = 0;
  static float g_last_posX = 0.0f;
  static float g_last_posY = 0.0f;
  static float g_last_posZ = 0.0f;

  void (*SetRelocPreOutput)(void* thisptr, void* data, uint16_t type) = nullptr;
  void SetRelocPreOutputHook(void* thisptr, void* data, uint16_t type) {
    if (type == 1 && data) {
      auto* packet = reinterpret_cast<SlamPosePacket*>(data);

      if (packet->status == 1) {
        float linVel = 0.0f;

        if (g_last_time != 0) {
          float dt = (float)(packet->timestamp - g_last_time) / 1000000.0f;

          if (dt > 0.0001f) {
            float dx = packet->posX - g_last_posX;
            float dy = packet->posY - g_last_posY;
            float dz = packet->posZ - g_last_posZ;
            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            linVel = dist / dt;
          }
        }

        linVel = std::max(0.0f, linVel - VELOCITY_THRESHOLD);

        g_last_time = packet->timestamp;
        g_last_posX = packet->posX;
        g_last_posY = packet->posY;
        g_last_posZ = packet->posZ;

        // As velocity increases, the multiplier decreases (trust drops).
        float multiplier = MAX_MULTIPLIER / (1.0f + (linVel * LIN_SENSITIVITY));

        multiplier = std::max(MIN_MULTIPLIER, multiplier);

        // We modify the diagonal elements to increase variance (reduce confidence)
        // when moving quickly.
        for (int i = 0; i < 6; i++) {
          int diagIndex = i * 7;
          packet->covariance[diagIndex] *= multiplier;
        }
      }
    }

    // Call the original function
    SetRelocPreOutput(thisptr, data, type);
  }

  void SlamRelocalizerHooks::InstallHooks() {
    static HmdDriverLoader* pHmdDriverLoader = HmdDriverLoader::Instance();
    uintptr_t baseAddr = pHmdDriverLoader->GetBaseAddress();

    Util::DriverLog("Installing SLAM Relocalizer hooks...");

    // Hook SetRelocPreOutput
    HookLib::InstallHook(
      reinterpret_cast<void*>(baseAddr + 0x1ADBA0),
      reinterpret_cast<void*>(SetRelocPreOutputHook),
      reinterpret_cast<void**>(&SetRelocPreOutput)
    );
  }

} // psvr2_toolkit