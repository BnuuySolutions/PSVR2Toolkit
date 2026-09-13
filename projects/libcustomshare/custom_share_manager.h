#pragma once

#include "cross_ipc.h"
#include "common.h"
#include "hmd2_gaze.h"

#include <mutex>

constexpr int k_maxSlots = 8;

// Layout/protocol generation for everything in this file.
//
// BufferData is mapped by name into several processes that are built and shipped together. If one of
// them is stale -- an old psvr2_toolkit_capi.dll left behind by a partial install, say -- the two sides
// disagree about field offsets while happily sharing the same memory, and the corruption is silent.
// Suffixing every IPC object name means a mismatched build simply sees nothing instead: the driver
// reads as inactive, which is a symptom someone can actually diagnose.
//
// Bump this whenever BufferData's layout or the command protocol changes.
#define CUSTOM_SHARE_IPC_GENERATION "2"
#define CUSTOM_SHARE_NAME(base) base "_G" CUSTOM_SHARE_IPC_GENERATION

struct GazeStatus {
  hmd2_gaze_status_t data;
  int counter;

  void set(const hmd2_gaze_status_t *pGazeStatus);
  void get(hmd2_gaze_status_t *pGazeStatus) const;
};

// Slot size: 0x100 header + 0x200000 payload.
constexpr uint32_t k_gazeImageSlotSize = 0x200100;
constexpr int k_gazeImageSlots = 8;

struct GazeImage {
  int counter;
  // Valid byte count per slot. The headset does not always fill the whole slot, and consumers need to
  // know how much of it is real.
  uint32_t sizes[k_gazeImageSlots];
  unsigned char images[k_gazeImageSlotSize * k_gazeImageSlots];

  void pushToCircularBuffer(const unsigned char *pGazeImage, uint32_t size);
  int getFromCircularBuffer(unsigned char **gazeImageBuffer);
  bool copyLatest(unsigned char *pDest, uint32_t destSize, uint32_t *pOutSize) const;
};

struct CommandBuffer {
  int head;
  int tail;
  DriverCommand commands[256];

  DriverCommand *push(const DriverCommand &command);
  DriverCommand *pop();
};

struct BufferData {
  GazeStatus gazeStatus;
  GazeImage gazeImage;
  unsigned char pcmLeft[k_maxSlots][k_senseChunkSize];
  unsigned char pcmRight[k_maxSlots][k_senseChunkSize];
  CommandBuffer commandBuffer;
};

class CustomShareManager {
public:
  static void createSingleton();
  static CustomShareManager *getSingleton();

#ifdef _WIN32
  void setupCAPIPath();
#endif

  bool getDriverActive();
  bool claimDriverMutex();
  void releaseDriverMutex();

  void setGazeStatus(const hmd2_gaze_status_t *pGazeStatus);
  bool getGazeStatus(hmd2_gaze_status_t *pGazeStatus, int *lastCounter = nullptr, uint32_t timeoutMs = 0);

  void setGazeImage(const unsigned char *pGazeImage, uint32_t size);

  // Deprecated: returns a pointer into shared memory after releasing the lock, so the producer can
  // recycle the slot while the caller is still reading it. Kept for existing clients.
  bool getGazeImageBuffer(unsigned char **gazeImageBuffer, int *lastCounter = nullptr, uint32_t timeoutMs = 0);

  // Copies the newest frame out under the lock. pOutSize receives the valid byte count and may be null.
  bool getGazeImageCopy(unsigned char *pDest, uint32_t destSize, uint32_t *pOutSize, int *lastCounter = nullptr, uint32_t timeoutMs = 0);

  void signalPcmUpdate();
  void readPcm(int slot, unsigned char *pcmLeft, unsigned char *pcmRight);

  int claimSlot();
  void releaseSlot(int slot);
  bool isSlotAlive(int slot);
  void writePcm(int slot, VRControllerType controllerType, const unsigned char *pcm);
  bool waitForPcmUpdate();

  bool submitCommand(DriverCommand &command);
  DriverCommand *popCommand(uint32_t timeoutMs);
  void fulfillCommand(DriverCommand *command);

private:
  static CustomShareManager *m_pInstance;
  static bool m_initialized;
  static std::mutex m_instanceMutex;

  IIpcBroadcast *m_gazeStatusBroadcast;
  IIpcMutex *m_gazeStatusMutex;

  IIpcBroadcast *m_gazeImageBroadcast;
  IIpcMutex *m_gazeImageMutex;

  IIpcMutex *m_slotOwnerMutex[k_maxSlots];

  IIpcBroadcast *m_pcmBroadcast;
  IIpcBroadcast *m_commandBroadcast;

  IIpcMutex *m_commandMutex;

  IIpcSharedMemory *m_sharedMemory;
  BufferData *m_pBufferData;

  IIpcMutex *m_driverActiveMutex;
  IIpcMutex *m_driverActiveGuardMutex;

  void initialize();
};
