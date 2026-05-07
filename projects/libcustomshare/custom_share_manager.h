#pragma once

#include "cross_ipc.h"
#include "common.h"
#include "hmd2_gaze.h"

#include <mutex>

constexpr int k_maxSlots = 4;

struct GazeStatus {
  hmd2_gaze_status_t data;
  int counter;

  void set(const hmd2_gaze_status_t* pGazeStatus);
  void get(hmd2_gaze_status_t* pGazeStatus) const;
};

struct GazeImage {
  int counter;
  unsigned char images[0x200100 * 8];

  void pushToCircularBuffer(const unsigned char* pGazeImage);
  int getFromCircularBuffer(unsigned char** gazeImageBuffer);
};

struct TriggerEffectCommand {
  int slot;
  TriggerEffectCommandPayload payload;
};

struct TriggerEffectBuffer {
  int head;
  int tail;
  TriggerEffectCommand commands[256];

  void push(int slot, const TriggerEffectCommandPayload& payload);
  bool pop(TriggerEffectCommand& outCommand);
};

struct CommandBuffer {
  int head;
  int tail;
  DriverCommand commands[256];

  DriverCommand* push(const DriverCommand& command);
  DriverCommand* pop();
};

struct BufferData {
  GazeStatus gazeStatus;
  GazeImage gazeImage;
  unsigned char pcmLeft[k_maxSlots][k_senseChunkSize];
  unsigned char pcmRight[k_maxSlots][k_senseChunkSize];
  TriggerEffectBuffer triggerEffectBuffer;
  CommandBuffer commandBuffer;
};

class CustomShareManager {
public:
  static void createSingleton();
  static CustomShareManager *getSingleton();

  void setGazeStatus(const hmd2_gaze_status_t* pGazeStatus);
  bool getGazeStatus(hmd2_gaze_status_t* pGazeStatus, int* lastCounter = nullptr, uint32_t timeoutMs = 0);

  void setGazeImage(const unsigned char* pGazeImage);

  bool getGazeImageBuffer(unsigned char** gazeImageBuffer, int* lastCounter = nullptr, uint32_t timeoutMs = 0);

  void signalPcmUpdate();
  void readPcm(int slot, unsigned char* pcmLeft, unsigned char* pcmRight);

  int claimSlot();
  void releaseSlot(int slot);
  bool isSlotAlive(int slot);
  void writePcm(int slot, VRControllerType controllerType, const unsigned char* pcm);
  void waitForPcmUpdate();

  void pushTriggerEffect(int slot, const TriggerEffectCommandPayload& payload);
  bool popTriggerEffect(TriggerEffectCommand& outCommand);

  void submitCommand(DriverCommand& command);
  DriverCommand* popCommand(uint32_t timeoutMs);
  void fulfillCommand(DriverCommand* command);

private:
  static CustomShareManager *m_pInstance;
  static bool m_initialized;
  static std::mutex m_instanceMutex;

  IIpcBroadcast* m_gazeStatusBroadcast;
  IIpcMutex* m_gazeStatusMutex;

  IIpcBroadcast* m_gazeImageBroadcast;
  IIpcMutex* m_gazeImageMutex;

  IIpcMutex* m_slotOwnerMutex[k_maxSlots];

  IIpcBroadcast* m_pcmBroadcast;
  IIpcBroadcast* m_commandBroadcast;

  IIpcMutex* m_triggerEffectMutex;

  IIpcMutex* m_commandMutex;

  IIpcSharedMemory* m_sharedMemory;
  BufferData* m_pBufferData;

  void initialize();
};
