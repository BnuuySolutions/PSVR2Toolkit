#pragma once

#include "cross_ipc.h"
#include "common.h"
#include "hmd2_gaze.h"

struct GazeStatus {
  hmd2_gaze_status_t data;

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

struct BufferData {
  GazeStatus gazeStatus;
  GazeImage gazeImage;
  unsigned char pcmLeft[MAX_SLOTS][k_unSenseChunkSize];
  unsigned char pcmRight[MAX_SLOTS][k_unSenseChunkSize];
  TriggerEffectBuffer triggerEffectBuffer;
};

class CustomShareManager {
public:
  static void createSingleton();
  static CustomShareManager *getSingleton();

  void setGazeStatus(const hmd2_gaze_status_t* pGazeStatus);
  void getGazeStatus(hmd2_gaze_status_t* pGazeStatus) const;

  void setGazeImage(const unsigned char* pGazeImage);

  int getGazeImageBuffer(unsigned char** gazeImageBuffer);

  void signalPcmUpdate();
  void readPcm(int slot, unsigned char* pcmLeft, unsigned char* pcmRight);

  int claimSlot();
  void releaseSlot(int slot);
  bool isSlotAlive(int slot);
  void writePcm(int slot, VRControllerType controllerType, const unsigned char* pcm);
  void waitForPcmUpdate(int slot);

  void pushTriggerEffect(int slot, const TriggerEffectCommandPayload& payload);
  bool popTriggerEffect(TriggerEffectCommand& outCommand);

private:
  static CustomShareManager *m_pInstance;
  static bool m_initialized;

  IIpcEvent* m_gazeStatusEvent;
  IIpcMutex* m_gazeStatusMutex;

  IIpcEvent* m_gazeImageEvent;
  IIpcMutex* m_gazeImageMutex;

  IIpcMutex* m_slotOwnerMutex[MAX_SLOTS];

  IIpcEvent* m_pcmEvent[MAX_SLOTS];

  IIpcMutex* m_triggerEffectMutex;

  IIpcSharedMemory* m_sharedMemory;
  BufferData* m_pBufferData;

  void initialize();
};
