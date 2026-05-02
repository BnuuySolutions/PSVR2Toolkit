#pragma once

#include "cross_ipc.h"

#define MAX_PCM_SLOTS 4
#define PCM_BUFFER_SIZE 32 // TODO: we should have a header for commnn types and defines

struct GazeStatus {
  unsigned char data[0x148];

  void set(const unsigned char* pGazeStatus);
  void get(unsigned char* pGazeStatus) const;
};

struct GazeImage {
  int counter;
  unsigned char images[0x200100 * 8];

  void pushToCircularBuffer(const unsigned char* pGazeImage);
  int getFromCircularBuffer(unsigned char** gazeImageBuffer);
};

struct BufferData {
  GazeStatus gazeStatus;
  GazeImage gazeImage;
  unsigned char pcmLeft[MAX_PCM_SLOTS][PCM_BUFFER_SIZE];
  unsigned char pcmRight[MAX_PCM_SLOTS][PCM_BUFFER_SIZE];
};

class CustomShareManager {
public:
  static void createSingleton();
  static CustomShareManager *getSingleton();

  void setGazeStatus(const unsigned char* pGazeStatus);
  void getGazeStatus(unsigned char* pGazeStatus) const;

  void setGazeImage(const unsigned char* pGazeImage);

  int getGazeImageBuffer(unsigned char** gazeImageBuffer);

  void signalPcmUpdate();
  void readPcm(int slot, unsigned char* pcmLeft, unsigned char* pcmRight);

  int claimPcmSlot();
  void releasePcmSlot(int slot);
  void writePcm(int slot, const unsigned char* pcmLeft, const unsigned char* pcmRight);
  void waitForPcmUpdate(int slot);

private:
  static CustomShareManager *m_pInstance;
  static bool m_initialized;

  IIpcEvent* m_gazeStatusEvent;
  IIpcMutex* m_gazeStatusMutex;

  IIpcEvent* m_gazeImageEvent;
  IIpcMutex* m_gazeImageMutex;

  IIpcMutex* m_pcmOwnerMutex[MAX_PCM_SLOTS];

  IIpcEvent* m_pcmEvent[MAX_PCM_SLOTS];

  IIpcSharedMemory* m_sharedMemory;
  BufferData* m_pBufferData;

  void initialize();
};
