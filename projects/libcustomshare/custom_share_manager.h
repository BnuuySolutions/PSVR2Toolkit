#pragma once

#include "cross_ipc.h" // The new IPC abstraction DLL header

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
};

class CustomShareManager {
public:
  static void createSingleton();
  static CustomShareManager *getSingleton();

  void setGazeStatus(const unsigned char* pGazeStatus);
  void getGazeStatus(unsigned char* pGazeStatus) const;

  void setGazeImage(const unsigned char* pGazeImage);

  int getGazeImageBuffer(unsigned char** gazeImageBuffer);

private:
  static CustomShareManager *m_pInstance;
  static bool m_initialized;

  IIpcEvent* m_gazeStatusEvent;
  IIpcMutex* m_gazeStatusMutex;

  IIpcEvent* m_gazeImageEvent;
  IIpcMutex* m_gazeImageMutex;

  IIpcSharedMemory* m_sharedMemory;
  BufferData* m_pBufferData;

  void initialize();
};
