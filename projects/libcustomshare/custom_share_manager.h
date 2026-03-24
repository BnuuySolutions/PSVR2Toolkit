#pragma once

class CustomShareManager {
public:
  static void createSingleton();
  static CustomShareManager *getSingleton();

  void setGazeStatus(unsigned char* pGazeStatus);
  void getGazeStatus(unsigned char* pGazeStatus);

private:
  struct HandlePair {
    void* hEvent;
    void* hMutex;
  };
  struct BufferData {
    unsigned char gazeStatus[0x148];
  };

  static CustomShareManager *m_pInstance;
  static bool m_initialized;

  HandlePair m_handles[1];
  void* m_hFileMap;
  BufferData* m_pBufferData;

  void initialize();
};
