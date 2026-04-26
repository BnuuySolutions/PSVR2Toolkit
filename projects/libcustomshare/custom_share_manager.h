#pragma once

class CustomShareManager {
public:
  static void createSingleton();
  static CustomShareManager *getSingleton();

  void setGazeStatus(unsigned char* pGazeStatus);
  void getGazeStatus(unsigned char* pGazeStatus);

  void setGazeImage(unsigned char* pGazeImage);
  void getGazeImage(unsigned char* pGazeImage);

  //int getGazeImageBuffer(unsigned char** gazeImageBuffer, unsigned char** a2);

private:
  struct HandlePair {
    void* hEvent;
    void* hMutex;
  };
  struct BufferData {
    unsigned char gazeStatus[0x148];
    //unsigned char bufferInfo[0x878 * 8]; // TODO: Circular buffer
    unsigned char gazeImage[0x200100/* * 8*/];
  };

  static CustomShareManager *m_pInstance;
  static bool m_initialized;

  HandlePair m_handles[2];
  void* m_hFileMap;
  BufferData* m_pBufferData;
  //int gazeImageCounter;

  void initialize();
};
