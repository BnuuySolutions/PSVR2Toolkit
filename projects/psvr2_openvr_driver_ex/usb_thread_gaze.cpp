#include "usb_thread_gaze.h"

#include "hmd_driver_loader.h"
#include "hmd_device_hooks.h"
#include "eyelid_estimator.h"
#include "hmd2_gaze.h"
#include "ipc_server.h"

#include <cstdlib>

#include <winusb.h>

#define GAZE_MAGIC_0 0x47
#define GAZE_MAGIC_1_CAL 0x43
#define GAZE_MAGIC_1_RAW 0x52
#define GAZE_MAGIC_1_STATE 0x53

using namespace psvr2_toolkit;
using namespace psvr2_toolkit::ipc;

void **ppVTable = nullptr; // We need to keep track of our customized CaesarUsbThread VTable here, so we may restore it.

void *(*Framework__Mutex__lock)(void *thisptr, uint32_t timeout) = nullptr;
void *(*Framework__Mutex__unlock)(void *thisptr) = nullptr;
void *(*Framework__Thread__stop)(void *thisptr) = nullptr;

void *(*CaesarUsbThread__CaesarUsbThread)(void *thisptr) = nullptr;
void *(*CaesarUsbThread__dtor_CaesarUsbThread)(void *thisptr, char a2) = nullptr;
int (*CaesarUsbThread__read)(void *thisptr, uint8_t pipeId, char *buffer, size_t length) = nullptr;

CaesarUsbThreadGaze *CaesarUsbThreadGaze::m_pInstance = nullptr;

psvr2_toolkit::EyelidEstimator leftEyelidEstimator;
psvr2_toolkit::EyelidEstimator rightEyelidEstimator;

void *j_CaesarUsbThreadGaze__dtor_CaesarUsbThreadGaze(CaesarUsbThreadGaze *thisptr, char a2) {
  thisptr->dtor_CaesarUsbThreadGaze();
  void *result = CaesarUsbThread__dtor_CaesarUsbThread(thisptr, a2);
  CaesarUsbThreadGaze::Reset();
  return result;
}

void j_CaesarUsbThreadGaze__close(CaesarUsbThreadGaze *thisptr) {
  return thisptr->close();
}

uint8_t j_CaesarUsbThreadGaze__getUsbInf(CaesarUsbThreadGaze *thisptr) {
  return thisptr->getUsbInf();
}

uint8_t j_CaesarUsbThreadGaze__getReadPipeId(CaesarUsbThreadGaze *thisptr) {
  return thisptr->getReadPipeId();
}

int j_CaesarUsbThreadGaze__poll(CaesarUsbThreadGaze *thisptr) {
  return thisptr->poll();
}

void CaesarUsbThreadGaze::Reset() {
  CaesarUsbThreadGaze::m_pInstance = nullptr;
}

CaesarUsbThreadGaze *CaesarUsbThreadGaze::Instance() {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

  if (!m_pInstance) {
    m_pInstance = static_cast<CaesarUsbThreadGaze *>(malloc(sizeof(CaesarUsbThreadGaze)));
    if (m_pInstance) {
      Framework__Mutex__lock = decltype(Framework__Mutex__lock)(pHmdDriverLoader->GetBaseAddress() + 0x16B5F0);
      Framework__Mutex__unlock = decltype(Framework__Mutex__unlock)(pHmdDriverLoader->GetBaseAddress() + 0x16B850);
      Framework__Thread__stop = decltype(Framework__Thread__stop)(pHmdDriverLoader->GetBaseAddress() + 0x16B540);

      CaesarUsbThread__CaesarUsbThread = decltype(CaesarUsbThread__CaesarUsbThread)(pHmdDriverLoader->GetBaseAddress() + 0x121F30);
      CaesarUsbThread__read = decltype(CaesarUsbThread__read)(pHmdDriverLoader->GetBaseAddress() + 0x127D60);

      // Initialize base class.
      CaesarUsbThread__CaesarUsbThread(m_pInstance);

      if (!ppVTable) {
        // Runtime VTable madness!
        // We must allocate the total size of the CaesarUsbThread VTable (9 virtual functions, multiplied by 8 to account for function pointer size).
        // We'll then copy the VTable initialized by calling CaesarUsbThread::CaesarUsbThread into our allocated VTable.
        // Pretty neat, right?
        ppVTable = static_cast<void **>(malloc(0x48));
        if (ppVTable) {
          memcpy(ppVTable, m_pInstance->m_ppVTable, 0x48);

          CaesarUsbThread__dtor_CaesarUsbThread = decltype(CaesarUsbThread__dtor_CaesarUsbThread)(ppVTable[0]); // Store the original destructor here.

          ppVTable[0] = &j_CaesarUsbThreadGaze__dtor_CaesarUsbThreadGaze;
          ppVTable[2] = &j_CaesarUsbThreadGaze__close;
          ppVTable[4] = &j_CaesarUsbThreadGaze__getUsbInf;
          ppVTable[5] = &j_CaesarUsbThreadGaze__getReadPipeId;
          ppVTable[8] = &j_CaesarUsbThreadGaze__poll;
        }
      }

      m_pInstance->m_ppVTable = ppVTable;
    }
  }

  return m_pInstance;
}

void CaesarUsbThreadGaze::dtor_CaesarUsbThreadGaze() {
  m_ppVTable = ppVTable;
  close();
}

void CaesarUsbThreadGaze::close() {
  Framework__Mutex__lock((void *)((__int64)(this) + 0x30), 0xFFFFFFFF);
  *(char *)((__int64)(this) + 0x1E0) = 1;
  if (*(int *)((__int64)(this) + 0x28) == 2) {
    WinUsb_AbortPipe(*(WINUSB_INTERFACE_HANDLE *)((__int64)(this) + 0x48), 0x85);
  }
  Framework__Mutex__unlock((void *)((__int64)(this) + 0x30));
  Framework__Thread__stop(this);
}

uint8_t CaesarUsbThreadGaze::getUsbInf() {
  return 5;
}

uint8_t CaesarUsbThreadGaze::getReadPipeId() {
  return 0x85;
}

int CaesarUsbThreadGaze::poll() {
  static IpcServer *pIpcServer = IpcServer::Instance();

  static char buffer[0x200000];
  int result = CaesarUsbThread__read(this, 0x85, buffer, sizeof(buffer));
  if (result < 0) {
    return -1;
  }

  if (buffer[0] == GAZE_MAGIC_0 && buffer[1] == GAZE_MAGIC_1_STATE) {
    Hmd2GazeState *pGazeState = reinterpret_cast<Hmd2GazeState *>(buffer);
    HmdDeviceHooks::UpdateGaze(pGazeState, sizeof(Hmd2GazeState));
    float leftEyelidOpenness = leftEyelidEstimator.Estimate(pGazeState->leftEye);
    float rightEyelidOpenness = rightEyelidEstimator.Estimate(pGazeState->rightEye);
    pIpcServer->UpdateGazeState(pGazeState, leftEyelidOpenness, rightEyelidOpenness);
  }

  return 0;
}
