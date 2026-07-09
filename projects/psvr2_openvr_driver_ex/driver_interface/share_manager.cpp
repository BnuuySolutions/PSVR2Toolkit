#include "share_manager.h"
#include "../hmd_driver_loader.h"

#include "cross_ipc.h"
#include "hook_lib.h"
#include "util.h"

using namespace psvr2_toolkit;

#include <shlobj.h>
#include <thread>
#include <atomic>

static const char *SharedResourceNames[][2] = {{"SHARE_VRT2_WIN_COMMON_EVT", "SHARE_VRT2_WIN_COMMON_MTX"},
                                               {"SHARE_VRT2_WIN_STATUS_EVT", "SHARE_VRT2_WIN_STATUS_MTX"},
                                               {"SHARE_VRT2_WIN_CALIB_EVT", "SHARE_VRT2_WIN_CALIB_MTX"},
                                               {"SHARE_VRT2_WIN_INPUT_HMD_EVT", "SHARE_VRT2_WIN_INPUT_HMD_MTX"},
                                               {"SHARE_VRT2_WIN_INPUT_CONT_R_EVT", "SHARE_VRT2_WIN_INPUT_CONT_R_MTX"},
                                               {"SHARE_VRT2_WIN_INPUT_CONT_L_EVT", "SHARE_VRT2_WIN_INPUT_CONT_L_MTX"},
                                               {"SHARE_VRT2_WIN_POSE_HMD_EVT", "SHARE_VRT2_WIN_POSE_HMD_MTX"},
                                               {"SHARE_VRT2_WIN_POSE_CONT_R_EVT", "SHARE_VRT2_WIN_POSE_CONT_R_MTX"},
                                               {"SHARE_VRT2_WIN_POSE_CONT_L_EVT", "SHARE_VRT2_WIN_POSE_CONT_L_MTX"},
                                               {"SHARE_VRT2_WIN_IMAGE_EVT", "SHARE_VRT2_WIN_IMAGE_MTX"},
                                               {"SHARE_VRT2_WIN_EVF_EVT", "SHARE_VRT2_WIN_EVF_MTX"},
                                               {"SHARE_VRT2_WIN_PLAYAREA_RESULT_EVT", "SHARE_VRT2_WIN_PLAYAREA_RESULT_MTX"},
                                               {"SHARE_VRT2_WIN_IMAGE_SETTING_EVT", "SHARE_VRT2_WIN_IMAGE_SETTING_MTX"},
                                               {"SHARE_VRT2_WIN_BLOB_CONFIG_EVT", "SHARE_VRT2_WIN_BLOB_CONFIG_MTX"},
                                               {"SHARE_VRT2_WIN_IR_CAM_SETTING_EVT", "SHARE_VRT2_WIN_IR_CAM_SETTING_MTX"},
                                               {"SHARE_VRT2_WIN_CONT_CONFIG_EVT", "SHARE_VRT2_WIN_CONT_CONFIG_MTX"},
                                               {"SHARE_VRT2_WIN_ARM_MODEL_EVT", "SHARE_VRT2_WIN_ARM_MODEL_MTX"},
                                               {"SHARE_VRT2_WIN_CONT_LED_INFO_EVT", "SHARE_VRT2_WIN_CONT_LED_INFO_MTX"},
                                               {"SHARE_VRT2_WIN_BLUETOOTH_QUALITY_INFO_EVT", "SHARE_VRT2_WIN_BLUETOOTH_QUALITY_INFO_MTX"},
                                               {"SHARE_VRT2_WIN_FW_INFO_EVT", "SHARE_VRT2_WIN_FW_INFO_MTX"},
                                               {"SHARE_VRT2_WIN_FW_INFO_EVT", "SHARE_VRT2_WIN_FW_INFO_MTX"}, // Duplicated in binary
                                               {"SHARE_VRT2_WIN_VR_DIALOG_EVT", "SHARE_VRT2_WIN_VR_DIALOG_MTX"},
                                               {"SHARE_VRT2_WIN_APPLICATION_EVT", "SHARE_VRT2_WIN_APPLICATION_MTX"},
                                               {"SHARE_VRT2_WIN_VRTREACE_DATA_EVT", "SHARE_VRT2_WIN_VRTRACE_DATA_MTX"}, // Note spelling: VRTREACE
                                               {"SHARE_VRT2_WIN_DEBUG_DATA_EVT", "SHARE_VRT2_WIN_DEBUG_DATA_MTX"},
                                               {"SHARE_VRT2_WIN_LIBPAD_ACCESS_EVT", "SHARE_VRT2_WIN_LIBPAD_ACCESS_MTX"},
                                               {"SHARE_VRT2_WIN_LIBPAD_REQUEST_STEAM_VR_PLUGIN_EVT", "SHARE_VRT2_WIN_LIBPAD_REQUEST_STEAM_VR_PLUGIN_MTX"},
                                               {"SHARE_VRT2_WIN_LIBPAD_REQUEST_ASSITANT_APP_EVT", "SHARE_VRT2_WIN_LIBPAD_REQUEST_ASSITANT_APP_MTX"},
                                               {"SHARE_VRT2_WIN_GENERAL_CONFIG_EVT", "SHARE_VRT2_WIN_GENERAL_CONFIG_MTX"},
                                               {"SHARE_VRT2_WIN_LOG_EVT", "SHARE_VRT2_WIN_LOG_MTX"},
                                               {"SHARE_VRT2_WIN_TELEMETRY_DEV_INFO_EVT", "SHARE_VRT2_WIN_TELEMETRY_DEV_INFO_MTX"},
                                               {"SHARE_VRT2_WIN_TELEMETRY_TRACKING_INFO_EVT", "SHARE_VRT2_WIN_TELEMETRY_TRACKING_INFO_MTX"},
                                               {"SHARE_VRT2_WIN_TELEMETRY_TRACKING_PC_INFO_EVT", "SHARE_VRT2_WIN_TELEMETRY_TRACKING_PC_INFO_MTX"},
                                               {"SHARE_VRT2_WIN_VR_APP_SCENE_INFO_EVT", "SHARE_VRT2_WIN_VR_APP_SCENE_INFO_MTX"},
                                               {"SHARE_VRT2_WIN_INITIAL_SETUP_INFO_EVT", "SHARE_VRT2_WIN_INITIAL_SETUP_INFO_MTX"},
                                               {"SHARE_VRT2_WIN_PLAYAREA_SETUP_INFO_EVT", "SHARE_VRT2_WIN_PLAYAREA_SETUP_INFO_MTX"}};

class ProcessOwnedMutexManager {
public:
  enum Action { ACT_LOCK, ACT_TRY_LOCK, ACT_UNLOCK, ACT_TRY_LOCK_AND_UNLOCK, ACT_EXIT };

  struct Request {
    Action action;
    IIpcMutex *mutex;
    std::mutex *callerMtx;
    std::condition_variable *callerCv;
    bool *doneFlag;
    bool *resultBool;
  };

  static ProcessOwnedMutexManager &GetInstance() {
    static ProcessOwnedMutexManager instance;
    return instance;
  }

  void QueueRequest(Action action, IIpcMutex *mutex, std::mutex *callerMtx, std::condition_variable *callerCv, bool *doneFlag, bool *resultBool) {
    Request req = {action, mutex, callerMtx, callerCv, doneFlag, resultBool};
    {
      std::lock_guard<std::mutex> lock(m_queueMtx);
      m_requests.push_back(req);
    }
    SetEvent(m_hWakeEvent);
  }

  void Shutdown() {
    m_exiting = true;
    {
      std::lock_guard<std::mutex> lock(m_queueMtx);
      Request req = {ACT_EXIT, nullptr, nullptr, nullptr, nullptr, nullptr};
      m_requests.push_back(req);
    }
    SetEvent(m_hWakeEvent);
    if (m_thread.joinable()) {
      m_thread.join();
    }
    if (m_hWakeEvent) {
      CloseHandle(m_hWakeEvent);
      m_hWakeEvent = nullptr;
    }
  }

private:
  ProcessOwnedMutexManager() {
    m_hWakeEvent = CreateEventA(NULL, FALSE, FALSE, NULL);
    m_exiting = false;
    m_thread = std::thread(&ProcessOwnedMutexManager::ThreadLoop, this);
  }

  ~ProcessOwnedMutexManager() {
    m_exiting = true;
    if (m_thread.joinable()) {
      m_thread.detach();
    }
    if (m_hWakeEvent) {
      CloseHandle(m_hWakeEvent);
      m_hWakeEvent = nullptr;
    }
  }

  void ThreadLoop() {
    auto completeRequest = [](const Request &req, bool success) {
      if (req.callerMtx && req.doneFlag && req.resultBool && req.callerCv) {
        std::lock_guard<std::mutex> lock(*req.callerMtx);
        *req.doneFlag = true;
        *req.resultBool = success;
        req.callerCv->notify_all();
      }
    };

    std::vector<Request> pendingLocks;

    while (!m_exiting) {
      DWORD timeout = pendingLocks.empty() ? INFINITE : 5;
      WaitForSingleObject(m_hWakeEvent, timeout);

      if (m_exiting) {
        break;
      }

      std::vector<Request> localRequests;
      {
        std::lock_guard<std::mutex> lock(m_queueMtx);
        localRequests = std::move(m_requests);
        m_requests.clear();
      }

      for (const auto &req : localRequests) {
        if (req.action == ACT_EXIT) {
          m_exiting = true;
          break;
        }

        if (req.action == ACT_UNLOCK) {
          if (req.mutex) {
            IpcMutex_Unlock(req.mutex);
          }
          for (auto it = pendingLocks.begin(); it != pendingLocks.end();) {
            if (it->mutex == req.mutex) {
              it = pendingLocks.erase(it);
            } else {
              ++it;
            }
          }
          completeRequest(req, true);
        } else if (req.action == ACT_LOCK) {
          if (req.mutex) {
            pendingLocks.push_back(req);
          } else {
            completeRequest(req, false);
          }
        } else { // ACT_TRY_LOCK or ACT_TRY_LOCK_AND_UNLOCK
          bool success = false;
          if (req.mutex) {
            success = IpcMutex_TryLock(req.mutex);
            if (success && req.action == ACT_TRY_LOCK_AND_UNLOCK) {
              IpcMutex_Unlock(req.mutex);
            }
          }
          completeRequest(req, success);
        }
      }

      if (m_exiting) {
        break;
      }

      // Poll pending locks (which includes any newly added lock requests)
      for (auto it = pendingLocks.begin(); it != pendingLocks.end();) {
        if (IpcMutex_TryLock(it->mutex)) {
          completeRequest(*it, true);
          it = pendingLocks.erase(it);
        } else {
          ++it;
        }
      }
    }
  }

  std::mutex m_queueMtx;
  std::vector<Request> m_requests;
  HANDLE m_hWakeEvent;
  std::thread m_thread;
  std::atomic<bool> m_exiting;
};

static bool QueueAndWaitForRequest(ProcessOwnedMutex *target, std::unique_lock<std::mutex> &lock, ProcessOwnedMutexManager::Action action) {
  bool done = false;
  bool result = false;

  ProcessOwnedMutexManager::GetInstance().QueueRequest(action, target->ipcMutex, &target->localMtx, &target->cv, &done, &result);

  while (!done) {
    target->cv.wait(lock);
  }

  return result;
}

void ProcessOwnedMutex::Lock() {
  if (!ipcMutex)
    return;
  std::unique_lock<std::mutex> lock(localMtx);
  while (isLocking) {
    cv.wait(lock);
  }
  if (isAcquired) {
    return;
  }
  isLocking = true;

  bool success = QueueAndWaitForRequest(this, lock, ProcessOwnedMutexManager::ACT_LOCK);

  isLocking = false;
  if (success) {
    isAcquired = true;
  }
  cv.notify_all();
}

bool ProcessOwnedMutex::TryLock() {
  if (!ipcMutex)
    return false;
  std::unique_lock<std::mutex> lock(localMtx);
  if (isLocking) {
    return false;
  }
  if (isAcquired) {
    return true;
  }

  bool success = QueueAndWaitForRequest(this, lock, ProcessOwnedMutexManager::ACT_TRY_LOCK);

  if (success) {
    isAcquired = true;
  }
  cv.notify_all();
  return success;
}

void ProcessOwnedMutex::Unlock() {
  if (!ipcMutex)
    return;
  std::unique_lock<std::mutex> lock(localMtx);
  if (!isAcquired) {
    return;
  }

  QueueAndWaitForRequest(this, lock, ProcessOwnedMutexManager::ACT_UNLOCK);

  isAcquired = false;
  cv.notify_all();
}

bool ProcessOwnedMutex::IsFreeOrOwned() {
  if (!ipcMutex)
    return false;
  std::unique_lock<std::mutex> lock(localMtx);
  if (isAcquired) {
    return true;
  }
  if (isLocking) {
    return false;
  }

  return QueueAndWaitForRequest(this, lock, ProcessOwnedMutexManager::ACT_TRY_LOCK_AND_UNLOCK);
}

ShareManager *ShareManager::s_instance = nullptr;
bool ShareManager::s_isInitialized = false;

ShareManager::ShareManager() : m_hSharedFileMapping(nullptr), m_pMem(nullptr), m_instanceId(0) {
  memset(m_hConfigMutexes, 0, sizeof(m_hConfigMutexes));
  memset(m_hEvents, 0, sizeof(m_hEvents));
  memset(m_hMutexes, 0, sizeof(m_hMutexes));

  memset(m_ipcConfigMutexes, 0, sizeof(m_ipcConfigMutexes));
  memset(m_ipcEvents, 0, sizeof(m_ipcEvents));
  memset(m_ipcMutexes, 0, sizeof(m_ipcMutexes));
  m_ipcSharedMemory = nullptr;

  // Compile-time offset checks to guarantee binary compatibility with the DLL
  static_assert(offsetof(ShareManager, m_hConfigMutexes) == 0x08, "m_hConfigMutexes offset mismatch");
  static_assert(offsetof(ShareManager, m_hEvents) == 0x2190, "m_hEvents offset mismatch");
  static_assert(offsetof(ShareManager, m_hMutexes) == 0x22b0, "m_hMutexes offset mismatch");
  static_assert(offsetof(ShareManager, m_pEventContext) == 0x23d0, "m_pEventContext offset mismatch");
  static_assert(offsetof(ShareManager, m_hSharedFileMapping) == 0x23d8, "m_hSharedFileMapping offset mismatch");
  static_assert(offsetof(ShareManager, m_pMem) == 0x23e0, "m_pMem offset mismatch");
  static_assert(offsetof(ShareManager, m_instanceId) == 0x23e8, "m_instanceId offset mismatch");
  static_assert(offsetof(ShareManager, m_inputSequences) == 0x23ec, "m_inputSequences offset mismatch");
  static_assert(offsetof(ShareManager, m_poseSequences) == 0x23f8, "m_poseSequences offset mismatch");
  static_assert(offsetof(ShareManager, m_sequence) == 0x2404, "m_sequence offset mismatch");
  static_assert(offsetof(ShareManager, m_writeIndex_40) == 0x2408, "m_writeIndex_40 offset mismatch");
  static_assert(offsetof(ShareManager, m_writeIndex_28) == 0x2414, "m_writeIndex_28 offset mismatch");
  static_assert(offsetof(ShareManager, m_writeIndex) == 0x2420, "m_writeIndex offset mismatch");
}

ShareManager::~ShareManager() {
  m_exitThreads = true;

  if (m_pEventContext) {
    HANDLE hThread = nullptr;
    if (m_pEventContext->threadInfo) {
      DuplicateHandle(GetCurrentProcess(), *reinterpret_cast<HANDLE *>(m_pEventContext->threadInfo), GetCurrentProcess(), &hThread, 0, FALSE,
                      DUPLICATE_SAME_ACCESS);
    }

    m_pEventContext->exitFlag = 1;
    if (m_ipcEvents[SR_Evf]) {
      IpcEvent_Set(m_ipcEvents[SR_Evf]);
    }

    if (hThread) {
      WaitForSingleObject(hThread, INFINITE);
      CloseHandle(hThread);
    }
    m_pEventContext = nullptr;
  }

  if (m_hConfigMutexes[SC_Max]) {
    HANDLE hThread = *reinterpret_cast<HANDLE *>(m_hConfigMutexes[SC_Max]);
    if (hThread && hThread != INVALID_HANDLE_VALUE) {
      WaitForSingleObject(hThread, INFINITE);
    }
  }

  ProcessOwnedMutexManager::GetInstance().Shutdown();

  for (int i = 0; i < SR_Max; ++i) {
    if (m_ipcEvents[i]) {
      DestroyIpcEvent(m_ipcEvents[i]);
      m_ipcEvents[i] = nullptr;
    }
    if (m_hEvents[i]) {
      delete m_hEvents[i];
      m_hEvents[i] = nullptr;
    }
    if (m_ipcMutexes[i]) {
      DestroyIpcMutex(m_ipcMutexes[i]);
      m_ipcMutexes[i] = nullptr;
    }
    if (m_hMutexes[i]) {
      delete m_hMutexes[i];
      m_hMutexes[i] = nullptr;
    }
  }
  for (int i = 0; i < 16; ++i) {
    if (i < SC_Max) {
      if (m_ipcConfigMutexes[i]) {
        DestroyIpcMutex(m_ipcConfigMutexes[i]);
        m_ipcConfigMutexes[i] = nullptr;
      }
      if (m_hConfigMutexes[i]) {
        delete m_hConfigMutexes[i];
        m_hConfigMutexes[i] = nullptr;
      }
    } else if (i == SC_Max) {
      if (m_hConfigMutexes[SC_Max]) {
        CloseHandle(*reinterpret_cast<HANDLE *>(m_hConfigMutexes[SC_Max]));
        operator delete(m_hConfigMutexes[SC_Max]);
        m_hConfigMutexes[SC_Max] = nullptr;
      }
    }
  }
  if (m_ipcSharedMemory) {
    IpcSharedMemory_Unmap(m_ipcSharedMemory);
    DestroyIpcSharedMemory(m_ipcSharedMemory);
    m_ipcSharedMemory = nullptr;
  }
}

ShareManager *ShareManager::GetInstance() {
  if (s_instance == nullptr) {
    s_instance = new ShareManager();
  }
  return s_instance;
}

void ShareManager::InitializeInstance(DWORD processInstanceId) {
  s_isInitialized = true;

  if (s_instance == nullptr) {
    s_instance = new ShareManager();
  }

  s_instance->Initialize(processInstanceId);
}

void ShareManager::ShutdownInstance() {
  if (s_instance != nullptr) {
    delete s_instance;
    s_instance = nullptr;
  }
}

void ShareManager::InstallHooks() {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();
  uint64_t baseAddress = pHmdDriverLoader->GetBaseAddress();

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15bbd0), reinterpret_cast<void *>(&ShareManager::GetInstance));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15bcf0), reinterpret_cast<void *>(&ShareManager::InitializeInstance));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e360), reinterpret_cast<void *>(&ShareManager::ShutdownInstance));

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x158600), reinterpret_cast<void *>(&ShareManager::Initialize));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e0b0), reinterpret_cast<void *>(&ShareManager::RegisterEventCallback));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x157f70), reinterpret_cast<void *>(&ShareManager::WaitDynamicEvent));

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15d270), reinterpret_cast<void *>(&ShareManager::GetIntConfig));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15d3d0), reinterpret_cast<void *>(&ShareManager::ReadStringConfig_Hook));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15f470), reinterpret_cast<void *>(&ShareManager::WriteConfigString_Hook));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15d730), reinterpret_cast<void *>(&ShareManager::ReadLogStrings));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15f8c0), reinterpret_cast<void *>(&ShareManager::WriteLogString));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15dda0), reinterpret_cast<void *>(&ShareManager::UpdateProcessExitCodes));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15b950), reinterpret_cast<void *>(&ShareManager::ReadIntConfigSafe));

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15cf20), reinterpret_cast<void *>(&ShareManager::ReadCommon_0x10));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15ebe0), reinterpret_cast<void *>(&ShareManager::WriteCommon_0x10));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15dae0), reinterpret_cast<void *>(&ShareManager::ReadCommon_0x18));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15c7e0), reinterpret_cast<void *>(&ShareManager::ReadCommon_0x9dd0));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e520), reinterpret_cast<void *>(&ShareManager::WriteCommon_0x9dd0));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15c840), reinterpret_cast<void *>(&ShareManager::ReadCommon_0xc72c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e580), reinterpret_cast<void *>(&ShareManager::WriteCommon_0xc72c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15da80), reinterpret_cast<void *>(&ShareManager::ReadPlayareaSetup_0xc814));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15fba0), reinterpret_cast<void *>(&ShareManager::WritePlayareaSetup_0xc814));

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15c920), reinterpret_cast<void *>(&ShareManager::ReadArmModel_0x84a4));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e670), reinterpret_cast<void *>(&ShareManager::WriteArmModel_0x84a4));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15ca50), reinterpret_cast<void *>(&ShareManager::ReadBtQualityInfo_0x9170));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e7a0), reinterpret_cast<void *>(&ShareManager::WriteBtQualityInfo_0x9170));

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15a070), reinterpret_cast<void *>(&ShareManager::AcquireImageWriteSlot));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15a160), reinterpret_cast<void *>(&ShareManager::AcquireInputWriteSlot));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15a250), reinterpret_cast<void *>(&ShareManager::AcquirePoseWriteSlot));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15b6a0), reinterpret_cast<void *>(&ShareManager::CommitImageWriteSlot));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15b780), reinterpret_cast<void *>(&ShareManager::CommitInputWriteSlot));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15b830), reinterpret_cast<void *>(&ShareManager::CommitPoseWriteSlot));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x159970), reinterpret_cast<void *>(&ShareManager::AcquireImageReadSlot));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x159b60), reinterpret_cast<void *>(&ShareManager::AcquireInputReadSlot));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x159dc0), reinterpret_cast<void *>(&ShareManager::AcquirePoseReadSlot));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15b4d0), reinterpret_cast<void *>(&ShareManager::CommitImageReadSlot));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15b550), reinterpret_cast<void *>(&ShareManager::CommitInputReadSlot));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15b600), reinterpret_cast<void *>(&ShareManager::CommitPoseReadSlot));

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15c610), reinterpret_cast<void *>(&ShareManager::TryAcquireShareMutex));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e260), reinterpret_cast<void *>(&ShareManager::AcquireShareMutex));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e4f0), reinterpret_cast<void *>(&ShareManager::ReleaseShareMutex));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x159940), reinterpret_cast<void *>(&ShareManager::TryAcquireLibpadMutex));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e230), reinterpret_cast<void *>(&ShareManager::ReleaseLibpadMutex));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e480), reinterpret_cast<void *>(&ShareManager::ReleaseMutexByIndex));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e2a0), reinterpret_cast<void *>(&ShareManager::ClearLogEventFlag));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e2f0), reinterpret_cast<void *>(&ShareManager::SetGlobalEventFlag));

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15d8d0), reinterpret_cast<void *>(&ShareManager::ReadPlayareaResult));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15fa80), reinterpret_cast<void *>(&ShareManager::WritePlayareaResult));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15f9d0), reinterpret_cast<void *>(&ShareManager::WriteFwInfo2_0x9280));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e4a0), reinterpret_cast<void *>(&ShareManager::WaitShareEvent));

  // Group 1: Status
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15db30), reinterpret_cast<void *>(&ShareManager::ReadStatus_0x2c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15fc00), reinterpret_cast<void *>(&ShareManager::WriteStatus_0x2c));

  // Group 2: Calibration Blocks
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15cc60), reinterpret_cast<void *>(&ShareManager::ReadCalib_0x70));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e990), reinterpret_cast<void *>(&ShareManager::WriteCalib_0x70));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15cd30), reinterpret_cast<void *>(&ShareManager::ReadCalib_0x170));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15ea40), reinterpret_cast<void *>(&ShareManager::WriteCalib_0x170));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15cb00), reinterpret_cast<void *>(&ShareManager::ReadCalib_0x370));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e850), reinterpret_cast<void *>(&ShareManager::WriteCalib_0x370));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15cbc0), reinterpret_cast<void *>(&ShareManager::ReadCalib_0x3f0));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e910), reinterpret_cast<void *>(&ShareManager::WriteCalib_0x3f0));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15d460), reinterpret_cast<void *>(&ShareManager::ReadCalib_0x41c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15f610), reinterpret_cast<void *>(&ShareManager::WriteCalib_0x41c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15ce00), reinterpret_cast<void *>(&ShareManager::ReadCalib_0x50c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15eb00), reinterpret_cast<void *>(&ShareManager::WriteCalib_0x50c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15ce90), reinterpret_cast<void *>(&ShareManager::ReadCalib_0xd0c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15eb70), reinterpret_cast<void *>(&ShareManager::WriteCalib_0xd0c));

  // Groups 12-19: Configs & Info
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15d550), reinterpret_cast<void *>(&ShareManager::ReadImageSetting_0x803c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15f6f0), reinterpret_cast<void *>(&ShareManager::WriteImageSetting_0x803c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15c9d0), reinterpret_cast<void *>(&ShareManager::ReadBlobConfig_0x8058));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15e720), reinterpret_cast<void *>(&ShareManager::WriteBlobConfig_0x8058));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15d6b0), reinterpret_cast<void *>(&ShareManager::ReadIrCamSetting_0x807c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15f850), reinterpret_cast<void *>(&ShareManager::WriteIrCamSetting_0x807c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15cfd0), reinterpret_cast<void *>(&ShareManager::ReadContConfig_0x8098));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15f110), reinterpret_cast<void *>(&ShareManager::WriteContConfig_0x8098));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15d090), reinterpret_cast<void *>(&ShareManager::ReadContLedInfo_0x855c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15f1d0), reinterpret_cast<void *>(&ShareManager::WriteContLedInfo_0x855c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15d1e0), reinterpret_cast<void *>(&ShareManager::ReadFwInfo1_0x922c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15f330), reinterpret_cast<void *>(&ShareManager::WriteFwInfo1_0x922c));

  // Groups 20-24: Application & Debug
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15d860), reinterpret_cast<void *>(&ShareManager::ReadFwInfo2_0x9280));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15df60), reinterpret_cast<void *>(&ShareManager::ReadVrDialog_0x9ae0));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15ff00), reinterpret_cast<void *>(&ShareManager::WriteVrDialog_0x9ae0));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15c8a0), reinterpret_cast<void *>(&ShareManager::ReadApplication_0x9b3c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15dff0), reinterpret_cast<void *>(&ShareManager::ReadVrTraceData_0x9b7c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15ffa0), reinterpret_cast<void *>(&ShareManager::WriteVrTraceData_0x9b7c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15d100), reinterpret_cast<void *>(&ShareManager::ReadDebugData_0x9c8c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15f250), reinterpret_cast<void *>(&ShareManager::WriteDebugData_0x9c8c));

  // Groups 30-34: Telemetry & Setup
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15df00), reinterpret_cast<void *>(&ShareManager::ReadSceneInfo_0x9de4));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15fe90), reinterpret_cast<void *>(&ShareManager::WriteSceneInfo_0x9de4));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15dba0), reinterpret_cast<void *>(&ShareManager::ReadTelDevInfo_0xa984));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15fc70), reinterpret_cast<void *>(&ShareManager::WriteTelDevInfo_0xa984));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15dc60), reinterpret_cast<void *>(&ShareManager::ReadTelTrkInfo_0xaaa0));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15fd40), reinterpret_cast<void *>(&ShareManager::WriteTelTrkInfo_0xaaa0));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15dcd0), reinterpret_cast<void *>(&ShareManager::ReadTelPcInfo_0xc600));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15fdc0), reinterpret_cast<void *>(&ShareManager::WriteTelPcInfo_0xc600));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15d5d0), reinterpret_cast<void *>(&ShareManager::ReadInitSetup_0xc73c));
  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x15f760), reinterpret_cast<void *>(&ShareManager::WriteInitSetup_0xc73c));
}

void ShareManager::Initialize(this ShareManager &self, DWORD processInstanceId) {
  OutputDebugStringA("ShareManager::Initialize");

  self.m_instanceId = processInstanceId;

  for (int i = 0; i < SR_Max; ++i) {
    self.m_ipcEvents[i] = CreateIpcEvent(SharedResourceNames[i][0], true);
    self.m_ipcMutexes[i] = CreateIpcMutex(SharedResourceNames[i][1]);

    void *rawEvt = IpcEvent_GetNativeHandle(self.m_ipcEvents[i]);
    self.m_hEvents[i] = rawEvt ? new HANDLE(reinterpret_cast<HANDLE>(rawEvt)) : nullptr;

    void *rawMtx = IpcMutex_GetNativeHandle(self.m_ipcMutexes[i]);
    self.m_hMutexes[i] = rawMtx ? new HANDLE(reinterpret_cast<HANDLE>(rawMtx)) : nullptr;
  }

  self.m_libpadMutex.ipcMutex = self.m_ipcMutexes[SR_LibpadAccess];
  self.m_shareMutexes[0].ipcMutex = self.m_ipcMutexes[SR_LibpadRequestSteamVRPlugin];
  self.m_shareMutexes[1].ipcMutex = self.m_ipcMutexes[SR_LibpadRequestAssistantApp];

  self.m_ipcSharedMemory = CreateIpcSharedMemory("SHARE_VRT2_WIN", 0x2000000);
  if (self.m_ipcSharedMemory) {
    self.m_pMem = reinterpret_cast<VRSharedMemory *>(IpcSharedMemory_Map(self.m_ipcSharedMemory));
    void *rawShm = IpcSharedMemory_GetNativeHandle(self.m_ipcSharedMemory);
    self.m_hSharedFileMapping = rawShm ? reinterpret_cast<HANDLE>(rawShm) : nullptr;
  } else {
    self.m_pMem = nullptr;
    self.m_hSharedFileMapping = nullptr;
  }

  if (!self.m_pMem) {
    std::abort();
  }

  GlobalEventContext *pCtx = new GlobalEventContext();
  pCtx->hMutex = self.m_hMutexes[SR_Evf];
  pCtx->hEvent = self.m_hEvents[SR_Evf];
  pCtx->pSharedFlags = reinterpret_cast<uint64_t *>(reinterpret_cast<uint8_t *>(self.m_pMem) + 0x110C200);
  pCtx->exitFlag = '\0';
  pCtx->ipcMutex = self.m_ipcMutexes[SR_Evf];
  pCtx->ipcEvent = self.m_ipcEvents[SR_Evf];

  void *threadInfo1 = operator new(0x10);
  if (threadInfo1) {
    void *threadContext1 = operator new(0x8);
    if (threadContext1) {
      *reinterpret_cast<GlobalEventContext **>(threadContext1) = pCtx;
    }
    unsigned int threadId;
    uintptr_t hThread = _beginthreadex(nullptr, 0, &ShareManager::WorkerThread_Unconditional, threadContext1, 0, &threadId);
    *reinterpret_cast<uintptr_t *>(threadInfo1) = hThread;
    *reinterpret_cast<unsigned int *>(reinterpret_cast<uint8_t *>(threadInfo1) + 8) = threadId;
    pCtx->threadInfo = threadInfo1;
  } else {
    pCtx->threadInfo = nullptr;
  }

  self.m_pEventContext = pCtx;

  uint8_t *memBase = reinterpret_cast<uint8_t *>(self.m_pMem);

  for (int group = 0; group < 3; group++) {
    for (int slot = 0; slot < 64; slot++) {
      InputSlotMeta &slotMeta = self.m_pMem->input_meta.groups[group].slots[slot];
      if (slotMeta.state == 2 || (self.m_instanceId == 1 && slotMeta.state == 3)) {
        slotMeta.state = 0;
        slotMeta.read_counter = 0;
      }
    }
  }

  for (int group = 0; group < 3; group++) {
    for (int slot = 0; slot < 64; slot++) {
      PoseSlotMeta &slotMeta = self.m_pMem->pose_meta.groups[group].slots[slot];
      if (slotMeta.state == 2 || (self.m_instanceId == 1 && slotMeta.state == 3)) {
        slotMeta.state = 0;
        slotMeta.read_counter = 0;
      }
    }
  }

  for (int i = 0; i < 8; i++) {
    ImageSlotMeta &slot = self.m_pMem->image_meta.slots[i];
    if (slot.state == 2 || (self.m_instanceId == 1 && slot.state == 3)) {
      slot.state = 0;
      slot.sequenceId = 0;
    }
  }

  for (int i = 0; i < 8; i++) {
    PlayareaSlotState &slot = self.m_pMem->playarea_meta.playarea_states[i];
    if (slot.state == 2 || (self.m_instanceId == 1 && slot.state == 3)) {
      slot.state = 0;
      slot.sequenceId = 0;
    }
  }

  for (int i = 0; i < SC_Max; i++) {
    char configName[32];
    snprintf(configName, sizeof(configName), "CONFIG_ID_%d", i);
    self.m_ipcConfigMutexes[i] = CreateIpcMutex(configName);

    void *rawMtx = IpcMutex_GetNativeHandle(self.m_ipcConfigMutexes[i]);
    self.m_hConfigMutexes[i] = rawMtx ? new HANDLE(reinterpret_cast<HANDLE>(rawMtx)) : nullptr;
  }

  self.InitializeSub_ad30();

  if (self.m_instanceId == 1 || self.m_instanceId == 4) {
    self.InitializeSub_bdb0();

    void *threadInfo2 = operator new(0x10);
    if (threadInfo2) {
      void *threadContext2 = operator new(0x8);
      if (threadContext2) {
        *reinterpret_cast<ShareManager **>(threadContext2) = &self;
      }
      unsigned int threadId;
      uintptr_t hThread = _beginthreadex(nullptr, 0, &ShareManager::EvfWorkerThread_Conditional, threadContext2, 0, &threadId);
      if (hThread == 0) {
        std::abort();
      }
      *reinterpret_cast<uintptr_t *>(threadInfo2) = hThread;
      *reinterpret_cast<unsigned int *>(reinterpret_cast<uint8_t *>(threadInfo2) + 8) = threadId;
      self.m_hConfigMutexes[11] = reinterpret_cast<HANDLE *>(threadInfo2);
    }
  }

  *reinterpret_cast<uint16_t *>(memBase + 0x18) = 0x29;

  LARGE_INTEGER qpcFreq, qpcCount;
  QueryPerformanceFrequency(&qpcFreq);
  QueryPerformanceCounter(&qpcCount);

  double timestamp = (static_cast<double>(qpcCount.QuadPart) / static_cast<double>(qpcFreq.QuadPart)) * 1000000.0;

  self.m_pMem->watchdog.timestamps[self.m_instanceId] = timestamp;
  self.m_pMem->watchdog.pids[self.m_instanceId] = GetCurrentProcessId();
  self.m_pMem->watchdog.states[self.m_instanceId] = 0x29;
}

void ShareManager::RegisterEventCallback(this ShareManager &self, uint64_t mask, std::function<void()> *pCallback) {
  GlobalEventContext *pCtx = self.m_pEventContext;

  if (pCtx && pCallback) {
    if (pCtx->ipcMutex)
      IpcMutex_Lock(pCtx->ipcMutex);
    pCtx->callbacks.push_back({*pCallback, mask});
    if (pCtx->ipcMutex)
      IpcMutex_Unlock(pCtx->ipcMutex);
  }
}

void ShareManager::WaitDynamicEvent(GlobalEventContext **ppCtx) {
  if (!ppCtx)
    return;
  GlobalEventContext *pCtx = *ppCtx;
  if (!pCtx || pCtx->exitFlag != '\0')
    return;

  do {
    if (pCtx->ipcMutex)
      IpcMutex_Lock(pCtx->ipcMutex);
    uint64_t currentFlags = *pCtx->pSharedFlags;

    if (currentFlags == 0) {
      if (pCtx->ipcMutex)
        IpcMutex_Unlock(pCtx->ipcMutex);
      bool signaled = false;
      if (pCtx->ipcEvent) {
        signaled = IpcEvent_Wait(pCtx->ipcEvent, 100);
        IpcEvent_Reset(pCtx->ipcEvent);
      } else {
        Sleep(100);
      }
      if (!signaled)
        continue;
    } else {
      *pCtx->pSharedFlags = 0;
      if (pCtx->ipcMutex)
        IpcMutex_Unlock(pCtx->ipcMutex);

      for (const auto &cb : pCtx->callbacks) {
        if ((currentFlags & cb.second) == cb.second) {
          if (cb.first) {
            cb.first();
          }
        }
      }
    }
  } while (pCtx->exitFlag == '\0');
}

void ShareManager::GetIntConfig(this ShareManager &self, int configId, long *outValue) {
  if (configId < 0 || configId >= SC_Max || !outValue)
    return;

  if (self.m_ipcConfigMutexes[configId]) {
    IpcMutex_Lock(self.m_ipcConfigMutexes[configId]);
  }

  const char *strData = self.m_pMem->configs_9e00.str_configs[configId].stringData;

  if (strData[0] != '\0') {
    *outValue = std::strtol(strData, nullptr, 10);
  }

  if (self.m_ipcConfigMutexes[configId]) {
    IpcMutex_Unlock(self.m_ipcConfigMutexes[configId]);
  }
}

static const char *GetHostStringPtr(const void *hostStrObj) {
  if (!hostStrObj)
    return "";
  const size_t *ptr = reinterpret_cast<const size_t *>(hostStrObj);
  size_t capacity = ptr[3];
  if (capacity > 15) {
    return *reinterpret_cast<const char *const *>(hostStrObj);
  } else {
    return reinterpret_cast<const char *>(hostStrObj);
  }
}

uint32_t ShareManager::ReadStringConfig(this ShareManager &self, int configId, std::string &outStr) {
  if (configId < 0 || configId >= SC_Max)
    return 0xFFFFFFFF;

  if (self.m_ipcConfigMutexes[configId]) {
    IpcMutex_Lock(self.m_ipcConfigMutexes[configId]);
  }

  uint32_t counter = self.m_pMem->configs_9e00.str_configs[configId].counter;
  const char *strData = self.m_pMem->configs_9e00.str_configs[configId].stringData;

  outStr = strData;

  if (self.m_ipcConfigMutexes[configId]) {
    IpcMutex_Unlock(self.m_ipcConfigMutexes[configId]);
  }
  return counter;
}

void ShareManager::WriteConfigString(this ShareManager &self, int configId, const std::string &str) {
  if (configId < 0 || configId >= SC_Max)
    return;

  if (self.m_ipcConfigMutexes[configId]) {
    IpcMutex_Lock(self.m_ipcConfigMutexes[configId]);
  }

  uint32_t &counter = self.m_pMem->configs_9e00.str_configs[configId].counter;
  char *strData = self.m_pMem->configs_9e00.str_configs[configId].stringData;

  strncpy_s(strData, 264, str.c_str(), _TRUNCATE);

  counter += 1;

  if (self.m_ipcConfigMutexes[configId]) {
    IpcMutex_Unlock(self.m_ipcConfigMutexes[configId]);
  }
}

// We need these _Hook versions because std::string is not the same when
// building in Release and Debug. If we don't, it crashes/writes garbage when
// PSVR2TK is built as Debug. Also if our std::string changes because of the
// compiler or whatnot, we'd crash regardless.

uint32_t ShareManager::ReadStringConfig_Hook(this ShareManager &self, int configId, void *outStrObj) {
  std::string tempStr;
  uint32_t counter = self.ReadStringConfig(configId, tempStr);

  if (outStrObj) {
    uint64_t baseAddress = HmdDriverLoader::Instance()->GetBaseAddress();
    using AssignStringFn = void *(*)(void *target, const char *source, size_t length);
    auto AssignString = reinterpret_cast<AssignStringFn>(baseAddress + 0x4e8f0);

    AssignString(outStrObj, tempStr.c_str(), tempStr.length());
  }

  return counter;
}

void ShareManager::WriteConfigString_Hook(this ShareManager &self, int configId, const void *strObj) {
  if (configId < 0 || configId >= SC_Max)
    return;

  const char *str = GetHostStringPtr(strObj);
  self.WriteConfigString(configId, str);
}

int ShareManager::ReadLogStrings(this ShareManager &self, long long destAddress, int maxCount) {
  int readCount = 0;
  self.Lock(SR_Log);
  uint8_t head = self.m_pMem->logs.log_head;
  uint8_t tail = self.m_pMem->logs.log_tail;

  while (head != tail && readCount < maxCount) {
    uint8_t *destEntry = reinterpret_cast<uint8_t *>(destAddress + (readCount * 0x104));
    LogMetadata &meta = self.m_pMem->logs.log_meta[tail];

    destEntry[0] = meta.instanceId;
    destEntry[1] = meta.level;
    destEntry[2] = meta.length;
    destEntry[3] = meta.pad;

    strncpy_s(reinterpret_cast<char *>(destEntry + 4), 0x100, self.m_pMem->logs.log_strings[tail], meta.length);
    destEntry[258] = 0;

    tail++;
    readCount++;
  }
  self.m_pMem->logs.log_tail = tail;
  self.Unlock(SR_Log);
  return readCount;
}

void ShareManager::WriteLogString(this ShareManager &self, char level, const char *format, ...) {
  va_list args;
  va_start(args, format);

  self.Lock(SR_Log);
  uint8_t head = self.m_pMem->logs.log_head;

  char *destStr = self.m_pMem->logs.log_strings[head];
  int len = vsnprintf(destStr, 256, format, args);

  va_end(args);

  if (len < 0) {
    len = -1;
  }

  LogMetadata &meta = self.m_pMem->logs.log_meta[head];
  meta.instanceId = static_cast<uint8_t>(self.m_instanceId);
  meta.level = level;
  meta.pad = 0;

  int finalLen = 0xff;
  if (len + 1 < 0xff) {
    finalLen = len + 1;
  }
  meta.length = static_cast<uint8_t>(finalLen);

  self.m_pMem->logs.log_head++;
  if (self.m_pMem->logs.log_tail == self.m_pMem->logs.log_head) {
    self.m_pMem->logs.log_tail++;
  }

  self.Unlock(SR_Log);
  if (self.m_ipcEvents[SR_Log])
    IpcEvent_Set(self.m_ipcEvents[SR_Log]);
}

void ShareManager::ReadIntConfigSafe(this ShareManager &self, int configId, int *outValue) {
  if (!outValue)
    return;

  std::string strVal;
  self.ReadStringConfig(configId, strVal);

  if (!strVal.empty()) {
    try {
      *outValue = std::stoi(strVal);
    } catch (const std::invalid_argument &) {
      Util::DriverLog("ReadIntConfigSafe failed: configId={}, strVal={}", configId, strVal.c_str());
    }
  }
}

uint64_t ShareManager::UpdateProcessExitCodes(this ShareManager &self, void *outData) {
  self.Lock(SR_Common);
  for (int i = 0; i < SC_Max; i++) {
    DWORD pid = self.m_pMem->watchdog.pids[i];
    if (pid != 0 && pid != GetCurrentProcessId()) {
      HANDLE hProcess = OpenProcess(PROCESS_QUERY_INFORMATION, FALSE, pid);
      bool shouldClear = true;
      if (hProcess) {
        DWORD exitCode;
        if (GetExitCodeProcess(hProcess, &exitCode)) {
          if (exitCode == STILL_ACTIVE) {
            shouldClear = false;
          }
        }
        CloseHandle(hProcess);
      }
      if (shouldClear) {
        self.m_pMem->watchdog.pids[i] = 0;
        self.m_pMem->watchdog.states[i] = 0;
      }
    }
  }
  memcpy(outData, self.m_pMem->watchdog.timestamps, 176);
  self.Unlock(SR_Common);
  return 0;
}

uint32_t ShareManager::ReadCommon_0x10(this ShareManager &self, uint64_t *outData) {
  self.Lock(SR_Common);
  if (outData)
    *outData = self.m_pMem->common_0x10.data_0x10;
  uint32_t cnt = self.m_pMem->common_0x10.counter_0x8;
  self.Unlock(SR_Common);
  return cnt;
}

int ShareManager::WriteCommon_0x10(this ShareManager &self, uint64_t data) {
  self.Lock(SR_Common);
  self.m_pMem->common_0x10.data_0x10 = data;
  int cnt = ++self.m_pMem->common_0x10.counter_0x8;
  self.Unlock(SR_Common);
  return cnt;
}

int ShareManager::ReadCommon_0x18(this ShareManager &self) {
  self.Lock(SR_Common);
  int val = self.m_pMem->common_0x18.data_0x18;
  self.Unlock(SR_Common);
  return val;
}

uint32_t ShareManager::ReadCommon_0x9dd0(this ShareManager &self, uint64_t *outData) {
  self.Lock(SR_Common);
  if (outData)
    *outData = self.m_pMem->common_9dd0.data_0x9dd0;
  uint32_t cnt = self.m_pMem->common_9dd0.counter_0x9dc8;
  self.Unlock(SR_Common);
  return cnt;
}

int ShareManager::WriteCommon_0x9dd0(this ShareManager &self, uint64_t data) {
  self.Lock(SR_Common);
  self.m_pMem->common_9dd0.data_0x9dd0 = data;
  int cnt = ++self.m_pMem->common_9dd0.counter_0x9dc8;
  self.Unlock(SR_Common);
  return cnt;
}

uint32_t ShareManager::ReadCommon_0xc72c(this ShareManager &self, uint8_t *outData) {
  self.Lock(SR_Common);
  if (outData)
    *outData = self.m_pMem->common_c72c.data_0xc72c;
  uint32_t cnt = self.m_pMem->common_c72c.ctr;
  self.Unlock(SR_Common);
  return cnt;
}

int ShareManager::WriteCommon_0xc72c(this ShareManager &self, uint8_t data) {
  self.Lock(SR_Common);
  self.m_pMem->common_c72c.data_0xc72c = data;
  int cnt = ++self.m_pMem->common_c72c.ctr;
  self.Unlock(SR_Common);
  return cnt;
}

uint32_t ShareManager::ReadPlayareaSetup_0xc814(this ShareManager &self, void *outData) {
  self.Lock(SR_PlayareaSetupInfo);
  if (outData)
    *(uint32_t *)outData = self.m_pMem->playarea_setup_c814.data;
  uint32_t cnt = self.m_pMem->playarea_setup_c814.ctr;
  self.Unlock(SR_PlayareaSetupInfo);
  return cnt;
}

int ShareManager::WritePlayareaSetup_0xc814(this ShareManager &self, uint32_t data) {
  self.Lock(SR_PlayareaSetupInfo);
  self.m_pMem->playarea_setup_c814.data = data;
  int cnt = ++self.m_pMem->playarea_setup_c814.ctr;
  self.Unlock(SR_PlayareaSetupInfo);
  return cnt;
}

uint32_t ShareManager::ReadArmModel_0x84a4(this ShareManager &self, void *outData, int param) {
  self.Lock(SR_ArmModel);
  if (param < 0 || param >= 2) {
    self.Unlock(SR_ArmModel);
    return 0;
  }
  auto &slot = self.m_pMem->arm_model_84a4.slots[param];
  if (outData)
    memcpy(outData, slot.data, 0x50);
  uint32_t cnt = slot.ctr;
  self.Unlock(SR_ArmModel);
  return cnt;
}

int ShareManager::WriteArmModel_0x84a4(this ShareManager &self, void *data, int param) {
  self.Lock(SR_ArmModel);
  if (param < 0 || param >= 2) {
    self.Unlock(SR_ArmModel);
    return 0;
  }
  auto &slot = self.m_pMem->arm_model_84a4.slots[param];
  if (data)
    memcpy(slot.data, data, 0x50);
  int cnt = ++slot.ctr;
  self.Unlock(SR_ArmModel);
  return cnt;
}

uint32_t ShareManager::ReadBtQualityInfo_0x9170(this ShareManager &self, void *outData, int param) {
  self.Lock(SR_BluetoothQualityInfo);
  if (param < 0 || param >= 2) {
    self.Unlock(SR_BluetoothQualityInfo);
    return 0;
  }
  auto &slot = self.m_pMem->bt_qual_9170.slots[param];
  if (outData)
    memcpy(outData, slot.data, 80);
  uint32_t cnt = slot.ctr;
  self.Unlock(SR_BluetoothQualityInfo);
  return cnt;
}

int ShareManager::WriteBtQualityInfo_0x9170(this ShareManager &self, void *data, int param) {
  self.Lock(SR_BluetoothQualityInfo);
  if (param < 0 || param >= 2) {
    self.Unlock(SR_BluetoothQualityInfo);
    return 0;
  }
  auto &slot = self.m_pMem->bt_qual_9170.slots[param];
  if (data)
    memcpy(slot.data, data, 80);
  int cnt = ++slot.ctr;
  self.Unlock(SR_BluetoothQualityInfo);
  return cnt;
}

uint32_t ShareManager::AcquireImageWriteSlot(this ShareManager &self, long long *outImageBuffer, long long *outTrackingData) {
  uint32_t result = 0xFFFFFFFF;
  self.Lock(SR_Image);

  uint32_t writeIdx = self.m_writeIndex[0];

  for (int i = 0; i < 8; i++) {
    writeIdx = (writeIdx + 1) % 8;
    ImageSlotMeta &slot = self.m_pMem->image_meta.slots[writeIdx];

    if (slot.state < 2) {
      slot.state = 3;
      if (outImageBuffer)
        *outImageBuffer = reinterpret_cast<long long>(self.m_pMem->image_data.slots[writeIdx].data);
      if (outTrackingData)
        *outTrackingData = reinterpret_cast<long long>(slot.trackingData);
      result = writeIdx;
      break;
    }
  }
  self.Unlock(SR_Image);
  return result;
}

uint32_t ShareManager::AcquireInputWriteSlot(this ShareManager &self, int groupIdx, long long *outOffset) {
  if (groupIdx < 0 || groupIdx >= 3)
    return 0xFFFFFFFF;

  uint32_t result = 0xFFFFFFFF;
  int mutexIdx = SR_InputHmd + groupIdx;
  self.Lock(mutexIdx);

  uint32_t writeIdx = self.m_writeIndex_40[groupIdx];

  for (int i = 0; i < 64; i++) {
    writeIdx = (writeIdx + 1) % 64;
    InputSlotMeta &slot = self.m_pMem->input_meta.groups[groupIdx].slots[writeIdx];

    if (slot.state < 2) {
      slot.state = 3;
      if (outOffset)
        *outOffset = reinterpret_cast<long long>(self.m_pMem->input_data.slots[groupIdx][writeIdx]);
      result = writeIdx;
      break;
    }
  }
  self.Unlock(mutexIdx);
  return result;
}

uint32_t ShareManager::AcquirePoseWriteSlot(this ShareManager &self, int groupIdx, long long *outOffset) {
  if (groupIdx < 0 || groupIdx >= 3)
    return 0xFFFFFFFF;

  uint32_t result = 0xFFFFFFFF;
  int mutexIdx = SR_PoseHmd + groupIdx;
  self.Lock(mutexIdx);

  uint32_t writeIdx = self.m_writeIndex_28[groupIdx];

  for (int i = 0; i < 64; i++) {
    writeIdx = (writeIdx + 1) % 64;
    PoseSlotMeta &slot = self.m_pMem->pose_meta.groups[groupIdx].slots[writeIdx];

    if (slot.state < 2) {
      slot.state = 3;
      if (outOffset)
        *outOffset = reinterpret_cast<long long>(self.m_pMem->pose_data.slots[groupIdx][writeIdx].data);
      result = writeIdx;
      break;
    }
  }
  self.Unlock(mutexIdx);
  return result;
}

void ShareManager::CommitImageWriteSlot(this ShareManager &self, uint32_t slotIdx) {
  if (slotIdx >= 8)
    return;
  self.Lock(SR_Image);

  ImageSlotMeta &slot = self.m_pMem->image_meta.slots[slotIdx];
  slot.state = 1;
  slot.sequenceId = self.m_sequence[0]++;

  uint32_t property = *reinterpret_cast<uint32_t *>(&self.m_pMem->image_data.slots[slotIdx].data[0x8f0]);
  slot.property = property;

  uint32_t exposureVal;
  if (property < 0x100) {
    exposureVal = 0x3f800000; // 1.0f
  } else {
    exposureVal = *reinterpret_cast<uint32_t *>(&self.m_pMem->image_data.slots[slotIdx].data[0x8f4]);
  }
  slot.exposure = exposureVal;

  self.Unlock(SR_Image);
  self.m_writeIndex[0] = slotIdx;
  if (self.m_ipcEvents[SR_Image])
    IpcEvent_Set(self.m_ipcEvents[SR_Image]);
}

void ShareManager::CommitInputWriteSlot(this ShareManager &self, int groupIdx, uint32_t slotIdx) {
  if (groupIdx < 0 || groupIdx >= 3 || slotIdx >= 64)
    return;

  int mutexIdx = SR_InputHmd + groupIdx;
  self.Lock(mutexIdx);

  InputSlotMeta &slot = self.m_pMem->input_meta.groups[groupIdx].slots[slotIdx];
  slot.state = 1;
  slot.sequenceId = self.m_inputSequences[groupIdx]++;

  self.Unlock(mutexIdx);
  self.m_writeIndex_40[groupIdx] = slotIdx;
  if (self.m_ipcEvents[mutexIdx])
    IpcEvent_Set(self.m_ipcEvents[mutexIdx]);
}

void ShareManager::CommitPoseWriteSlot(this ShareManager &self, int groupIdx, uint32_t slotIdx, void *params) {
  if (groupIdx < 0 || groupIdx >= 3 || slotIdx >= 64)
    return;

  int mutexIdx = SR_PoseHmd + groupIdx;
  self.Lock(mutexIdx);

  PoseSlotMeta &slot = self.m_pMem->pose_meta.groups[groupIdx].slots[slotIdx];

  if (params) {
    memcpy(slot.params, params, 24);
  }

  slot.state = 1;
  slot.sequenceId = self.m_poseSequences[groupIdx]++;

  self.Unlock(mutexIdx);
  self.m_writeIndex_28[groupIdx] = slotIdx;
  if (self.m_ipcEvents[mutexIdx])
    IpcEvent_Set(self.m_ipcEvents[mutexIdx]);
}

uint32_t ShareManager::AcquireImageReadSlot(this ShareManager &self, long long *outImageBuffer, void *outTrackingData, void *outUnknown) {
  uint32_t result = 0xFFFFFFFF;
  uint32_t highestSeq = 0;
  self.Lock(SR_Image);

  for (uint32_t i = 0; i < 8; i++) {
    ImageSlotMeta &slot = self.m_pMem->image_meta.slots[i];
    if (slot.state == 1 || slot.state == 2) {
      if (result == 0xFFFFFFFF || slot.sequenceId >= highestSeq) {
        highestSeq = slot.sequenceId;
        result = i;
      }
    }
  }

  if (result != 0xFFFFFFFF) {
    ImageSlotMeta &slot = self.m_pMem->image_meta.slots[result];
    slot.state = 2;
    slot.read_counter += 1;
    if (outImageBuffer)
      *outImageBuffer = reinterpret_cast<long long>(self.m_pMem->image_data.slots[result].data);
    if (outTrackingData)
      memcpy(outTrackingData, slot.trackingData, 56);
    if (outUnknown)
      memcpy(outUnknown, &slot.exposure, 4);
  }
  self.Unlock(9);
  return result;
}

uint32_t ShareManager::AcquireInputReadSlot(this ShareManager &self, int groupIdx, long long *outOffset) {
  if (groupIdx < 0 || groupIdx >= 3)
    return 0xFFFFFFFF;

  uint32_t result = 0xFFFFFFFF;
  uint32_t highestSeq = 0;
  int mutexIdx = SR_InputHmd + groupIdx;
  self.Lock(mutexIdx);

  for (uint32_t i = 0; i < 64; i++) {
    InputSlotMeta &slot = self.m_pMem->input_meta.groups[groupIdx].slots[i];
    if (slot.state == 1 || slot.state == 2) {
      if (result == 0xFFFFFFFF || slot.sequenceId >= highestSeq) {
        highestSeq = slot.sequenceId;
        result = i;
      }
    }
  }

  if (result != 0xFFFFFFFF) {
    InputSlotMeta &slot = self.m_pMem->input_meta.groups[groupIdx].slots[result];
    slot.state = 2;
    slot.read_counter += 1;
    if (outOffset)
      *outOffset = reinterpret_cast<long long>(self.m_pMem->input_data.slots[groupIdx][result]);
  }
  self.Unlock(mutexIdx);
  return result;
}

uint32_t ShareManager::AcquirePoseReadSlot(this ShareManager &self, int groupIdx, long long *outOffset, void *outParams) {
  if (groupIdx < 0 || groupIdx >= 3)
    return 0xFFFFFFFF;

  uint32_t result = 0xFFFFFFFF;
  uint32_t highestSeq = 0;
  int mutexIdx = SR_PoseHmd + groupIdx;
  self.Lock(mutexIdx);

  for (uint32_t i = 0; i < 64; i++) {
    PoseSlotMeta &slot = self.m_pMem->pose_meta.groups[groupIdx].slots[i];
    if (slot.state == 1 || slot.state == 2) {
      if (result == 0xFFFFFFFF || slot.sequenceId >= highestSeq) {
        highestSeq = slot.sequenceId;
        result = i;
      }
    }
  }

  if (result != 0xFFFFFFFF) {
    PoseSlotMeta &slot = self.m_pMem->pose_meta.groups[groupIdx].slots[result];
    slot.state = 2;
    slot.read_counter += 1;
    if (outOffset)
      *outOffset = reinterpret_cast<long long>(self.m_pMem->pose_data.slots[groupIdx][result].data);
    if (outParams)
      memcpy(outParams, slot.params, 24);
  }
  self.Unlock(mutexIdx);
  return result;
}

void ShareManager::CommitImageReadSlot(this ShareManager &self, uint32_t slotIdx) {
  if (slotIdx >= 8)
    return;
  self.Lock(SR_Image);
  ImageSlotMeta &slot = self.m_pMem->image_meta.slots[slotIdx];
  if (slot.read_counter > 0) {
    slot.read_counter -= 1;
    if (slot.read_counter == 0)
      slot.state = 1;
  }
  self.Unlock(SR_Image);
}

void ShareManager::CommitInputReadSlot(this ShareManager &self, int groupIdx, uint32_t slotIdx) {
  if (groupIdx < 0 || groupIdx >= 3 || slotIdx >= 64)
    return;

  int mutexIdx = SR_InputHmd + groupIdx;
  self.Lock(mutexIdx);
  InputSlotMeta &slot = self.m_pMem->input_meta.groups[groupIdx].slots[slotIdx];
  if (slot.read_counter > 0) {
    slot.read_counter -= 1;
    if (slot.read_counter == 0) {
      slot.state = 1;
    }
  }
  self.Unlock(mutexIdx);
}

void ShareManager::CommitPoseReadSlot(this ShareManager &self, int groupIdx, uint32_t slotIdx) {
  if (groupIdx < 0 || groupIdx >= 3 || slotIdx >= 64)
    return;

  int mutexIdx = SR_PoseHmd + groupIdx;
  self.Lock(mutexIdx);
  PoseSlotMeta &slot = self.m_pMem->pose_meta.groups[groupIdx].slots[slotIdx];
  if (slot.read_counter > 0) {
    slot.read_counter -= 1;
    if (slot.read_counter == 0) {
      slot.state = 1;
    }
  }
  self.Unlock(mutexIdx);
}

uint64_t ShareManager::TryAcquireShareMutex(this ShareManager &self, uint32_t typeIndex) {
  if (typeIndex > 1) {
    return 0xFFFFFFFFFFFFFF00;
  }

  int realIdx = SR_LibpadRequestSteamVRPlugin + typeIndex;
  if (!self.m_ipcMutexes[realIdx]) {
    return 0xFFFFFFFFFFFFFF00;
  }

  if (self.m_shareMutexes[typeIndex].IsFreeOrOwned()) {
    return 0;
  }

  return 1;
}

uint32_t ShareManager::AcquireShareMutex(this ShareManager &self, uint32_t typeIndex) {
  if (typeIndex > 1) {
    return 0xFFFFFFFF;
  }

  self.m_shareMutexes[typeIndex].Lock();

  return 0;
}

uint32_t ShareManager::ReleaseShareMutex(this ShareManager &self, uint32_t typeIndex) {
  if (typeIndex > 1) {
    return 0xFFFFFFFF;
  }

  self.m_shareMutexes[typeIndex].Unlock();

  return 0;
}

uint32_t ShareManager::TryAcquireLibpadMutex(this ShareManager &self) {
  if (!self.m_ipcMutexes[SR_LibpadAccess]) {
    return 0xFFFFFFFF;
  }

  if (self.m_libpadMutex.TryLock()) {
    return 0;
  }

  return 0xFFFFFFFF;
}

int ShareManager::ReleaseLibpadMutex(this ShareManager &self) {
  if (!self.m_ipcMutexes[SR_LibpadAccess]) {
    return 0;
  }

  self.m_libpadMutex.Unlock();

  return 0;
}

void ShareManager::ReleaseMutexByIndex(this ShareManager &self, int index) {
  if (index == SR_LibpadAccess) {
    self.ReleaseLibpadMutex();
    return;
  }
  if (index == SR_LibpadRequestSteamVRPlugin) {
    self.ReleaseShareMutex(0);
    return;
  }
  if (index == SR_LibpadRequestAssistantApp) {
    self.ReleaseShareMutex(1);
    return;
  }

  if (index >= 0 && index < SR_Max && self.m_ipcMutexes[index]) {
    IpcMutex_Unlock(self.m_ipcMutexes[index]);
  }
}

void ShareManager::ClearLogEventFlag(this ShareManager &self) {
  self.Lock(SR_Log);
  self.m_pMem->logs.log_head = 0;
  self.Unlock(SR_Log);
}

void ShareManager::SetGlobalEventFlag(this ShareManager &self, uint64_t flags) {
  GlobalEventContext *pCtx = self.m_pEventContext;
  if (pCtx) {
    if (pCtx->ipcMutex)
      IpcMutex_Lock(pCtx->ipcMutex);
    *pCtx->pSharedFlags |= flags;
    if (pCtx->ipcEvent)
      IpcEvent_Set(pCtx->ipcEvent);
    if (pCtx->ipcMutex)
      IpcMutex_Unlock(pCtx->ipcMutex);
  }
}

uint32_t ShareManager::ReadPlayareaResult(this ShareManager &self, void *outData) {
  self.Lock(SR_PlayareaResult);

  uint32_t bestSlot = 0xFFFFFFFF;
  uint32_t highestSeq = 0;

  for (uint32_t i = 0; i < 8; i++) {
    PlayareaSlotState &slot = self.m_pMem->playarea_meta.playarea_states[i];
    if (slot.state == 1 || slot.state == 2) {
      if (bestSlot == 0xFFFFFFFF || slot.sequenceId >= highestSeq) {
        highestSeq = slot.sequenceId;
        bestSlot = i;
      }
    }
  }

  if (bestSlot != 0xFFFFFFFF) {
    PlayareaSlotState &slot = self.m_pMem->playarea_meta.playarea_states[bestSlot];
    slot.state = 2;

    if (outData) {
      memcpy(outData, self.m_pMem->playarea_ring.playarea_slots[bestSlot], 64);
    }

    slot.state = 1;
  }

  self.Unlock(SR_PlayareaResult);
  return bestSlot;
}

uint32_t ShareManager::WritePlayareaResult(this ShareManager &self, void *data) {
  self.Lock(SR_PlayareaResult);

  uint32_t targetSlot = 0xFFFFFFFF;
  uint32_t highestSeq = 0;

  for (uint32_t i = 0; i < 8; i++) {
    PlayareaSlotState &slot = self.m_pMem->playarea_meta.playarea_states[i];
    if (slot.sequenceId > highestSeq) {
      highestSeq = slot.sequenceId;
    }

    if (targetSlot == 0xFFFFFFFF && slot.state < 2) {
      targetSlot = i;
    }
  }

  if (targetSlot == 0xFFFFFFFF)
    targetSlot = 0;

  PlayareaSlotState &slot = self.m_pMem->playarea_meta.playarea_states[targetSlot];
  slot.state = 3;

  if (data) {
    memcpy(self.m_pMem->playarea_ring.playarea_slots[targetSlot], data, 64);
  }

  slot.sequenceId = highestSeq + 1;
  slot.state = 1;

  self.Unlock(SR_PlayareaResult);
  return targetSlot;
}

int ShareManager::WriteFwInfo2_0x9280(this ShareManager &self, void *data) {
  self.Lock(SR_FwInfo2);
  if (data) {
    memcpy(self.m_pMem->fw_info2_9280.data, data, sizeof(self.m_pMem->fw_info2_9280.data));
  }
  int cnt = ++self.m_pMem->fw_info2_9280.ctr;
  self.Unlock(SR_FwInfo2);

  self.SetGlobalEventFlag(0x40);

  return cnt;
}

uint32_t ShareManager::WaitShareEvent(this ShareManager &self, int groupIdx, DWORD timeoutMs) {
  if (groupIdx < 0 || groupIdx >= SR_Max || !self.m_ipcEvents[groupIdx])
    return 0xFFFFFFFF;
  bool success = IpcEvent_Wait(self.m_ipcEvents[groupIdx], timeoutMs);
  if (success) {
    IpcEvent_Reset(self.m_ipcEvents[groupIdx]);
    return 0;
  }
  return 0xFFFFFFFF;
}

#define IMPL_READ_MEMCPY_RET(FuncName, GroupIdx, MemGroup, MemData, Size)                                                                                      \
  uint32_t ShareManager::FuncName(this ShareManager &self, void *outData) {                                                                                    \
    self.Lock(GroupIdx);                                                                                                                                       \
    if (outData)                                                                                                                                               \
      memcpy(outData, self.m_pMem->MemGroup.MemData, Size);                                                                                                    \
    uint32_t cnt = self.m_pMem->MemGroup.ctr;                                                                                                                  \
    self.Unlock(GroupIdx);                                                                                                                                     \
    return cnt;                                                                                                                                                \
  }

#define IMPL_READ_MEMCPY_OUT(FuncName, GroupIdx, MemGroup, MemData, Size)                                                                                      \
  uint32_t ShareManager::FuncName(this ShareManager &self, void *outData, uint32_t *outCounter) {                                                              \
    self.Lock(GroupIdx);                                                                                                                                       \
    if (outData)                                                                                                                                               \
      memcpy(outData, self.m_pMem->MemGroup.MemData, Size);                                                                                                    \
    if (outCounter)                                                                                                                                            \
      *outCounter = self.m_pMem->MemGroup.ctr;                                                                                                                 \
    self.Unlock(GroupIdx);                                                                                                                                     \
    return 0;                                                                                                                                                  \
  }

#define IMPL_WRITE_MEMCPY(FuncName, GroupIdx, MemGroup, MemData, Size)                                                                                         \
  int ShareManager::FuncName(this ShareManager &self, void *data) {                                                                                            \
    self.Lock(GroupIdx);                                                                                                                                       \
    if (data)                                                                                                                                                  \
      memcpy(self.m_pMem->MemGroup.MemData, data, Size);                                                                                                       \
    int cnt = ++self.m_pMem->MemGroup.ctr;                                                                                                                     \
    self.Unlock(GroupIdx);                                                                                                                                     \
    return cnt;                                                                                                                                                \
  }

// Group 1: Status
IMPL_READ_MEMCPY_RET(ReadStatus_0x2c, 1, status_0x2c, data_0x2c, sizeof(self.m_pMem->status_0x2c.data_0x2c))
IMPL_WRITE_MEMCPY(WriteStatus_0x2c, 1, status_0x2c, data_0x2c, sizeof(self.m_pMem->status_0x2c.data_0x2c))

// Group 2: Calibration Blocks
IMPL_READ_MEMCPY_OUT(ReadCalib_0x70, 2, calib_0x70, data_0x70, sizeof(self.m_pMem->calib_0x70.data_0x70))
IMPL_WRITE_MEMCPY(WriteCalib_0x70, 2, calib_0x70, data_0x70, sizeof(self.m_pMem->calib_0x70.data_0x70))
IMPL_READ_MEMCPY_OUT(ReadCalib_0x170, 2, calib_170, data_0x170, sizeof(self.m_pMem->calib_170.data_0x170))
IMPL_WRITE_MEMCPY(WriteCalib_0x170, 2, calib_170, data_0x170, sizeof(self.m_pMem->calib_170.data_0x170))
IMPL_READ_MEMCPY_OUT(ReadCalib_0x370, 2, calib_370, data_0x370, sizeof(self.m_pMem->calib_370.data_0x370))
IMPL_WRITE_MEMCPY(WriteCalib_0x370, 2, calib_370, data_0x370, sizeof(self.m_pMem->calib_370.data_0x370))
IMPL_READ_MEMCPY_OUT(ReadCalib_0x3f0, 2, calib_3f0, data_0x3f0, sizeof(self.m_pMem->calib_3f0.data_0x3f0))
IMPL_WRITE_MEMCPY(WriteCalib_0x3f0, 2, calib_3f0, data_0x3f0, sizeof(self.m_pMem->calib_3f0.data_0x3f0))
IMPL_READ_MEMCPY_OUT(ReadCalib_0x41c, 2, calib_41c, data_0x41c, sizeof(self.m_pMem->calib_41c.data_0x41c))
IMPL_WRITE_MEMCPY(WriteCalib_0x41c, 2, calib_41c, data_0x41c, sizeof(self.m_pMem->calib_41c.data_0x41c))
IMPL_READ_MEMCPY_OUT(ReadCalib_0x50c, 2, calib_50c, data_0x50c, sizeof(self.m_pMem->calib_50c.data_0x50c))
IMPL_WRITE_MEMCPY(WriteCalib_0x50c, 2, calib_50c, data_0x50c, sizeof(self.m_pMem->calib_50c.data_0x50c))
IMPL_READ_MEMCPY_OUT(ReadCalib_0xd0c, 2, calib_d0c, data_0xd0c, sizeof(self.m_pMem->calib_d0c.data_0xd0c))
IMPL_WRITE_MEMCPY(WriteCalib_0xd0c, 2, calib_d0c, data_0xd0c, sizeof(self.m_pMem->calib_d0c.data_0xd0c))

// Groups 12-19: Configs & Info
IMPL_READ_MEMCPY_OUT(ReadImageSetting_0x803c, 12, img_setting_803c, data, sizeof(self.m_pMem->img_setting_803c.data))
IMPL_WRITE_MEMCPY(WriteImageSetting_0x803c, 12, img_setting_803c, data, sizeof(self.m_pMem->img_setting_803c.data))
IMPL_READ_MEMCPY_OUT(ReadBlobConfig_0x8058, 13, blob_cfg_8058, data, sizeof(self.m_pMem->blob_cfg_8058.data))
IMPL_WRITE_MEMCPY(WriteBlobConfig_0x8058, 13, blob_cfg_8058, data, sizeof(self.m_pMem->blob_cfg_8058.data))
IMPL_READ_MEMCPY_OUT(ReadIrCamSetting_0x807c, 14, ir_cam_807c, data, sizeof(self.m_pMem->ir_cam_807c.data))
IMPL_WRITE_MEMCPY(WriteIrCamSetting_0x807c, 14, ir_cam_807c, data, sizeof(self.m_pMem->ir_cam_807c.data))
IMPL_READ_MEMCPY_RET(ReadContConfig_0x8098, 15, cont_cfg_8098, data, sizeof(self.m_pMem->cont_cfg_8098.data))
IMPL_WRITE_MEMCPY(WriteContConfig_0x8098, 15, cont_cfg_8098, data, sizeof(self.m_pMem->cont_cfg_8098.data))
IMPL_READ_MEMCPY_RET(ReadContLedInfo_0x855c, 17, cont_led_855c, data, sizeof(self.m_pMem->cont_led_855c.data))
IMPL_WRITE_MEMCPY(WriteContLedInfo_0x855c, 17, cont_led_855c, data, sizeof(self.m_pMem->cont_led_855c.data))
IMPL_READ_MEMCPY_RET(ReadFwInfo1_0x922c, 19, fw_info1_922c, data, sizeof(self.m_pMem->fw_info1_922c.data))
IMPL_WRITE_MEMCPY(WriteFwInfo1_0x922c, 19, fw_info1_922c, data, sizeof(self.m_pMem->fw_info1_922c.data))

// Groups 20-24: Application & Debug
IMPL_READ_MEMCPY_RET(ReadFwInfo2_0x9280, 20, fw_info2_9280, data, sizeof(self.m_pMem->fw_info2_9280.data))
IMPL_READ_MEMCPY_RET(ReadVrDialog_0x9ae0, 21, vr_dialog_9ae0, data, sizeof(self.m_pMem->vr_dialog_9ae0.data))
IMPL_WRITE_MEMCPY(WriteVrDialog_0x9ae0, 21, vr_dialog_9ae0, data, sizeof(self.m_pMem->vr_dialog_9ae0.data))
IMPL_READ_MEMCPY_RET(ReadApplication_0x9b3c, 22, app_9b3c, data, sizeof(self.m_pMem->app_9b3c.data))
IMPL_READ_MEMCPY_RET(ReadVrTraceData_0x9b7c, 23, vr_trace_9b7c, data, sizeof(self.m_pMem->vr_trace_9b7c.data))
IMPL_WRITE_MEMCPY(WriteVrTraceData_0x9b7c, 23, vr_trace_9b7c, data, sizeof(self.m_pMem->vr_trace_9b7c.data))
IMPL_READ_MEMCPY_RET(ReadDebugData_0x9c8c, 24, dbg_data_9c8c, data, sizeof(self.m_pMem->dbg_data_9c8c.data))
IMPL_WRITE_MEMCPY(WriteDebugData_0x9c8c, 24, dbg_data_9c8c, data, sizeof(self.m_pMem->dbg_data_9c8c.data))

// Groups 30-34: Telemetry & Setup
IMPL_READ_MEMCPY_RET(ReadSceneInfo_0x9de4, 33, scene_info_9de4, data, sizeof(self.m_pMem->scene_info_9de4.data))
IMPL_WRITE_MEMCPY(WriteSceneInfo_0x9de4, 33, scene_info_9de4, data, sizeof(self.m_pMem->scene_info_9de4.data))
IMPL_READ_MEMCPY_RET(ReadTelDevInfo_0xa984, 30, tel_dev_a984, data, sizeof(self.m_pMem->tel_dev_a984.data))
IMPL_WRITE_MEMCPY(WriteTelDevInfo_0xa984, 30, tel_dev_a984, data, sizeof(self.m_pMem->tel_dev_a984.data))
IMPL_READ_MEMCPY_RET(ReadTelTrkInfo_0xaaa0, 31, tel_trk_aaa0, data, sizeof(self.m_pMem->tel_trk_aaa0.data))
IMPL_WRITE_MEMCPY(WriteTelTrkInfo_0xaaa0, 31, tel_trk_aaa0, data, sizeof(self.m_pMem->tel_trk_aaa0.data))
IMPL_READ_MEMCPY_RET(ReadTelPcInfo_0xc600, 32, tel_pc_c600, data, sizeof(self.m_pMem->tel_pc_c600.data))
IMPL_WRITE_MEMCPY(WriteTelPcInfo_0xc600, 32, tel_pc_c600, data, sizeof(self.m_pMem->tel_pc_c600.data))
IMPL_READ_MEMCPY_RET(ReadInitSetup_0xc73c, 34, init_setup_c73c, data, sizeof(self.m_pMem->init_setup_c73c.data))
IMPL_WRITE_MEMCPY(WriteInitSetup_0xc73c, 34, init_setup_c73c, data, sizeof(self.m_pMem->init_setup_c73c.data))

unsigned __stdcall ShareManager::WorkerThread_Unconditional(void *pContext) {
  GlobalEventContext *pEvtStruct = *reinterpret_cast<GlobalEventContext **>(pContext);
  delete reinterpret_cast<GlobalEventContext **>(pContext);

  if (pEvtStruct) {
    ShareManager::WaitDynamicEvent(&pEvtStruct);
    if (pEvtStruct->threadInfo) {
      CloseHandle(*reinterpret_cast<HANDLE *>(pEvtStruct->threadInfo));
      operator delete(pEvtStruct->threadInfo);
    }
    delete pEvtStruct;
  }

  return 0;
}

unsigned __stdcall ShareManager::EvfWorkerThread_Conditional(void *pContext) {
  ShareManager *self = *reinterpret_cast<ShareManager **>(pContext);

  delete reinterpret_cast<ShareManager **>(pContext);
  if (!self)
    return 0;

  char sharedMemBuffer[256];

  while (!self->m_exitThreads) {
    for (int i = 0; i < SC_Max; i++) {
      ConfigMapping &map = self->m_configMappings[i];

      std::string sharedStr;
      self->ReadStringConfig(i, sharedStr);

      strncpy_s(sharedMemBuffer, sizeof(sharedMemBuffer), sharedStr.c_str(), _TRUNCATE);

      if (strncmp(map.CachedValue, sharedMemBuffer, sizeof(map.CachedValue)) != 0) {
        WritePrivateProfileStringA(map.AppName, map.KeyName, sharedMemBuffer, self->m_configIniPath);

        strncpy_s(map.CachedValue, sizeof(map.CachedValue), sharedMemBuffer, _TRUNCATE);
      }
    }

    Sleep(1000);
  }

  return 0;
}

void ShareManager::InitializeSub_ad30(this ShareManager &self) {
  self.m_configMappings[SC_LastRecordedDateTime] = {"SafetyNotice", "LastRecordedDateTime", ""};
  self.m_configMappings[SC_VibrationStrength] = {"Controller", "VibrationStrength", "1"};
  self.m_configMappings[SC_ScreenBrightness] = {"HMD", "ScreenBrightness", "31"};
  self.m_configMappings[SC_LimitDisplay] = {"SafetyNotice", "LimitDisplay", ""};
  self.m_configMappings[SC_MemorySize] = {"VRTrace", "MemorySize", "1"};
  self.m_configMappings[SC_Done] = {"InitialSetup", "Done", "1"}; // Default is 1 because PSVR2TK already skips Sony's setup.
  self.m_configMappings[SC_State] = {"DataCollection", "State", ""};
  self.m_configMappings[SC_CrashCount] = {"HMD", "CrashCount", "0"};
  self.m_configMappings[SC_OutputLog] = {"Telemetry", "OutputLog", "0"};
  self.m_configMappings[SC_BluetoothNotification] = {"Controller", "BluetoothNotification", "1"};
  self.m_configMappings[SC_Status] = {"Controller", "Status", "0"};

  char szPath[MAX_PATH];
  if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, szPath))) {
    std::string dirPath = std::string(szPath) + "\\Sony\\PlayStation VR2";
    SHCreateDirectoryExA(NULL, dirPath.c_str(), NULL);

    std::string iniPath = dirPath + "\\config.ini";
    strncpy_s(self.m_configIniPath, sizeof(self.m_configIniPath), iniPath.c_str(), _TRUNCATE);

    // Check if the file exists, if not create it empty
    DWORD dwAttrib = GetFileAttributesA(iniPath.c_str());
    if (dwAttrib == INVALID_FILE_ATTRIBUTES) {
      FILE *f = nullptr;
      if (fopen_s(&f, iniPath.c_str(), "w") == 0 && f) {
        fclose(f);
      }
      Util::DriverLog("[VrTracker2] create user config.ini");
    }
  } else {
    snprintf(self.m_configIniPath, sizeof(self.m_configIniPath), "\\Sony\\PlayStation VR2\\config.ini");
  }

  self.m_exitThreads = false;

  for (int i = 0; i < SC_Max; i++) {
    self.m_configMappings[i].CachedValue[0] = '\0';
  }
}

void ShareManager::InitializeSub_bdb0(this ShareManager &self) {
  uint64_t baseAddress = HmdDriverLoader::Instance()->GetBaseAddress();
  auto ValidateConfig = reinterpret_cast<bool (*)(const ShareManager *self, int configId, const char *str)>(baseAddress + 0x15c230);

  char tempBuffer[256];
  std::string sharedStr;

  for (int i = 0; i < SC_Max; i++) {
    ConfigMapping &map = self.m_configMappings[i];
    tempBuffer[0] = '\0';

    GetPrivateProfileStringA(map.AppName, map.KeyName, "", tempBuffer, sizeof(tempBuffer), self.m_configIniPath);

    // Remove carriage return / line feed if present
    tempBuffer[strcspn(tempBuffer, "\r\n")] = 0;

    // If the INI config is empty or invalid, fall back to default
    if (tempBuffer[0] == '\0' || !ValidateConfig(&self, i, tempBuffer)) {
      WritePrivateProfileStringA(map.AppName, map.KeyName, map.DefaultValue, self.m_configIniPath);
      strncpy_s(tempBuffer, sizeof(tempBuffer), map.DefaultValue, _TRUNCATE);
    }

    // Read current shared memory config
    sharedStr.clear();
    self.ReadStringConfig(i, sharedStr);

    // If shared memory config is empty or invalid, write INI config to shared
    // memory
    if (sharedStr.empty() || !ValidateConfig(&self, i, sharedStr.c_str())) {
      self.WriteConfigString(i, tempBuffer);
      strncpy_s(map.CachedValue, sizeof(map.CachedValue), tempBuffer, _TRUNCATE);
    } else {
      strncpy_s(map.CachedValue, sizeof(map.CachedValue), sharedStr.c_str(), _TRUNCATE);
    }
  }
}

void ShareManager::Lock(int idx) {
  if (idx >= 0 && idx < 36 && m_ipcMutexes[idx]) {
    IpcMutex_Lock(m_ipcMutexes[idx]);
  }
}

void ShareManager::Unlock(int idx) {
  if (idx >= 0 && idx < 36 && m_ipcMutexes[idx]) {
    IpcMutex_Unlock(m_ipcMutexes[idx]);
  }
}