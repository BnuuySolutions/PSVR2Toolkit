#pragma once

#include <cstdint>
#include <string>

#include <functional>
#include <vector>
#include <windows.h>
#include <mutex>
#include <condition_variable>

class IIpcMutex;
class IIpcEvent;
class IIpcSharedMemory;

struct ProcessOwnedMutex {
  std::mutex localMtx;
  std::condition_variable cv;
  IIpcMutex *ipcMutex = nullptr;
  bool isAcquired = false;
  bool isLocking = false;

  void Lock();
  bool TryLock();
  void Unlock();
  bool IsFreeOrOwned();
};

#pragma pack(push, 1)

struct EventStruct {
  void *field2_0x10;
  HANDLE *Evt;
  HANDLE *Mtx;
  char field27_0x30;
};

struct InputSlotMeta {
  uint32_t state;        // 0=Free, 1=Ready, 2=Reading, 3=Writing
  uint32_t read_counter; // Number of active readers
  uint32_t sequenceId;   // Sequence counter
};

struct InputGroupMeta {
  InputSlotMeta slots[64];
};

struct PoseSlotMeta {
  uint32_t state; // 0=Free, 1=Ready, 2=Reading, 3=Writing
  uint32_t read_counter;
  uint32_t sequenceId;
  uint32_t pad;
  uint8_t params[24];
};

struct PoseGroupMeta {
  PoseSlotMeta slots[64];
};

struct ImageSlotMeta {
  uint32_t state;
  uint32_t read_counter;
  uint32_t sequenceId;
  uint8_t trackingData[56];
  uint32_t property;
  uint32_t exposure;
  uint8_t pad[2092]; // Total size is 2168 (0x878)
};

struct ImageSlotData {
  uint8_t data[0x200100];
};

struct PoseSlotData {
  uint8_t data[184];
};

struct PlayareaSlotState {
  uint32_t state;
  uint32_t pad;
  uint32_t sequenceId;
};

struct LogMetadata {
  uint8_t instanceId;
  uint8_t level;
  uint8_t length;
  uint8_t pad;
};

struct VRSharedMemory {
  union {
    uint8_t raw[0x2000000];
    struct {
      // Offset 0x0
      uint8_t pad_0x0[8];

      // Offset 0x8
      struct {
        uint32_t counter_0x8;
        uint32_t pad_0xC;
        uint64_t data_0x10;
      } common_0x10;

      // Offset 0x18
      struct {
        uint16_t data_0x18;
      } common_0x18;

      // Offset 0x1A - 0x28
      uint8_t pad_0x1a[14];

      // Offset 0x28
      struct {
        uint32_t ctr;
        uint32_t data_0x2c[9];
      } status_0x2c;

      // Offset 0x50 - 0x54
      uint8_t pad_0x50[4];

      // Offset 0x54 - 0x150C
      struct {
        uint32_t ctr_70;
        uint32_t ctr_170;
        uint32_t ctr_370;
        uint32_t ctr_3f0;
        uint32_t ctr_41c;
        uint32_t ctr_50c;
        uint32_t ctr_d0c;

        uint8_t data_0x70[0x100];
        uint8_t data_0x170[0x200];
        uint8_t data_0x370[120];
        uint8_t data_0x3f0[52];
        uint8_t data_0x41c[0xF0];
        uint8_t data_0x50c[0x800];
        uint8_t data_0xd0c[0x800];
      } calib;

      // Offset 0x150C - 0x1E0C
      struct {
        InputGroupMeta groups[3];
      } input_meta;

      // Offset 0x1E0C - 0x1E10
      uint8_t pad_0x1e0c[4];

      // Offset 0x1E10
      struct {
        PoseGroupMeta groups[3];
      } pose_meta;

      // Offset 0x3C10
      struct {
        ImageSlotMeta slots[8];
      } image_meta;

      // Offset 0x7FD0
      struct {
        PlayareaSlotState playarea_states[8];
      } playarea_meta;

      // Offset 0x8030 - 0x8038
      uint8_t pad_0x8030[8];

      // Offset 0x8038
      struct {
        uint32_t ctr;
        uint32_t data[4];
      } img_setting_803c;

      // Offset 0x804C - 0x8054
      uint8_t pad_0x804c[8];

      // Offset 0x8054
      struct {
        uint32_t ctr;
        uint32_t data[6];
      } blob_cfg_8058;

      // Offset 0x8070 - 0x8078
      uint8_t pad_0x8070[8];

      // Offset 0x8078
      struct {
        uint32_t ctr;
        uint32_t data[4];
      } ir_cam_807c;

      // Offset 0x808C - 0x8094
      uint8_t pad_0x808c[8];

      // Offset 0x8094
      struct {
        uint32_t ctr;
        uint8_t data[0x400];
      } cont_cfg_8098;

      // Offset 0x8498 - 0x84A0
      uint8_t pad_0x8498[8];

      // Offset 0x84A0
      struct {
        struct ArmModelSlot {
          uint32_t ctr;
          uint8_t data[80];
          uint8_t pad[8];
        } slots[2];
      } arm_model_84a4;

      // Offset 0x8558
      struct {
        uint32_t ctr;
        uint8_t data[0xC00];
      } cont_led_855c;

      // Offset 0x915C - 0x9168
      uint8_t pad_0x915c[12];

      // Offset 0x9168
      struct {
        struct BtQualitySlot {
          uint32_t ctr;
          uint32_t pad;
          uint8_t data[80];
          uint8_t unused_pad[8];
        } slots[2];
      } bt_qual_9170;

      // Offset 0x9228
      struct {
        uint32_t ctr;
        uint8_t data[0x48];
      } fw_info1_922c;

      // Offset 0x9274 - 0x927C
      uint8_t pad_0x9274[8];

      // Offset 0x927C
      struct {
        uint32_t ctr;
        uint8_t data[0x850];
      } fw_info2_9280;

      // Offset 0x9AD0 - 0x9AD8
      uint8_t pad_0x9ad0[8];

      // Offset 0x9AD8
      struct {
        uint32_t ctr;
        uint32_t pad2;
        uint8_t data[0x50];
      } vr_dialog_9ae0;

      // Offset 0x9B30 - 0x9B38
      uint8_t pad_0x9b30[8];

      // Offset 0x9B38
      struct {
        uint32_t ctr;
        uint8_t data[49];
      } app_9b3c;

      // Offset 0x9B6D - 0x9B78
      uint8_t pad_0x9b6d[11];

      // Offset 0x9B78
      struct {
        uint32_t ctr;
        uint8_t data[0x80];
      } vr_trace_9b7c;

      // Offset 0x9BFC - 0x9C88
      uint8_t pad_0x9bfc[140];

      // Offset 0x9C88
      struct {
        uint32_t ctr;
        uint8_t data[0x131];
      } dbg_data_9c8c;

      // Offset 0x9DBD - 0x9DC8
      uint8_t pad_0x9dbd[11];

      // Offset 0x9DC8
      struct {
        uint32_t counter_0x9dc8;
        uint32_t pad_0x9dcc;
        uint64_t data_0x9dd0;
      } common_9dd0;

      // Offset 0x9DD8 - 0x9DE0
      uint8_t pad_0x9dd8[8];

      // Offset 0x9DE0
      struct {
        uint32_t ctr;
        uint32_t data[4];
      } scene_info_9de4;

      // Offset 0x9DF4 - 0x9DFC
      uint8_t pad_0x9df4[8];

      // Offset 0x9DFC (yes, config and telemetry overlap)
      union {
        struct {
          struct ConfigSlot {
            uint32_t counter;
            char stringData[264];
          } str_configs[16];
        } configs_9e00;

        struct {
          uint8_t pad_to_a980[0xB84];
          struct {
            uint32_t ctr;
            uint8_t data[0x90];
          } tel_dev_a984;

          uint8_t pad_to_aa9c[0x88];
          struct {
            uint32_t ctr;
            uint8_t data[0x1B50];
          } tel_trk_aaa0;

          uint8_t pad_to_c5f8[8];
          struct {
            uint32_t ctr;
            uint8_t data[0xA0];
          } tel_pc_c600;
        };
      };

      // Offset 0xC69C - 0xC728
      uint8_t pad_to_c728[0x8C];

      // Offset 0xC728
      struct {
        uint32_t ctr;
        uint8_t data_0xc72c;
      } common_c72c;

      // Offset 0xC72D - 0xC738
      uint8_t pad_to_c738[11];

      // Offset 0xC738
      struct {
        uint32_t ctr;
        uint8_t data[200];
      } init_setup_c73c;

      // Offset 0xC804 - 0xC810
      uint8_t pad_to_c810[12];

      // Offset 0xC810
      struct {
        uint32_t ctr;
        uint32_t data;
      } playarea_setup_c814;

      // Offset 0xC818 - 0xFFC00
      uint8_t pad_to_ffc00[0xF33E8];

      // Offset 0xFFC00
      struct {
        double timestamps[11];
        DWORD pids[11];
        uint32_t states[11];
      } watchdog;

      // Offset 0xFFCB0 - 0x100000
      uint8_t pad_to_100000[0x350];

      // Offset 0x100000
      struct {
        uint8_t slots[3][64][64];
      } input_data;

      // Offset 0x103000
      union {
        struct {
          PoseSlotData slots[4][64];
        } pose_data;

        struct {
          uint8_t pad_to_10ba00[0x8A00];
          ImageSlotData slots[8];
        } image_data;
      };

      // Offset 0x110C200 - 0x110C208
      uint8_t pad_to_110c208[8];

      // Offset 0x110C208
      struct {
        uint8_t playarea_slots[8][64];
      } playarea_ring;

      // Offset 0x110C408
      struct {
        uint8_t log_head;
        uint8_t log_tail;
        uint8_t pad2[0xFE];
        LogMetadata log_meta[256];
        uint8_t pad3[0x3fb00];
        char log_strings[256][0x100];
      } logs;

      // Pad to end of 32MB
      uint8_t pad_to_end[0xEA3BF8];
    };
  };
};

#pragma pack(pop)

#define DEF_READ_MEMCPY_RET(FuncName) uint32_t FuncName(this ShareManager &self, void *outData);

#define DEF_READ_MEMCPY_OUT(FuncName) uint32_t FuncName(this ShareManager &self, void *outData, uint32_t *outCounter);

#define DEF_WRITE_MEMCPY(FuncName) int FuncName(this ShareManager &self, void *data);

enum ShareResourceIndex {
  SR_Common = 0,
  SR_Status = 1,
  SR_Calib = 2,
  SR_InputHmd = 3,
  SR_InputContR = 4,
  SR_InputContL = 5,
  SR_PoseHmd = 6,
  SR_PoseContR = 7,
  SR_PoseContL = 8,
  SR_Image = 9,
  SR_Evf = 10,
  SR_PlayareaResult = 11,
  SR_ImageSetting = 12,
  SR_BlobConfig = 13,
  SR_IrCamSetting = 14,
  SR_ContConfig = 15,
  SR_ArmModel = 16,
  SR_ContLedInfo = 17,
  SR_BluetoothQualityInfo = 18,
  SR_FwInfo1 = 19,
  SR_FwInfo2 = 20,
  SR_VrDialog = 21,
  SR_Application = 22,
  SR_VrTraceData = 23,
  SR_DebugData = 24,
  SR_LibpadAccess = 25,
  SR_LibpadRequestSteamVRPlugin = 26,
  SR_LibpadRequestAssistantApp = 27,
  SR_GeneralConfig = 28,
  SR_Log = 29,
  SR_TelemetryDevInfo = 30,
  SR_TelemetryTrackingInfo = 31,
  SR_TelemetryTrackingPcInfo = 32,
  SR_VrAppSceneInfo = 33,
  SR_InitialSetupInfo = 34,
  SR_PlayareaSetupInfo = 35,
  SR_Max = 36
};

enum ShareConfigIndex {
  SC_LastRecordedDateTime = 0,
  SC_VibrationStrength = 1,
  SC_ScreenBrightness = 2,
  SC_LimitDisplay = 3,
  SC_MemorySize = 4,
  SC_Done = 5,
  SC_State = 6,
  SC_CrashCount = 7,
  SC_OutputLog = 8,
  SC_BluetoothNotification = 9,
  SC_Status = 10,
  SC_Max = 11
};

struct GlobalEventContext {
  HANDLE *hMutex;
  HANDLE *hEvent;
  uint64_t *pSharedFlags;

  std::vector<std::pair<std::function<void()>, uint64_t>> callbacks;

  char exitFlag;
  void *threadInfo; // Offset 0x38

  IIpcMutex *ipcMutex;
  IIpcEvent *ipcEvent;
};

class ShareManager {
public:
  ShareManager();
  ~ShareManager();

  static ShareManager *GetInstance();
  static void InitializeInstance(DWORD processInstanceId);

  static void InstallHooks();
  static void ShutdownInstance();

  void Initialize(this ShareManager &self, DWORD processInstanceId);
  void RegisterEventCallback(this ShareManager &self, uint64_t mask, std::function<void()> *pCallback);
  static void WaitDynamicEvent(GlobalEventContext **evtStruct);
  void GetIntConfig(this ShareManager &self, int configId, long *outValue);
  uint32_t ReadStringConfig(this ShareManager &self, int configId, std::string &outStr);
  void WriteConfigString(this ShareManager &self, int configId, const std::string &str);
  uint32_t ReadStringConfig_Hook(this ShareManager &self, int configId, void *outStrObj);
  void WriteConfigString_Hook(this ShareManager &self, int configId, const void *strObj);
  int ReadLogStrings(this ShareManager &self, long long destAddress, int maxCount);
  void WriteLogString(this ShareManager &self, char level, const char *format, ...);
  void ReadIntConfigSafe(this ShareManager &self, int configId, int *outValue);
  uint64_t UpdateProcessExitCodes(this ShareManager &self, void *outData);

  uint32_t ReadCommon_0x10(this ShareManager &self, uint64_t *outData);
  int WriteCommon_0x10(this ShareManager &self, uint64_t data);
  int ReadCommon_0x18(this ShareManager &self);
  uint32_t ReadCommon_0x9dd0(this ShareManager &self, uint64_t *outData);
  int WriteCommon_0x9dd0(this ShareManager &self, uint64_t data);
  uint32_t ReadCommon_0xc72c(this ShareManager &self, uint8_t *outData);
  int WriteCommon_0xc72c(this ShareManager &self, uint8_t data);
  uint32_t ReadPlayareaSetup_0xc814(this ShareManager &self, void *outData);
  int WritePlayareaSetup_0xc814(this ShareManager &self, uint32_t data);

  uint32_t ReadArmModel_0x84a4(this ShareManager &self, void *outData, int param);
  int WriteArmModel_0x84a4(this ShareManager &self, void *data, int param);
  uint32_t ReadBtQualityInfo_0x9170(this ShareManager &self, void *outData, int param);
  int WriteBtQualityInfo_0x9170(this ShareManager &self, void *data, int param);

  uint32_t AcquireImageWriteSlot(this ShareManager &self, long long *outImageBuffer, long long *outTrackingData);
  uint32_t AcquireInputWriteSlot(this ShareManager &self, int groupIdx, long long *outOffset);
  uint32_t AcquirePoseWriteSlot(this ShareManager &self, int groupIdx, long long *outOffset);

  void CommitImageWriteSlot(this ShareManager &self, uint32_t slotIdx);
  void CommitInputWriteSlot(this ShareManager &self, int groupIdx, uint32_t slotIdx);
  void CommitPoseWriteSlot(this ShareManager &self, int groupIdx, uint32_t slotIdx, void *params);

  uint32_t AcquireImageReadSlot(this ShareManager &self, long long *outImageBuffer, void *outTrackingData, void *outUnknown);
  uint32_t AcquireInputReadSlot(this ShareManager &self, int groupIdx, long long *outOffset);
  uint32_t AcquirePoseReadSlot(this ShareManager &self, int groupIdx, long long *outOffset, void *outParams);

  void CommitImageReadSlot(this ShareManager &self, uint32_t slotIdx);
  void CommitInputReadSlot(this ShareManager &self, int groupIdx, uint32_t slotIdx);
  void CommitPoseReadSlot(this ShareManager &self, int groupIdx, uint32_t slotIdx);

  uint64_t TryAcquireShareMutex(this ShareManager &self, uint32_t typeIndex);
  uint32_t AcquireShareMutex(this ShareManager &self, uint32_t typeIndex);
  uint32_t ReleaseShareMutex(this ShareManager &self, uint32_t typeIndex);
  uint32_t TryAcquireLibpadMutex(this ShareManager &self);
  int ReleaseLibpadMutex(this ShareManager &self);
  void ReleaseMutexByIndex(this ShareManager &self, int index);
  void ClearLogEventFlag(this ShareManager &self);
  void SetGlobalEventFlag(this ShareManager &self, uint64_t flags);

  uint32_t ReadPlayareaResult(this ShareManager &self, void *outData);
  uint32_t WritePlayareaResult(this ShareManager &self, void *data);
  int WriteFwInfo2_0x9280(this ShareManager &self, void *data);

  uint32_t WaitShareEvent(this ShareManager &self, int groupIdx, DWORD timeoutMs);

  // Group 1: Status
  DEF_READ_MEMCPY_RET(ReadStatus_0x2c)
  DEF_WRITE_MEMCPY(WriteStatus_0x2c)

  // Group 2: Calibration Blocks
  DEF_READ_MEMCPY_OUT(ReadCalib_0x70)
  DEF_WRITE_MEMCPY(WriteCalib_0x70)
  DEF_READ_MEMCPY_OUT(ReadCalib_0x170)
  DEF_WRITE_MEMCPY(WriteCalib_0x170)
  DEF_READ_MEMCPY_OUT(ReadCalib_0x370)
  DEF_WRITE_MEMCPY(WriteCalib_0x370)
  DEF_READ_MEMCPY_OUT(ReadCalib_0x3f0)
  DEF_WRITE_MEMCPY(WriteCalib_0x3f0)
  DEF_READ_MEMCPY_OUT(ReadCalib_0x41c)
  DEF_WRITE_MEMCPY(WriteCalib_0x41c)
  DEF_READ_MEMCPY_OUT(ReadCalib_0x50c)
  DEF_WRITE_MEMCPY(WriteCalib_0x50c)
  DEF_READ_MEMCPY_OUT(ReadCalib_0xd0c)
  DEF_WRITE_MEMCPY(WriteCalib_0xd0c)

  // Groups 12-19: Configs & Info
  DEF_READ_MEMCPY_OUT(ReadImageSetting_0x803c)
  DEF_WRITE_MEMCPY(WriteImageSetting_0x803c)
  DEF_READ_MEMCPY_OUT(ReadBlobConfig_0x8058)
  DEF_WRITE_MEMCPY(WriteBlobConfig_0x8058)
  DEF_READ_MEMCPY_OUT(ReadIrCamSetting_0x807c)
  DEF_WRITE_MEMCPY(WriteIrCamSetting_0x807c)
  DEF_READ_MEMCPY_RET(ReadContConfig_0x8098)
  DEF_WRITE_MEMCPY(WriteContConfig_0x8098)
  DEF_READ_MEMCPY_RET(ReadContLedInfo_0x855c)
  DEF_WRITE_MEMCPY(WriteContLedInfo_0x855c)
  DEF_READ_MEMCPY_RET(ReadFwInfo1_0x922c)
  DEF_WRITE_MEMCPY(WriteFwInfo1_0x922c)

  // Groups 20-24: Application & Debug
  DEF_READ_MEMCPY_RET(ReadFwInfo2_0x9280)
  DEF_READ_MEMCPY_RET(ReadVrDialog_0x9ae0)
  DEF_WRITE_MEMCPY(WriteVrDialog_0x9ae0)
  DEF_READ_MEMCPY_RET(ReadApplication_0x9b3c)
  DEF_READ_MEMCPY_RET(ReadVrTraceData_0x9b7c)
  DEF_WRITE_MEMCPY(WriteVrTraceData_0x9b7c)
  DEF_READ_MEMCPY_RET(ReadDebugData_0x9c8c)
  DEF_WRITE_MEMCPY(WriteDebugData_0x9c8c)

  // Groups 30-34: Telemetry & Setup
  DEF_READ_MEMCPY_RET(ReadSceneInfo_0x9de4)
  DEF_WRITE_MEMCPY(WriteSceneInfo_0x9de4)
  DEF_READ_MEMCPY_RET(ReadTelDevInfo_0xa984)
  DEF_WRITE_MEMCPY(WriteTelDevInfo_0xa984)
  DEF_READ_MEMCPY_RET(ReadTelTrkInfo_0xaaa0)
  DEF_WRITE_MEMCPY(WriteTelTrkInfo_0xaaa0)
  DEF_READ_MEMCPY_RET(ReadTelPcInfo_0xc600)
  DEF_WRITE_MEMCPY(WriteTelPcInfo_0xc600)
  DEF_READ_MEMCPY_RET(ReadInitSetup_0xc73c)
  DEF_WRITE_MEMCPY(WriteInitSetup_0xc73c)

private:
  uint8_t m_pad_offset0[8]; // Offset 0x0
  HANDLE
  *m_hConfigMutexes[16];               // Offset 0x8, size 128 bytes (0x80), ends at 0x88
  uint8_t m_pad_offset88[0x2108];      // Pad from 0x88 to 0x2190
  HANDLE *m_hEvents[SR_Max];           // Offset 0x2190, size 288 bytes (0x120), ends at
                                       // 0x22b0
  HANDLE *m_hMutexes[SR_Max];          // Offset 0x22b0, size 288 bytes (0x120), ends at
                                       // 0x23d0
  GlobalEventContext *m_pEventContext; // Offset 0x23d0, size 8 bytes, ends at 0x23d8
  HANDLE m_hSharedFileMapping;         // Offset 0x23d8, size 8 bytes, ends at 0x23e0
  VRSharedMemory *m_pMem;              // Offset 0x23e0, size 8 bytes, ends at 0x23e8
  DWORD m_instanceId;                  // Offset 0x23e8, size 4 bytes, ends at 0x23ec
  uint32_t m_inputSequences[3] = {0};  // Offset 0x23ec, size 12 bytes, ends at 0x23f8
  uint32_t m_poseSequences[3] = {0};   // Offset 0x23f8, size 12 bytes, ends at 0x2404
  uint32_t m_sequence[1] = {0};        // Offset 0x2404, size 4 bytes, ends at 0x2408
  uint32_t m_writeIndex_40[3] = {0};   // Offset 0x2408, size 12 bytes, ends at 0x2414
  uint32_t m_writeIndex_28[3] = {0};   // Offset 0x2414, size 12 bytes, ends at 0x2420
  uint32_t m_writeIndex[1] = {0};      // Offset 0x2420, size 4 bytes, ends at 0x2424

  // The following members are not where the should be for ShareManager, or they
  // don't exist.

  bool m_exitThreads;

  // Configuration mapping definition for user config.ini
  struct ConfigMapping {
    const char *AppName;
    const char *KeyName;
    const char *DefaultValue;
    char CachedValue[256]; // Used by EVF worker to detect changes
  };

  ConfigMapping m_configMappings[SC_Max];
  char m_configIniPath[MAX_PATH];

  static ShareManager *s_instance;
  static bool s_isInitialized;

  static unsigned __stdcall WorkerThread_Unconditional(void *pContext);
  static unsigned __stdcall EvfWorkerThread_Conditional(void *pContext);
  static unsigned __stdcall CameraMonitorThread(void *pContext);
  void InitializeSub_ad30(this ShareManager &self);
  void InitializeSub_bdb0(this ShareManager &self);

  // IPC helper mutex locks (we'll use CrossIPC here later)
  void Lock(int idx);
  void Unlock(int idx);

  IIpcMutex *m_ipcConfigMutexes[16];
  IIpcEvent *m_ipcEvents[SR_Max];
  IIpcMutex *m_ipcMutexes[SR_Max];
  IIpcSharedMemory *m_ipcSharedMemory;

  ProcessOwnedMutex m_libpadMutex;
  ProcessOwnedMutex m_shareMutexes[2];
};
