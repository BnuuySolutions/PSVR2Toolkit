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
      uint8_t pad[0x8];
      uint32_t counter_0x8;
      uint32_t pad_0xC;
      uint64_t data_0x10;
    } common_0x10;

    struct {
      uint8_t pad[0x18];
      uint16_t data_0x18;
    } common_0x18;

    struct {
      uint8_t pad[0x9dc8];
      uint32_t counter_0x9dc8;
      uint32_t pad_0x9dcc;
      uint64_t data_0x9dd0;
    } common_9dd0;

    struct {
      uint8_t pad[0xc728];
      uint32_t ctr;
      uint8_t data_0xc72c;
    } common_c72c;

    struct {
      uint8_t pad[0x28];
      uint32_t ctr;
      uint32_t data_0x2c[9];
    } status_0x2c;

    struct {
      uint8_t pad[0x54];
      uint32_t ctr;
      uint8_t pad_0x58[0x18];
      uint8_t data_0x70[0x100];
    } calib_0x70;

    struct {
      uint8_t pad[0x58];
      uint32_t ctr;
      uint8_t pad_0x5c[0x114];
      uint8_t data_0x170[0x200];
    } calib_170;

    struct {
      uint8_t pad[0x5c];
      uint32_t ctr;
      uint8_t pad_0x60[0x310];
      uint8_t data_0x370[120];
    } calib_370;

    struct {
      uint8_t pad[0x60];
      uint32_t ctr;
      uint8_t pad_0x64[0x384];
      uint8_t data_0x3f0[52];
    } calib_3f0;

    struct {
      uint8_t pad[0x64];
      uint32_t ctr;
      uint8_t pad_0x68[0x3B4];
      uint8_t data_0x41c[0xF0];
    } calib_41c;

    struct {
      uint8_t pad[0x68];
      uint32_t ctr;
      uint8_t pad_0x6c[0x4a0];
      uint8_t data_0x50c[0x800];
    } calib_50c;

    struct {
      uint8_t pad[0x6c];
      uint32_t ctr;
      uint8_t pad_0x70[0xc9c];
      uint8_t data_0xd0c[0x800];
    } calib_d0c;

    struct {
      uint8_t pad[0x152c];
      uint32_t counters[0x30];
    } tracking_40_meta;

    struct {
      uint8_t pad[0x150c];
      InputGroupMeta groups[3];
    } input_meta;

    struct {
      uint8_t pad[0x1e10];
      PoseGroupMeta groups[3];
    } pose_meta;

    struct {
      uint8_t pad[0x3c10];
      ImageSlotMeta slots[8];
    } image_meta;

    struct {
      uint8_t pad[0x100000];
      uint8_t slots[3][64][64];
    } input_data;

    struct {
      uint8_t pad[0x103000];
      PoseSlotData slots[4][64];
    } pose_data;

    struct {
      uint8_t pad[0x10ba00];
      ImageSlotData slots[8];
    } image_data;

    struct {
      uint8_t pad[0x7fd0];
      PlayareaSlotState playarea_states[8];
    } playarea_meta;

    struct {
      uint8_t pad[0x110c208];
      uint8_t playarea_slots[8][64];
    } playarea_ring;

    struct {
      uint8_t pad[0x8038];
      uint32_t ctr;
      uint32_t data[4];
    } img_setting_803c;
    struct {
      uint8_t pad[0x8054];
      uint32_t ctr;
      uint32_t data[6];
    } blob_cfg_8058;
    struct {
      uint8_t pad[0x8078];
      uint32_t ctr;
      uint32_t data[4];
    } ir_cam_807c;
    struct {
      uint8_t pad[0x8094];
      uint32_t ctr;
      uint8_t data[0x400];
    } cont_cfg_8098;
    struct {
      uint8_t pad[0x84a0];
      struct ArmModelSlot {
        uint32_t ctr;
        uint8_t data[80];
        uint8_t pad[8];
      } slots[2];
    } arm_model_84a4;
    struct {
      uint8_t pad[0x8558];
      uint32_t ctr;
      uint8_t data[0xC00];
    } cont_led_855c;
    struct {
      uint8_t pad[0x9168];
      struct BtQualitySlot {
        uint32_t ctr;
        uint32_t pad;
        uint8_t data[80];
        uint8_t unused_pad[8];
      } slots[2];
    } bt_qual_9170;
    struct {
      uint8_t pad[0x9228];
      uint32_t ctr;
      uint8_t data[0x48];
    } fw_info1_922c;
    struct {
      uint8_t pad[0x927c];
      uint32_t ctr;
      uint8_t data[0x850];
    } fw_info2_9280;
    struct {
      uint8_t pad[0x9ad8];
      uint32_t ctr;
      uint32_t pad2;
      uint8_t data[0x50];
    } vr_dialog_9ae0;
    struct {
      uint8_t pad[0x9b38];
      uint32_t ctr;
      uint8_t data[49];
    } app_9b3c;
    struct {
      uint8_t pad[0x9b78];
      uint32_t ctr;
      uint8_t data[0x80];
    } vr_trace_9b7c;
    struct {
      uint8_t pad[0x9c88];
      uint32_t ctr;
      uint8_t data[0x131];
    } dbg_data_9c8c;
    struct {
      uint8_t pad[0x9de0];
      uint32_t ctr;
      uint32_t data[4];
    } scene_info_9de4;
    struct {
      uint8_t pad[0xa980];
      uint32_t ctr;
      uint8_t data[0x90];
    } tel_dev_a984;
    struct {
      uint8_t pad[0xaa9c];
      uint32_t ctr;
      uint8_t data[0x1B50];
    } tel_trk_aaa0;
    struct {
      uint8_t pad[0xc5f8];
      uint32_t ctr;
      uint8_t data[0xA0];
    } tel_pc_c600;
    struct {
      uint8_t pad[0xc738];
      uint32_t ctr;
      uint8_t data[200];
    } init_setup_c73c;
    struct {
      uint8_t pad[0xc810];
      uint32_t ctr;
      uint32_t data;
    } playarea_setup_c814;

    struct {
      uint8_t pad[0x9dfc];
      struct ConfigSlot {
        uint32_t counter;
        char stringData[264];
      } str_configs[16];
    } configs_9e00;

    struct {
      uint8_t pad[0x110c408];
      uint8_t log_head;
      uint8_t log_tail;
      uint8_t pad2[0xFE];
      LogMetadata log_meta[256];
      uint8_t pad3[0x3fb00];
      char log_strings[256][0x100];
    } logs;

    struct {
      uint8_t pad[0xffc00];
      double timestamps[11];
      DWORD pids[11];
      uint32_t states[11];
    } watchdog;
  };
};

#pragma pack(pop)

#define DEF_READ_MEMCPY_RET(FuncName, GroupIdx, MemGroup, MemData, Size) uint32_t FuncName(this ShareManager &self, void *outData);

#define DEF_READ_MEMCPY_OUT(FuncName, GroupIdx, MemGroup, MemData, Size) uint32_t FuncName(this ShareManager &self, void *outData, uint32_t *outCounter);

#define DEF_WRITE_MEMCPY(FuncName, GroupIdx, MemGroup, MemData, Size) int FuncName(this ShareManager &self, void *data);

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
  DEF_READ_MEMCPY_RET(ReadStatus_0x2c, 1, status_0x2c, data_0x2c, sizeof(self.m_pMem->status_0x2c.data_0x2c))
  DEF_WRITE_MEMCPY(WriteStatus_0x2c, 1, status_0x2c, data_0x2c, sizeof(self.m_pMem->status_0x2c.data_0x2c))

  // Group 2: Calibration Blocks
  DEF_READ_MEMCPY_OUT(ReadCalib_0x70, 2, calib_0x70, data_0x70, sizeof(self.m_pMem->calib_0x70.data_0x70))
  DEF_WRITE_MEMCPY(WriteCalib_0x70, 2, calib_0x70, data_0x70, sizeof(self.m_pMem->calib_0x70.data_0x70))
  DEF_READ_MEMCPY_OUT(ReadCalib_0x170, 2, calib_170, data_0x170, sizeof(self.m_pMem->calib_170.data_0x170))
  DEF_WRITE_MEMCPY(WriteCalib_0x170, 2, calib_170, data_0x170, sizeof(self.m_pMem->calib_170.data_0x170))
  DEF_READ_MEMCPY_OUT(ReadCalib_0x370, 2, calib_370, data_0x370, sizeof(self.m_pMem->calib_370.data_0x370))
  DEF_WRITE_MEMCPY(WriteCalib_0x370, 2, calib_370, data_0x370, sizeof(self.m_pMem->calib_370.data_0x370))
  DEF_READ_MEMCPY_OUT(ReadCalib_0x3f0, 2, calib_3f0, data_0x3f0, sizeof(self.m_pMem->calib_3f0.data_0x3f0))
  DEF_WRITE_MEMCPY(WriteCalib_0x3f0, 2, calib_3f0, data_0x3f0, sizeof(self.m_pMem->calib_3f0.data_0x3f0))
  DEF_READ_MEMCPY_OUT(ReadCalib_0x41c, 2, calib_41c, data_0x41c, sizeof(self.m_pMem->calib_41c.data_0x41c))
  DEF_WRITE_MEMCPY(WriteCalib_0x41c, 2, calib_41c, data_0x41c, sizeof(self.m_pMem->calib_41c.data_0x41c))
  DEF_READ_MEMCPY_OUT(ReadCalib_0x50c, 2, calib_50c, data_0x50c, sizeof(self.m_pMem->calib_50c.data_0x50c))
  DEF_WRITE_MEMCPY(WriteCalib_0x50c, 2, calib_50c, data_0x50c, sizeof(self.m_pMem->calib_50c.data_0x50c))
  DEF_READ_MEMCPY_OUT(ReadCalib_0xd0c, 2, calib_d0c, data_0xd0c, sizeof(self.m_pMem->calib_d0c.data_0xd0c))
  DEF_WRITE_MEMCPY(WriteCalib_0xd0c, 2, calib_d0c, data_0xd0c, sizeof(self.m_pMem->calib_d0c.data_0xd0c))

  // Groups 12-19: Configs & Info
  DEF_READ_MEMCPY_OUT(ReadImageSetting_0x803c, 12, img_setting_803c, data, sizeof(self.m_pMem->img_setting_803c.data))
  DEF_WRITE_MEMCPY(WriteImageSetting_0x803c, 12, img_setting_803c, data, sizeof(self.m_pMem->img_setting_803c.data))
  DEF_READ_MEMCPY_OUT(ReadBlobConfig_0x8058, 13, blob_cfg_8058, data, sizeof(self.m_pMem->blob_cfg_8058.data))
  DEF_WRITE_MEMCPY(WriteBlobConfig_0x8058, 13, blob_cfg_8058, data, sizeof(self.m_pMem->blob_cfg_8058.data))
  DEF_READ_MEMCPY_OUT(ReadIrCamSetting_0x807c, 14, ir_cam_807c, data, sizeof(self.m_pMem->ir_cam_807c.data))
  DEF_WRITE_MEMCPY(WriteIrCamSetting_0x807c, 14, ir_cam_807c, data, sizeof(self.m_pMem->ir_cam_807c.data))
  DEF_READ_MEMCPY_RET(ReadContConfig_0x8098, 15, cont_cfg_8098, data, sizeof(self.m_pMem->cont_cfg_8098.data))
  DEF_WRITE_MEMCPY(WriteContConfig_0x8098, 15, cont_cfg_8098, data, sizeof(self.m_pMem->cont_cfg_8098.data))
  DEF_READ_MEMCPY_RET(ReadContLedInfo_0x855c, 17, cont_led_855c, data, sizeof(self.m_pMem->cont_led_855c.data))
  DEF_WRITE_MEMCPY(WriteContLedInfo_0x855c, 17, cont_led_855c, data, sizeof(self.m_pMem->cont_led_855c.data))
  DEF_READ_MEMCPY_RET(ReadFwInfo1_0x922c, 19, fw_info1_922c, data, sizeof(self.m_pMem->fw_info1_922c.data))
  DEF_WRITE_MEMCPY(WriteFwInfo1_0x922c, 19, fw_info1_922c, data, sizeof(self.m_pMem->fw_info1_922c.data))

  // Groups 20-24: Application & Debug
  DEF_READ_MEMCPY_RET(ReadFwInfo2_0x9280, 20, fw_info2_9280, data, sizeof(self.m_pMem->fw_info2_9280.data))
  DEF_READ_MEMCPY_RET(ReadVrDialog_0x9ae0, 21, vr_dialog_9ae0, data, sizeof(self.m_pMem->vr_dialog_9ae0.data))
  DEF_WRITE_MEMCPY(WriteVrDialog_0x9ae0, 21, vr_dialog_9ae0, data, sizeof(self.m_pMem->vr_dialog_9ae0.data))
  DEF_READ_MEMCPY_RET(ReadApplication_0x9b3c, 22, app_9b3c, data, sizeof(self.m_pMem->app_9b3c.data))
  DEF_READ_MEMCPY_RET(ReadVrTraceData_0x9b7c, 23, vr_trace_9b7c, data, sizeof(self.m_pMem->vr_trace_9b7c.data))
  DEF_WRITE_MEMCPY(WriteVrTraceData_0x9b7c, 23, vr_trace_9b7c, data, sizeof(self.m_pMem->vr_trace_9b7c.data))
  DEF_READ_MEMCPY_RET(ReadDebugData_0x9c8c, 24, dbg_data_9c8c, data, sizeof(self.m_pMem->dbg_data_9c8c.data))
  DEF_WRITE_MEMCPY(WriteDebugData_0x9c8c, 24, dbg_data_9c8c, data, sizeof(self.m_pMem->dbg_data_9c8c.data))

  // Groups 30-34: Telemetry & Setup
  DEF_READ_MEMCPY_RET(ReadSceneInfo_0x9de4, 33, scene_info_9de4, data, sizeof(self.m_pMem->scene_info_9de4.data))
  DEF_WRITE_MEMCPY(WriteSceneInfo_0x9de4, 33, scene_info_9de4, data, sizeof(self.m_pMem->scene_info_9de4.data))
  DEF_READ_MEMCPY_RET(ReadTelDevInfo_0xa984, 30, tel_dev_a984, data, sizeof(self.m_pMem->tel_dev_a984.data))
  DEF_WRITE_MEMCPY(WriteTelDevInfo_0xa984, 30, tel_dev_a984, data, sizeof(self.m_pMem->tel_dev_a984.data))
  DEF_READ_MEMCPY_RET(ReadTelTrkInfo_0xaaa0, 31, tel_trk_aaa0, data, sizeof(self.m_pMem->tel_trk_aaa0.data))
  DEF_WRITE_MEMCPY(WriteTelTrkInfo_0xaaa0, 31, tel_trk_aaa0, data, sizeof(self.m_pMem->tel_trk_aaa0.data))
  DEF_READ_MEMCPY_RET(ReadTelPcInfo_0xc600, 32, tel_pc_c600, data, sizeof(self.m_pMem->tel_pc_c600.data))
  DEF_WRITE_MEMCPY(WriteTelPcInfo_0xc600, 32, tel_pc_c600, data, sizeof(self.m_pMem->tel_pc_c600.data))
  DEF_READ_MEMCPY_RET(ReadInitSetup_0xc73c, 34, init_setup_c73c, data, sizeof(self.m_pMem->init_setup_c73c.data))
  DEF_WRITE_MEMCPY(WriteInitSetup_0xc73c, 34, init_setup_c73c, data, sizeof(self.m_pMem->init_setup_c73c.data))

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
