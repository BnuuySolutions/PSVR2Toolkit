#include "../hmd_driver_loader.h"
#include "share_manager.h"
#include "hook_lib.h"
#include "util.h"

using namespace psvr2_toolkit;

#include <cstdlib>
#include <process.h>
#include <cstdarg>
#include <shlobj.h>

static const char* SharedResourceNames[][2] = {
    {"SHARE_VRT2_WIN_COMMON_EVT", "SHARE_VRT2_WIN_COMMON_MTX"},
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
    {"SHARE_VRT2_WIN_PLAYAREA_SETUP_INFO_EVT", "SHARE_VRT2_WIN_PLAYAREA_SETUP_INFO_MTX"}
};

ShareManager* ShareManager::s_instance = nullptr;
bool ShareManager::s_isInitialized = false;

ShareManager::ShareManager() : m_hSharedFileMapping(nullptr), m_pMem(nullptr), m_instanceId(0) {
    memset(m_hConfigMutexes, 0, sizeof(m_hConfigMutexes));
    memset(m_hEvents, 0, sizeof(m_hEvents));
    memset(m_hMutexes, 0, sizeof(m_hMutexes));

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
    for (int i = 0; i < 36; ++i) {
        if (m_hEvents[i]) {
            CloseHandle(*m_hEvents[i]);
            delete m_hEvents[i];
            m_hEvents[i] = nullptr;
        }
        if (m_hMutexes[i]) {
            CloseHandle(*m_hMutexes[i]);
            delete m_hMutexes[i];
            m_hMutexes[i] = nullptr;
        }
    }
    for (int i = 0; i < 16; ++i) {
        if (m_hConfigMutexes[i]) {
            if (i == 11) {
                CloseHandle(*reinterpret_cast<HANDLE*>(m_hConfigMutexes[11]));
                operator delete(m_hConfigMutexes[11]);
            } else {
                CloseHandle(*m_hConfigMutexes[i]);
                delete m_hConfigMutexes[i];
            }
            m_hConfigMutexes[i] = nullptr;
        }
    }
}

ShareManager* ShareManager::GetInstance() {
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

void ShareManager::ReplaceShareManager() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();
    uint64_t baseAddress = pHmdDriverLoader->GetBaseAddress();

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15bbd0), reinterpret_cast<void*>(&ShareManager::GetInstance));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15bcf0), reinterpret_cast<void*>(&ShareManager::InitializeInstance));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x158600), reinterpret_cast<void*>(&ShareManager::Initialize));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e0b0), reinterpret_cast<void*>(&ShareManager::RegisterEventCallback));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x157f70), reinterpret_cast<void*>(&ShareManager::WaitDynamicEvent));
    
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15d270), reinterpret_cast<void*>(&ShareManager::GetIntConfig));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15d3d0), reinterpret_cast<void*>(&ShareManager::ReadStringConfig_Hook));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15f470), reinterpret_cast<void*>(&ShareManager::WriteConfigString_Hook));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15d730), reinterpret_cast<void*>(&ShareManager::ReadLogStrings));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15f8c0), reinterpret_cast<void*>(&ShareManager::WriteLogString));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15dda0), reinterpret_cast<void*>(&ShareManager::UpdateProcessExitCodes));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15b950), reinterpret_cast<void*>(&ShareManager::ReadIntConfigSafe));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15cf20), reinterpret_cast<void*>(&ShareManager::ReadCommon_0x10));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15ebe0), reinterpret_cast<void*>(&ShareManager::WriteCommon_0x10));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15dae0), reinterpret_cast<void*>(&ShareManager::ReadCommon_0x18));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15c7e0), reinterpret_cast<void*>(&ShareManager::ReadCommon_0x9dd0));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e520), reinterpret_cast<void*>(&ShareManager::WriteCommon_0x9dd0));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15c840), reinterpret_cast<void*>(&ShareManager::ReadCommon_0xc72c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e580), reinterpret_cast<void*>(&ShareManager::WriteCommon_0xc72c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15da80), reinterpret_cast<void*>(&ShareManager::ReadPlayareaSetup_0xc814));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15fba0), reinterpret_cast<void*>(&ShareManager::WritePlayareaSetup_0xc814));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15c920), reinterpret_cast<void*>(&ShareManager::ReadArmModel_0x84a4));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e670), reinterpret_cast<void*>(&ShareManager::WriteArmModel_0x84a4));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15ca50), reinterpret_cast<void*>(&ShareManager::ReadBtQualityInfo_0x9170));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e7a0), reinterpret_cast<void*>(&ShareManager::WriteBtQualityInfo_0x9170));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15a070), reinterpret_cast<void*>(&ShareManager::AcquireImageWriteSlot));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15a160), reinterpret_cast<void*>(&ShareManager::AcquireInputWriteSlot));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15a250), reinterpret_cast<void*>(&ShareManager::AcquirePoseWriteSlot));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15b6a0), reinterpret_cast<void*>(&ShareManager::CommitImageWriteSlot));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15b780), reinterpret_cast<void*>(&ShareManager::CommitInputWriteSlot));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15b830), reinterpret_cast<void*>(&ShareManager::CommitPoseWriteSlot));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x159970), reinterpret_cast<void*>(&ShareManager::AcquireImageReadSlot));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x159b60), reinterpret_cast<void*>(&ShareManager::AcquireInputReadSlot));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x159dc0), reinterpret_cast<void*>(&ShareManager::AcquirePoseReadSlot));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15b4d0), reinterpret_cast<void*>(&ShareManager::CommitImageReadSlot));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15b550), reinterpret_cast<void*>(&ShareManager::CommitInputReadSlot));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15b600), reinterpret_cast<void*>(&ShareManager::CommitPoseReadSlot));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15c610), reinterpret_cast<void*>(&ShareManager::TryAcquireShareMutex));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e260), reinterpret_cast<void*>(&ShareManager::AcquireShareMutex));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e4f0), reinterpret_cast<void*>(&ShareManager::ReleaseShareMutex));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x159940), reinterpret_cast<void*>(&ShareManager::TryAcquireLibpadMutex));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e230), reinterpret_cast<void*>(&ShareManager::ReleaseLibpadMutex));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e480), reinterpret_cast<void*>(&ShareManager::ReleaseMutexByIndex));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e2a0), reinterpret_cast<void*>(&ShareManager::ClearLogEventFlag));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e2f0), reinterpret_cast<void*>(&ShareManager::SetGlobalEventFlag));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15d8d0), reinterpret_cast<void*>(&ShareManager::ReadPlayareaResult));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15fa80), reinterpret_cast<void*>(&ShareManager::WritePlayareaResult));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15f9d0), reinterpret_cast<void*>(&ShareManager::WriteFwInfo2_0x9280));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e4a0), reinterpret_cast<void*>(&ShareManager::WaitShareEvent));

    // Group 1: Status
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15db30), reinterpret_cast<void*>(&ShareManager::ReadStatus_0x2c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15fc00), reinterpret_cast<void*>(&ShareManager::WriteStatus_0x2c));

    // Group 2: Calibration Blocks
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15cc60), reinterpret_cast<void*>(&ShareManager::ReadCalib_0x70));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e990), reinterpret_cast<void*>(&ShareManager::WriteCalib_0x70));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15cd30), reinterpret_cast<void*>(&ShareManager::ReadCalib_0x170));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15ea40), reinterpret_cast<void*>(&ShareManager::WriteCalib_0x170));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15cb00), reinterpret_cast<void*>(&ShareManager::ReadCalib_0x370));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e850), reinterpret_cast<void*>(&ShareManager::WriteCalib_0x370));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15cbc0), reinterpret_cast<void*>(&ShareManager::ReadCalib_0x3f0));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e910), reinterpret_cast<void*>(&ShareManager::WriteCalib_0x3f0));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15d460), reinterpret_cast<void*>(&ShareManager::ReadCalib_0x41c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15f610), reinterpret_cast<void*>(&ShareManager::WriteCalib_0x41c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15ce00), reinterpret_cast<void*>(&ShareManager::ReadCalib_0x50c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15eb00), reinterpret_cast<void*>(&ShareManager::WriteCalib_0x50c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15ce90), reinterpret_cast<void*>(&ShareManager::ReadCalib_0xd0c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15eb70), reinterpret_cast<void*>(&ShareManager::WriteCalib_0xd0c));

    // Groups 12-19: Configs & Info
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15d550), reinterpret_cast<void*>(&ShareManager::ReadImageSetting_0x803c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15f6f0), reinterpret_cast<void*>(&ShareManager::WriteImageSetting_0x803c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15c9d0), reinterpret_cast<void*>(&ShareManager::ReadBlobConfig_0x8058));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15e720), reinterpret_cast<void*>(&ShareManager::WriteBlobConfig_0x8058));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15d6b0), reinterpret_cast<void*>(&ShareManager::ReadIrCamSetting_0x807c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15f850), reinterpret_cast<void*>(&ShareManager::WriteIrCamSetting_0x807c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15cfd0), reinterpret_cast<void*>(&ShareManager::ReadContConfig_0x8098));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15f110), reinterpret_cast<void*>(&ShareManager::WriteContConfig_0x8098));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15d090), reinterpret_cast<void*>(&ShareManager::ReadContLedInfo_0x855c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15f1d0), reinterpret_cast<void*>(&ShareManager::WriteContLedInfo_0x855c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15d1e0), reinterpret_cast<void*>(&ShareManager::ReadFwInfo1_0x922c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15f330), reinterpret_cast<void*>(&ShareManager::WriteFwInfo1_0x922c));

    // Groups 20-24: Application & Debug
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15d860), reinterpret_cast<void*>(&ShareManager::ReadFwInfo2_0x9280));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15df60), reinterpret_cast<void*>(&ShareManager::ReadVrDialog_0x9ae0));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15ff00), reinterpret_cast<void*>(&ShareManager::WriteVrDialog_0x9ae0));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15c8a0), reinterpret_cast<void*>(&ShareManager::ReadApplication_0x9b3c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15dff0), reinterpret_cast<void*>(&ShareManager::ReadVrTraceData_0x9b7c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15ffa0), reinterpret_cast<void*>(&ShareManager::WriteVrTraceData_0x9b7c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15d100), reinterpret_cast<void*>(&ShareManager::ReadDebugData_0x9c8c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15f250), reinterpret_cast<void*>(&ShareManager::WriteDebugData_0x9c8c));

    // Groups 30-34: Telemetry & Setup
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15df00), reinterpret_cast<void*>(&ShareManager::ReadSceneInfo_0x9de4));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15fe90), reinterpret_cast<void*>(&ShareManager::WriteSceneInfo_0x9de4));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15dba0), reinterpret_cast<void*>(&ShareManager::ReadTelDevInfo_0xa984));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15fc70), reinterpret_cast<void*>(&ShareManager::WriteTelDevInfo_0xa984));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15dc60), reinterpret_cast<void*>(&ShareManager::ReadTelTrkInfo_0xaaa0));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15fd40), reinterpret_cast<void*>(&ShareManager::WriteTelTrkInfo_0xaaa0));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15dcd0), reinterpret_cast<void*>(&ShareManager::ReadTelPcInfo_0xc600));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15fdc0), reinterpret_cast<void*>(&ShareManager::WriteTelPcInfo_0xc600));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15d5d0), reinterpret_cast<void*>(&ShareManager::ReadInitSetup_0xc73c));
    HookLib::InstallHook(reinterpret_cast<void*>(baseAddress + 0x15f760), reinterpret_cast<void*>(&ShareManager::WriteInitSetup_0xc73c));
}

void ShareManager::Initialize(this ShareManager& self, DWORD processInstanceId) {
    OutputDebugStringA("ShareManager::Initialize");

    self.m_instanceId = processInstanceId;

    for (int i = 0; i < 36; ++i) {
        self.m_hEvents[i] = new HANDLE(CreateEventA(nullptr, TRUE, FALSE, SharedResourceNames[i][0]));
        self.m_hMutexes[i] = new HANDLE(CreateMutexA(nullptr, FALSE, SharedResourceNames[i][1]));
    }

    self.m_hSharedFileMapping = CreateFileMappingA(
        INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, 0x2000000, "SHARE_VRT2_WIN"
    );

    if (self.m_hSharedFileMapping) {
        self.m_pMem = reinterpret_cast<VRSharedMemory*>(
            MapViewOfFile(self.m_hSharedFileMapping, FILE_MAP_ALL_ACCESS, 0, 0, 0x2000000)
        );
    }

    if (!self.m_pMem) {
        std::abort();
    }

    GlobalEventContext* pCtx = new GlobalEventContext();
    pCtx->hMutex = self.m_hMutexes[10];
    pCtx->hEvent = self.m_hEvents[10];
    pCtx->pSharedFlags = reinterpret_cast<uint64_t*>(reinterpret_cast<uint8_t*>(self.m_pMem) + 0x110C200);
    pCtx->exitFlag = '\0';

    void* threadInfo1 = operator new(0x10);
    if (threadInfo1) {
        void* threadContext1 = operator new(0x8);
        if (threadContext1) {
            *reinterpret_cast<GlobalEventContext**>(threadContext1) = pCtx;
        }
        unsigned int threadId;
        uintptr_t hThread = _beginthreadex(
            nullptr, 0, &ShareManager::WorkerThread_Unconditional, threadContext1, 0, &threadId
        );
        *reinterpret_cast<uintptr_t*>(threadInfo1) = hThread;
        *reinterpret_cast<unsigned int*>(reinterpret_cast<uint8_t*>(threadInfo1) + 8) = threadId;
        pCtx->threadInfo = threadInfo1;
    } else {
        pCtx->threadInfo = nullptr;
    }

    self.m_pEventContext = pCtx;

    uint8_t* memBase = reinterpret_cast<uint8_t*>(self.m_pMem);

    for (int group = 0; group < 12; group++) {
        for (int slot = 0; slot < 64; slot++) {
            uint32_t* pState = reinterpret_cast<uint32_t*>(memBase + 0x150c + ((group * 64) + slot) * 12);
            if (*pState == 2 || (self.m_instanceId == 1 && *pState == 3)) {
                *pState = 0;
                *(pState + 1) = 0;
            }
        }
    }

    for (int group = 0; group < 4; group++) {
        for (int slot = 0; slot < 64; slot++) {
            uint32_t* pState = reinterpret_cast<uint32_t*>(memBase + 0x1e10 + ((group * 64) + slot) * 40);
            if (*pState == 2 || (self.m_instanceId == 1 && *pState == 3)) {
                *pState = 0;
                *(pState + 1) = 0;
            }
        }
    }

    uint32_t ring878Offsets[] = {
        0x3c10, 0x4488, 0x4d00, 0x5578, 0x5df0, 0x6668, 0x6ee0, 0x7758,
        0x7fd0, 0x7fdc, 0x7fe8, 0x7ff4, 0x8000, 0x800c, 0x8018, 0x8024
    };

    for (uint32_t offset : ring878Offsets) {
        uint32_t* pState = reinterpret_cast<uint32_t*>(memBase + offset);
        if (*pState == 2 || (self.m_instanceId == 1 && *pState == 3)) {
            *pState = 0;
            *(pState + 2) = 0;
        }
    }

    for (int i = 0; i < 11; i++) {
        char configName[32];
        snprintf(configName, sizeof(configName), "CONFIG_ID_%d", i);
        self.m_hConfigMutexes[i] = new HANDLE(CreateMutexA(nullptr, FALSE, configName));
    }

    self.InitializeSub_ad30();

    if (self.m_instanceId == 1 || self.m_instanceId == 4) {
        self.InitializeSub_bdb0();

        void* threadInfo2 = operator new(0x10);
        if (threadInfo2) {
            void* threadContext2 = operator new(0x8);
            if (threadContext2) {
                *reinterpret_cast<ShareManager**>(threadContext2) = &self;
            }
            unsigned int threadId;
            uintptr_t hThread = _beginthreadex(
                nullptr, 0, &ShareManager::EvfWorkerThread_Conditional, threadContext2, 0, &threadId
            );
            if (hThread == 0) {
                std::abort();
            }
            *reinterpret_cast<uintptr_t*>(threadInfo2) = hThread;
            *reinterpret_cast<unsigned int*>(reinterpret_cast<uint8_t*>(threadInfo2) + 8) = threadId;
            self.m_hConfigMutexes[11] = reinterpret_cast<HANDLE*>(threadInfo2);
        }
    }

    *reinterpret_cast<uint16_t*>(memBase + 0x18) = 0x29;

    LARGE_INTEGER qpcFreq, qpcCount;
    QueryPerformanceFrequency(&qpcFreq);
    QueryPerformanceCounter(&qpcCount);

    double timestamp = (static_cast<double>(qpcCount.QuadPart) / static_cast<double>(qpcFreq.QuadPart)) * 1000000.0;

    self.m_pMem->watchdog.timestamps[self.m_instanceId] = timestamp;
    self.m_pMem->watchdog.pids[self.m_instanceId] = GetCurrentProcessId();
    self.m_pMem->watchdog.states[self.m_instanceId] = 0x29;
}

void ShareManager::RegisterEventCallback(this ShareManager& self, uint64_t mask, std::function<void()>* pCallback) {
    GlobalEventContext* pCtx = self.m_pEventContext;

    if (pCtx && pCallback) {
        WaitForSingleObject(pCtx->hMutex, INFINITE);
        pCtx->callbacks.push_back({*pCallback, mask});
        ReleaseMutex(pCtx->hMutex);
    }
}

void ShareManager::WaitDynamicEvent(GlobalEventContext** ppCtx) {
    if (!ppCtx) return;
    GlobalEventContext* pCtx = *ppCtx;
    if (!pCtx || pCtx->exitFlag != '\0') return;

    do {
        WaitForSingleObject(pCtx->hMutex, INFINITE);
        uint64_t currentFlags = *pCtx->pSharedFlags;

        if (currentFlags == 0) {
            ReleaseMutex(pCtx->hMutex);
            DWORD res = WaitForSingleObject(pCtx->hEvent, 100);
            ResetEvent(pCtx->hEvent);
            if (res != WAIT_TIMEOUT) continue;
        } else {
            *pCtx->pSharedFlags = 0;
            ReleaseMutex(pCtx->hMutex);

            for (const auto& cb : pCtx->callbacks) {
                if ((currentFlags & cb.second) == cb.second) {
                    if (cb.first) {
                        cb.first();
                    }
                }
            }
        }
    } while (pCtx->exitFlag == '\0');
}


void ShareManager::GetIntConfig(this ShareManager& self, int configId, long* outValue) {
    if (configId < 0 || configId >= 11 || !outValue) return;

    if (self.m_hConfigMutexes[configId]) {
        WaitForSingleObject(*self.m_hConfigMutexes[configId], INFINITE);
    }

    const char* strData = self.m_pMem->configs_9e00.str_configs[configId].stringData;

    if (strData[0] != '\0') {
        *outValue = std::strtol(strData, nullptr, 10);
    }

    if (self.m_hConfigMutexes[configId]) {
        ReleaseMutex(*self.m_hConfigMutexes[configId]);
    }
}

static const char* GetHostStringPtr(const void* hostStrObj) {
    if (!hostStrObj) return "";
    const size_t* ptr = reinterpret_cast<const size_t*>(hostStrObj);
    size_t capacity = ptr[3];
    if (capacity > 15) {
        return *reinterpret_cast<const char* const*>(hostStrObj);
    } else {
        return reinterpret_cast<const char*>(hostStrObj);
    }
}

uint32_t ShareManager::ReadStringConfig(this ShareManager& self, int configId, std::string& outStr) {
    if (configId < 0 || configId >= 11) return 0xFFFFFFFF;

    if (self.m_hConfigMutexes[configId]) {
        WaitForSingleObject(*self.m_hConfigMutexes[configId], INFINITE);
    }

    uint32_t counter = self.m_pMem->configs_9e00.str_configs[configId].counter;
    const char* strData = self.m_pMem->configs_9e00.str_configs[configId].stringData;

    outStr = strData;

    if (self.m_hConfigMutexes[configId]) {
        ReleaseMutex(*self.m_hConfigMutexes[configId]);
    }
    return counter;
}

void ShareManager::WriteConfigString(this ShareManager& self, int configId, const std::string& str) {
    if (configId < 0 || configId >= 11) return;

    if (self.m_hConfigMutexes[configId]) {
        WaitForSingleObject(*self.m_hConfigMutexes[configId], INFINITE);
    }

    uint32_t& counter = self.m_pMem->configs_9e00.str_configs[configId].counter;
    char* strData = self.m_pMem->configs_9e00.str_configs[configId].stringData;

    strncpy_s(strData, 264, str.c_str(), _TRUNCATE);

    counter += 1;

    if (self.m_hConfigMutexes[configId]) {
        ReleaseMutex(*self.m_hConfigMutexes[configId]);
    }
}

// We need these _Hook versions because std::string is not the same when building in Release and Debug.
// If we don't, it crashes/writes garbage when PSVR2TK is built as Debug.
// Also if our std::string changes because of the compiler or whatnot, we'd crash regardless.

uint32_t ShareManager::ReadStringConfig_Hook(this ShareManager& self, int configId, void* outStrObj) {
    std::string tempStr;
    uint32_t counter = self.ReadStringConfig(configId, tempStr);

    if (outStrObj) {
        uint64_t baseAddress = HmdDriverLoader::Instance()->GetBaseAddress();
        using AssignStringFn = void* (*)(void* target, const char* source, size_t length);
        auto AssignString = reinterpret_cast<AssignStringFn>(baseAddress + 0x4e8f0);

        AssignString(outStrObj, tempStr.c_str(), tempStr.length());
    }

    return counter;
}

void ShareManager::WriteConfigString_Hook(this ShareManager& self, int configId, const void* strObj) {
    if (configId < 0 || configId >= 11) return;

    const char* str = GetHostStringPtr(strObj);
    self.WriteConfigString(configId, str);
}

int ShareManager::ReadLogStrings(this ShareManager& self, long long destAddress, int maxCount) {
    int readCount = 0;
    self.Lock(29);
    uint8_t head = self.m_pMem->logs.log_head;
    uint8_t tail = self.m_pMem->logs.log_tail;

    while (head != tail && readCount < maxCount) {
        uint8_t* destEntry = reinterpret_cast<uint8_t*>(destAddress + (readCount * 0x104));
        LogMetadata& meta = self.m_pMem->logs.log_meta[tail];

        destEntry[0] = meta.instanceId;
        destEntry[1] = meta.level;
        destEntry[2] = meta.length;
        destEntry[3] = meta.pad;

        strncpy_s(reinterpret_cast<char*>(destEntry + 4), 0x100, self.m_pMem->logs.log_strings[tail], meta.length);
        destEntry[258] = 0;

        tail++;
        readCount++;
    }
    self.m_pMem->logs.log_tail = tail;
    self.Unlock(29);
    return readCount;
}

void ShareManager::WriteLogString(this ShareManager& self, char level, const char* format, ...) {
    va_list args;
    va_start(args, format);

    self.Lock(29);
    uint8_t head = self.m_pMem->logs.log_head;

    char* destStr = self.m_pMem->logs.log_strings[head];
    int len = vsnprintf(destStr, 256, format, args);

    va_end(args);

    if (len < 0) {
        len = -1;
    }

    LogMetadata& meta = self.m_pMem->logs.log_meta[head];
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

    self.Unlock(29);
    if (self.m_hEvents[29]) SetEvent(*self.m_hEvents[29]);
}

void ShareManager::ReadIntConfigSafe(this ShareManager& self, int configId, int* outValue) {
    if (!outValue) return;

    std::string strVal;
    self.ReadStringConfig(configId, strVal);

    if (!strVal.empty()) {
        try {
            *outValue = std::stoi(strVal);
        }
        catch (const std::invalid_argument&) {
            Util::DriverLog("ReadIntConfigSafe failed: configId={}, strVal={}", configId, strVal.c_str());
        }
    }
}


uint64_t ShareManager::UpdateProcessExitCodes(this ShareManager& self, void* outData) {
    self.Lock(0);
    for (int i = 0; i < 11; i++) {
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
    self.Unlock(0);
    return 0;
}

uint32_t ShareManager::ReadCommon_0x10(this ShareManager& self, uint64_t* outData) {
    self.Lock(0);
    if (outData) *outData = self.m_pMem->common_0x10.data_0x10;
    uint32_t cnt = self.m_pMem->common_0x10.counter_0x8;
    self.Unlock(0);
    return cnt;
}

int ShareManager::WriteCommon_0x10(this ShareManager& self, uint64_t data) {
    self.Lock(0);
    self.m_pMem->common_0x10.data_0x10 = data;
    int cnt = ++self.m_pMem->common_0x10.counter_0x8;
    self.Unlock(0);
    return cnt;
}

int ShareManager::ReadCommon_0x18(this ShareManager& self) {
    self.Lock(0);
    int val = self.m_pMem->common_0x18.data_0x18;
    self.Unlock(0);
    return val;
}

uint32_t ShareManager::ReadCommon_0x9dd0(this ShareManager& self, uint64_t* outData) {
    self.Lock(0);
    if (outData) *outData = self.m_pMem->common_9dd0.data_0x9dd0;
    uint32_t cnt = self.m_pMem->common_9dd0.counter_0x9dc8;
    self.Unlock(0);
    return cnt;
}

int ShareManager::WriteCommon_0x9dd0(this ShareManager& self, uint64_t data) {
    self.Lock(0);
    self.m_pMem->common_9dd0.data_0x9dd0 = data;
    int cnt = ++self.m_pMem->common_9dd0.counter_0x9dc8;
    self.Unlock(0);
    return cnt;
}

uint32_t ShareManager::ReadCommon_0xc72c(this ShareManager& self, uint8_t* outData) {
    self.Lock(0);
    if (outData) *outData = self.m_pMem->common_c72c.data_0xc72c;
    uint32_t cnt = self.m_pMem->common_c72c.ctr;
    self.Unlock(0);
    return cnt;
}

int ShareManager::WriteCommon_0xc72c(this ShareManager& self, uint8_t data) {
    self.Lock(0);
    self.m_pMem->common_c72c.data_0xc72c = data;
    int cnt = ++self.m_pMem->common_c72c.ctr;
    self.Unlock(0);
    return cnt;
}

uint32_t ShareManager::ReadPlayareaSetup_0xc814(this ShareManager& self, void* outData) {
    self.Lock(35);
    if (outData) *(uint32_t*)outData = self.m_pMem->playarea_setup_c814.data;
    uint32_t cnt = self.m_pMem->playarea_setup_c814.ctr;
    self.Unlock(35);
    return cnt;
}

int ShareManager::WritePlayareaSetup_0xc814(this ShareManager& self, uint32_t data) {
    self.Lock(35);
    self.m_pMem->playarea_setup_c814.data = data;
    int cnt = ++self.m_pMem->playarea_setup_c814.ctr;
    self.Unlock(35);
    return cnt;
}

uint32_t ShareManager::ReadArmModel_0x84a4(this ShareManager& self, void* outData, int param) {
    self.Lock(16);
    if (param < 0 || param >= 2) {
        self.Unlock(16);
        return 0;
    }
    auto& slot = self.m_pMem->arm_model_84a4.slots[param];
    if (outData) memcpy(outData, slot.data, 0x50);
    uint32_t cnt = slot.ctr;
    self.Unlock(16);
    return cnt;
}

int ShareManager::WriteArmModel_0x84a4(this ShareManager& self, void* data, int param) {
    self.Lock(16);
    if (param < 0 || param >= 2) {
        self.Unlock(16);
        return 0;
    }
    auto& slot = self.m_pMem->arm_model_84a4.slots[param];
    if (data) memcpy(slot.data, data, 0x50);
    int cnt = ++slot.ctr;
    self.Unlock(16);
    return cnt;
}

uint32_t ShareManager::ReadBtQualityInfo_0x9170(this ShareManager& self, void* outData, int param) {
    self.Lock(18);
    if (param < 0 || param >= 2) {
        self.Unlock(18);
        return 0;
    }
    auto& slot = self.m_pMem->bt_qual_9170.slots[param];
    if (outData) memcpy(outData, slot.data, 80);
    uint32_t cnt = slot.ctr;
    self.Unlock(18);
    return cnt;
}

int ShareManager::WriteBtQualityInfo_0x9170(this ShareManager& self, void* data, int param) {
    self.Lock(18);
    if (param < 0 || param >= 2) {
        self.Unlock(18);
        return 0;
    }
    auto& slot = self.m_pMem->bt_qual_9170.slots[param];
    if (data) memcpy(slot.data, data, 80);
    int cnt = ++slot.ctr;
    self.Unlock(18);
    return cnt;
}

uint32_t ShareManager::AcquireImageWriteSlot(this ShareManager& self, long long* outImageBuffer, long long* outTrackingData) {
    uint32_t result = 0xFFFFFFFF;
    self.Lock(9);

    uint32_t writeIdx = self.m_writeIndex[0];

    for (int i = 0; i < 8; i++) {
        writeIdx = (writeIdx + 1) % 8;
        ImageSlotMeta& slot = self.m_pMem->image_meta.slots[writeIdx];

        if (slot.state < 2) {
            slot.state = 3;
            if (outImageBuffer) *outImageBuffer = reinterpret_cast<long long>(self.m_pMem->image_data.slots[writeIdx].data);
            if (outTrackingData) *outTrackingData = reinterpret_cast<long long>(slot.trackingData);
            result = writeIdx;
            break;
        }
    }
    self.Unlock(9);
    return result;
}

uint32_t ShareManager::AcquireInputWriteSlot(this ShareManager& self, int groupIdx, long long* outOffset) {
    if (groupIdx < 0 || groupIdx >= 3) return 0xFFFFFFFF;

    uint32_t result = 0xFFFFFFFF;
    int mutexIdx = 3 + groupIdx;
    self.Lock(mutexIdx);

    uint32_t writeIdx = self.m_writeIndex_40[groupIdx];

    for (int i = 0; i < 64; i++) {
        writeIdx = (writeIdx + 1) % 64;
        InputSlotMeta& slot = self.m_pMem->input_meta.groups[groupIdx].slots[writeIdx];

        if (slot.state < 2) {
            slot.state = 3;
            if (outOffset) *outOffset = reinterpret_cast<long long>(self.m_pMem->input_data.slots[groupIdx][writeIdx]);
            result = writeIdx;
            break;
        }
    }
    self.Unlock(mutexIdx);
    return result;
}

uint32_t ShareManager::AcquirePoseWriteSlot(this ShareManager& self, int groupIdx, long long* outOffset) {
    if (groupIdx < 0 || groupIdx >= 4) return 0xFFFFFFFF;

    uint32_t result = 0xFFFFFFFF;
    int mutexIdx = 6 + groupIdx;
    self.Lock(mutexIdx);

    uint32_t writeIdx = self.m_writeIndex_28[groupIdx];

    for (int i = 0; i < 64; i++) {
        writeIdx = (writeIdx + 1) % 64;
        PoseSlotMeta& slot = self.m_pMem->pose_meta.groups[groupIdx].slots[writeIdx];

        if (slot.state < 2) {
            slot.state = 3;
            if (outOffset) *outOffset = reinterpret_cast<long long>(self.m_pMem->pose_data.slots[groupIdx][writeIdx].data);
            result = writeIdx;
            break;
        }
    }
    self.Unlock(mutexIdx);
    return result;
}

void ShareManager::CommitImageWriteSlot(this ShareManager& self, uint32_t slotIdx) {
    if (slotIdx >= 8) return;
    self.Lock(9);

    ImageSlotMeta& slot = self.m_pMem->image_meta.slots[slotIdx];
    slot.state = 1;
    slot.sequenceId = self.m_sequence[0]++;

    uint32_t property = *reinterpret_cast<uint32_t*>(&self.m_pMem->image_data.slots[slotIdx].data[0x8f0]);
    slot.property = property;

    uint32_t exposureVal;
    if (property < 0x100) {
        exposureVal = 0x3f800000; // 1.0f
    } else {
        exposureVal = *reinterpret_cast<uint32_t*>(&self.m_pMem->image_data.slots[slotIdx].data[0x8f4]);
    }
    slot.exposure = exposureVal;

    self.Unlock(9);
    self.m_writeIndex[0] = slotIdx;
    if (self.m_hEvents[9]) SetEvent(*self.m_hEvents[9]);
}

void ShareManager::CommitInputWriteSlot(this ShareManager& self, int groupIdx, uint32_t slotIdx) {
    if (groupIdx < 0 || groupIdx >= 3 || slotIdx >= 64) return;

    int mutexIdx = 3 + groupIdx;
    self.Lock(mutexIdx);

    InputSlotMeta& slot = self.m_pMem->input_meta.groups[groupIdx].slots[slotIdx];
    slot.state = 1;
    slot.sequenceId = self.m_inputSequences[groupIdx]++;

    self.Unlock(mutexIdx);
    self.m_writeIndex_40[groupIdx] = slotIdx;
    if (self.m_hEvents[mutexIdx]) SetEvent(*self.m_hEvents[mutexIdx]);
}

void ShareManager::CommitPoseWriteSlot(this ShareManager& self, int groupIdx, uint32_t slotIdx, void* params) {
    if (groupIdx < 0 || groupIdx >= 4 || slotIdx >= 64) return;

    int mutexIdx = 6 + groupIdx;
    self.Lock(mutexIdx);

    PoseSlotMeta& slot = self.m_pMem->pose_meta.groups[groupIdx].slots[slotIdx];

    if (params) {
        memcpy(slot.params, params, 24);
    }

    slot.state = 1;
    slot.sequenceId = self.m_poseSequences[groupIdx]++;

    self.Unlock(mutexIdx);
    self.m_writeIndex_28[groupIdx] = slotIdx;
    if (self.m_hEvents[mutexIdx]) SetEvent(*self.m_hEvents[mutexIdx]);
}

uint32_t ShareManager::AcquireImageReadSlot(this ShareManager& self, long long* outImageBuffer, void* outTrackingData, void* outUnknown) {
    uint32_t result = 0xFFFFFFFF;
    uint32_t highestSeq = 0;
    self.Lock(9);

    for (uint32_t i = 0; i < 8; i++) {
        ImageSlotMeta& slot = self.m_pMem->image_meta.slots[i];
        if (slot.state == 1 || slot.state == 2) {
            if (result == 0xFFFFFFFF || slot.sequenceId >= highestSeq) {
                highestSeq = slot.sequenceId;
                result = i;
            }
        }
    }

    if (result != 0xFFFFFFFF) {
        ImageSlotMeta& slot = self.m_pMem->image_meta.slots[result];
        slot.state = 2;
        slot.read_counter += 1;
        if (outImageBuffer) *outImageBuffer = reinterpret_cast<long long>(self.m_pMem->image_data.slots[result].data);
        if (outTrackingData) memcpy(outTrackingData, slot.trackingData, 56);
        if (outUnknown) memcpy(outUnknown, &slot.exposure, 4);
    }
    self.Unlock(9);
    return result;
}

uint32_t ShareManager::AcquireInputReadSlot(this ShareManager& self, int groupIdx, long long* outOffset) {
    if (groupIdx < 0 || groupIdx >= 3) return 0xFFFFFFFF;

    uint32_t result = 0xFFFFFFFF;
    uint32_t highestSeq = 0;
    int mutexIdx = 3 + groupIdx;
    self.Lock(mutexIdx);

    for (uint32_t i = 0; i < 64; i++) {
        InputSlotMeta& slot = self.m_pMem->input_meta.groups[groupIdx].slots[i];
        if (slot.state == 1 || slot.state == 2) {
            if (result == 0xFFFFFFFF || slot.sequenceId >= highestSeq) {
                highestSeq = slot.sequenceId;
                result = i;
            }
        }
    }

    if (result != 0xFFFFFFFF) {
        InputSlotMeta& slot = self.m_pMem->input_meta.groups[groupIdx].slots[result];
        slot.state = 2;
        slot.read_counter += 1;
        if (outOffset) *outOffset = reinterpret_cast<long long>(self.m_pMem->input_data.slots[groupIdx][result]);
    }
    self.Unlock(mutexIdx);
    return result;
}

uint32_t ShareManager::AcquirePoseReadSlot(this ShareManager& self, int groupIdx, long long* outOffset, void* outParams) {
    if (groupIdx < 0 || groupIdx >= 4) return 0xFFFFFFFF;

    uint32_t result = 0xFFFFFFFF;
    uint32_t highestSeq = 0;
    int mutexIdx = 6 + groupIdx;
    self.Lock(mutexIdx);

    for (uint32_t i = 0; i < 64; i++) {
        PoseSlotMeta& slot = self.m_pMem->pose_meta.groups[groupIdx].slots[i];
        if (slot.state == 1 || slot.state == 2) {
            if (result == 0xFFFFFFFF || slot.sequenceId >= highestSeq) {
                highestSeq = slot.sequenceId;
                result = i;
            }
        }
    }

    if (result != 0xFFFFFFFF) {
        PoseSlotMeta& slot = self.m_pMem->pose_meta.groups[groupIdx].slots[result];
        slot.state = 2;
        slot.read_counter += 1;
        if (outOffset) *outOffset = reinterpret_cast<long long>(self.m_pMem->pose_data.slots[groupIdx][result].data);
        if (outParams) memcpy(outParams, slot.params, 24);
    }
    self.Unlock(mutexIdx);
    return result;
}

void ShareManager::CommitImageReadSlot(this ShareManager& self, uint32_t slotIdx) {
    if (slotIdx >= 8) return;
    self.Lock(9);
    ImageSlotMeta& slot = self.m_pMem->image_meta.slots[slotIdx];
    if (slot.read_counter > 0) {
        slot.read_counter -= 1;
        if (slot.read_counter == 0) slot.state = 1;
    }
    self.Unlock(9);
}

void ShareManager::CommitInputReadSlot(this ShareManager& self, int groupIdx, uint32_t slotIdx) {
    if (groupIdx < 0 || groupIdx >= 3 || slotIdx >= 64) return;

    int mutexIdx = 3 + groupIdx;
    self.Lock(mutexIdx);
    InputSlotMeta& slot = self.m_pMem->input_meta.groups[groupIdx].slots[slotIdx];
    if (slot.read_counter > 0) {
        slot.read_counter -= 1;
        if (slot.read_counter == 0) {
            slot.state = 1;
        }
    }
    self.Unlock(mutexIdx);
}

void ShareManager::CommitPoseReadSlot(this ShareManager& self, int groupIdx, uint32_t slotIdx) {
    if (groupIdx < 0 || groupIdx >= 4 || slotIdx >= 64) return;

    int mutexIdx = 6 + groupIdx;
    self.Lock(mutexIdx);
    PoseSlotMeta& slot = self.m_pMem->pose_meta.groups[groupIdx].slots[slotIdx];
    if (slot.read_counter > 0) {
        slot.read_counter -= 1;
        if (slot.read_counter == 0) {
            slot.state = 1;
        }
    }
    self.Unlock(mutexIdx);
}

uint64_t ShareManager::TryAcquireShareMutex(this ShareManager& self, uint32_t typeIndex) {
    if (typeIndex > 1) return 0xFFFFFFFFFFFFFF00;

    int realIdx = 26 + typeIndex;
    if (!self.m_hMutexes[realIdx]) return 0xFFFFFFFFFFFFFF00;
    DWORD res = WaitForSingleObject(*self.m_hMutexes[realIdx], 0);

    if (res == WAIT_OBJECT_0 || res == WAIT_ABANDONED) {
        ReleaseMutex(*self.m_hMutexes[realIdx]);
        return 0;
    }

    return 1;
}

uint32_t ShareManager::AcquireShareMutex(this ShareManager& self, uint32_t typeIndex) {
    if (typeIndex > 1) {
        return 0xFFFFFFFF;
    }

    if (self.m_hMutexes[26 + typeIndex]) {
        WaitForSingleObject(*self.m_hMutexes[26 + typeIndex], INFINITE);
    }

    return 0;
}

uint32_t ShareManager::ReleaseShareMutex(this ShareManager& self, uint32_t typeIndex) {
    if (typeIndex > 1) {
        return 0xFFFFFFFF;
    }

    if (self.m_hMutexes[26 + typeIndex]) {
        ReleaseMutex(*self.m_hMutexes[26 + typeIndex]);
    }

    return 0;
}

uint32_t ShareManager::TryAcquireLibpadMutex(this ShareManager& self) {
    if (!self.m_hMutexes[25]) return 0xFFFFFFFF;
    DWORD res = WaitForSingleObject(*self.m_hMutexes[25], 0);

    if (res == WAIT_OBJECT_0 || res == WAIT_ABANDONED) {
        return 0;
    }

    return 0xFFFFFFFF;
}

int ShareManager::ReleaseLibpadMutex(this ShareManager& self) {
    if (!self.m_hMutexes[25]) return 0;
    return ReleaseMutex(*self.m_hMutexes[25]) ? 1 : 0;
}

void ShareManager::ReleaseMutexByIndex(this ShareManager& self, int index) {
    if (self.m_hMutexes[index]) {
        ReleaseMutex(*self.m_hMutexes[index]);
    }
}

void ShareManager::ClearLogEventFlag(this ShareManager& self) {
    self.Lock(29);
    self.m_pMem->logs.log_head = 0;
    self.Unlock(29);
}

void ShareManager::SetGlobalEventFlag(this ShareManager& self, uint64_t flags) {
    GlobalEventContext* pCtx = self.m_pEventContext;
    if (pCtx) {
        if (pCtx->hMutex) WaitForSingleObject(*pCtx->hMutex, INFINITE);
        *pCtx->pSharedFlags |= flags;
        if (pCtx->hEvent) SetEvent(*pCtx->hEvent);
        if (pCtx->hMutex) ReleaseMutex(*pCtx->hMutex);
    }
}

uint32_t ShareManager::ReadPlayareaResult(this ShareManager& self, void* outData) {
    self.Lock(11);

    uint32_t bestSlot = 0xFFFFFFFF;
    uint32_t highestSeq = 0;

    for (uint32_t i = 0; i < 8; i++) {
        PlayareaSlotState& slot = self.m_pMem->playarea_meta.playarea_states[i];
        if (slot.state == 1 || slot.state == 2) {
            if (bestSlot == 0xFFFFFFFF || slot.sequenceId >= highestSeq) {
                highestSeq = slot.sequenceId;
                bestSlot = i;
            }
        }
    }

    if (bestSlot != 0xFFFFFFFF) {
        PlayareaSlotState& slot = self.m_pMem->playarea_meta.playarea_states[bestSlot];
        slot.state = 2;

        if (outData) {
            memcpy(outData, self.m_pMem->playarea_ring.playarea_slots[bestSlot], 64);
        }

        slot.state = 1;
    }

    self.Unlock(11);
    return bestSlot;
}

uint32_t ShareManager::WritePlayareaResult(this ShareManager& self, void* data) {
    self.Lock(11);

    uint32_t targetSlot = 0xFFFFFFFF;
    uint32_t highestSeq = 0;

    for (uint32_t i = 0; i < 8; i++) {
        PlayareaSlotState& slot = self.m_pMem->playarea_meta.playarea_states[i];
        if (slot.sequenceId > highestSeq) {
            highestSeq = slot.sequenceId;
        }

        if (targetSlot == 0xFFFFFFFF && slot.state < 2) {
            targetSlot = i;
        }
    }

    if (targetSlot == 0xFFFFFFFF) targetSlot = 0;

    PlayareaSlotState& slot = self.m_pMem->playarea_meta.playarea_states[targetSlot];
    slot.state = 3;

    if (data) {
        memcpy(self.m_pMem->playarea_ring.playarea_slots[targetSlot], data, 64);
    }

    slot.sequenceId = highestSeq + 1;
    slot.state = 1;

    self.Unlock(11);
    return targetSlot;
}

int ShareManager::WriteFwInfo2_0x9280(this ShareManager& self, void* data) {
    self.Lock(20);
    if (data) {
        memcpy(self.m_pMem->fw_info2_9280.data, data, sizeof(self.m_pMem->fw_info2_9280.data));
    }
    int cnt = ++self.m_pMem->fw_info2_9280.ctr;
    self.Unlock(20);

    self.SetGlobalEventFlag(0x40);

    return cnt;
}

uint32_t ShareManager::WaitShareEvent(this ShareManager& self, int groupIdx, DWORD timeoutMs) {
    if (!self.m_hEvents[groupIdx]) return 0xFFFFFFFF;
    DWORD result = WaitForSingleObject(*self.m_hEvents[groupIdx], timeoutMs);

    if (result != WAIT_OBJECT_0) {
        return 0xFFFFFFFF;
    }

    ResetEvent(*self.m_hEvents[groupIdx]);
    return 0;
}

#define IMPL_READ_MEMCPY_RET(FuncName, GroupIdx, MemGroup, MemData, Size) \
    uint32_t ShareManager::FuncName(this ShareManager& self, void* outData) { \
        self.Lock(GroupIdx); \
        if (outData) memcpy(outData, self.m_pMem->MemGroup.MemData, Size); \
        uint32_t cnt = self.m_pMem->MemGroup.ctr; \
        self.Unlock(GroupIdx); \
        return cnt; \
    }

#define IMPL_READ_MEMCPY_OUT(FuncName, GroupIdx, MemGroup, MemData, Size) \
    uint32_t ShareManager::FuncName(this ShareManager& self, void* outData, uint32_t* outCounter) { \
        self.Lock(GroupIdx); \
        if (outData) memcpy(outData, self.m_pMem->MemGroup.MemData, Size); \
        if (outCounter) *outCounter = self.m_pMem->MemGroup.ctr; \
        self.Unlock(GroupIdx); \
        return 0; \
    }

#define IMPL_WRITE_MEMCPY(FuncName, GroupIdx, MemGroup, MemData, Size) \
    int ShareManager::FuncName(this ShareManager& self, void* data) { \
        self.Lock(GroupIdx); \
        if (data) memcpy(self.m_pMem->MemGroup.MemData, data, Size); \
        int cnt = ++self.m_pMem->MemGroup.ctr; \
        self.Unlock(GroupIdx); \
        return cnt; \
    }

// Group 1: Status
IMPL_READ_MEMCPY_RET(ReadStatus_0x2c, 1, status_0x2c, data_0x2c, sizeof(self.m_pMem->status_0x2c.data_0x2c))
IMPL_WRITE_MEMCPY(WriteStatus_0x2c, 1, status_0x2c, data_0x2c, sizeof(self.m_pMem->status_0x2c.data_0x2c))

// Group 2: Calibration Blocks
IMPL_READ_MEMCPY_OUT(ReadCalib_0x70,  2, calib_0x70,  data_0x70,  sizeof(self.m_pMem->calib_0x70.data_0x70))
IMPL_WRITE_MEMCPY(WriteCalib_0x70,    2, calib_0x70,  data_0x70,  sizeof(self.m_pMem->calib_0x70.data_0x70))
IMPL_READ_MEMCPY_OUT(ReadCalib_0x170, 2, calib_170,   data_0x170, sizeof(self.m_pMem->calib_170.data_0x170))
IMPL_WRITE_MEMCPY(WriteCalib_0x170,   2, calib_170,   data_0x170, sizeof(self.m_pMem->calib_170.data_0x170))
IMPL_READ_MEMCPY_OUT(ReadCalib_0x370, 2, calib_370,   data_0x370, sizeof(self.m_pMem->calib_370.data_0x370))
IMPL_WRITE_MEMCPY(WriteCalib_0x370,   2, calib_370,   data_0x370, sizeof(self.m_pMem->calib_370.data_0x370))
IMPL_READ_MEMCPY_OUT(ReadCalib_0x3f0, 2, calib_3f0,   data_0x3f0, sizeof(self.m_pMem->calib_3f0.data_0x3f0))
IMPL_WRITE_MEMCPY(WriteCalib_0x3f0,   2, calib_3f0,   data_0x3f0, sizeof(self.m_pMem->calib_3f0.data_0x3f0))
IMPL_READ_MEMCPY_OUT(ReadCalib_0x41c, 2, calib_41c,   data_0x41c, sizeof(self.m_pMem->calib_41c.data_0x41c))
IMPL_WRITE_MEMCPY(WriteCalib_0x41c,   2, calib_41c,   data_0x41c, sizeof(self.m_pMem->calib_41c.data_0x41c))
IMPL_READ_MEMCPY_OUT(ReadCalib_0x50c, 2, calib_50c,   data_0x50c, sizeof(self.m_pMem->calib_50c.data_0x50c))
IMPL_WRITE_MEMCPY(WriteCalib_0x50c,   2, calib_50c,   data_0x50c, sizeof(self.m_pMem->calib_50c.data_0x50c))
IMPL_READ_MEMCPY_OUT(ReadCalib_0xd0c, 2, calib_d0c,   data_0xd0c, sizeof(self.m_pMem->calib_d0c.data_0xd0c))
IMPL_WRITE_MEMCPY(WriteCalib_0xd0c,   2, calib_d0c,   data_0xd0c, sizeof(self.m_pMem->calib_d0c.data_0xd0c))

// Groups 12-19: Configs & Info
IMPL_READ_MEMCPY_OUT(ReadImageSetting_0x803c, 12, img_setting_803c, data, sizeof(self.m_pMem->img_setting_803c.data))
IMPL_WRITE_MEMCPY(WriteImageSetting_0x803c,   12, img_setting_803c, data, sizeof(self.m_pMem->img_setting_803c.data))
IMPL_READ_MEMCPY_OUT(ReadBlobConfig_0x8058,   13, blob_cfg_8058,    data, sizeof(self.m_pMem->blob_cfg_8058.data))
IMPL_WRITE_MEMCPY(WriteBlobConfig_0x8058,     13, blob_cfg_8058,    data, sizeof(self.m_pMem->blob_cfg_8058.data))
IMPL_READ_MEMCPY_OUT(ReadIrCamSetting_0x807c, 14, ir_cam_807c,      data, sizeof(self.m_pMem->ir_cam_807c.data))
IMPL_WRITE_MEMCPY(WriteIrCamSetting_0x807c,   14, ir_cam_807c,      data, sizeof(self.m_pMem->ir_cam_807c.data))
IMPL_READ_MEMCPY_RET(ReadContConfig_0x8098,   15, cont_cfg_8098,    data, sizeof(self.m_pMem->cont_cfg_8098.data))
IMPL_WRITE_MEMCPY(WriteContConfig_0x8098,     15, cont_cfg_8098,    data, sizeof(self.m_pMem->cont_cfg_8098.data))
IMPL_READ_MEMCPY_RET(ReadContLedInfo_0x855c,  17, cont_led_855c,    data, sizeof(self.m_pMem->cont_led_855c.data))
IMPL_WRITE_MEMCPY(WriteContLedInfo_0x855c,    17, cont_led_855c,    data, sizeof(self.m_pMem->cont_led_855c.data))
IMPL_READ_MEMCPY_RET(ReadFwInfo1_0x922c,      19, fw_info1_922c,    data, sizeof(self.m_pMem->fw_info1_922c.data))
IMPL_WRITE_MEMCPY(WriteFwInfo1_0x922c,        19, fw_info1_922c,    data, sizeof(self.m_pMem->fw_info1_922c.data))

// Groups 20-24: Application & Debug
IMPL_READ_MEMCPY_RET(ReadFwInfo2_0x9280,      20, fw_info2_9280, data, sizeof(self.m_pMem->fw_info2_9280.data))
IMPL_READ_MEMCPY_RET(ReadVrDialog_0x9ae0,     21, vr_dialog_9ae0, data, sizeof(self.m_pMem->vr_dialog_9ae0.data))
IMPL_WRITE_MEMCPY(WriteVrDialog_0x9ae0,       21, vr_dialog_9ae0, data, sizeof(self.m_pMem->vr_dialog_9ae0.data))
IMPL_READ_MEMCPY_RET(ReadApplication_0x9b3c,  22, app_9b3c,      data, sizeof(self.m_pMem->app_9b3c.data))
IMPL_READ_MEMCPY_RET(ReadVrTraceData_0x9b7c,  23, vr_trace_9b7c, data, sizeof(self.m_pMem->vr_trace_9b7c.data))
IMPL_WRITE_MEMCPY(WriteVrTraceData_0x9b7c,    23, vr_trace_9b7c, data, sizeof(self.m_pMem->vr_trace_9b7c.data))
IMPL_READ_MEMCPY_RET(ReadDebugData_0x9c8c,    24, dbg_data_9c8c, data, sizeof(self.m_pMem->dbg_data_9c8c.data))
IMPL_WRITE_MEMCPY(WriteDebugData_0x9c8c,      24, dbg_data_9c8c, data, sizeof(self.m_pMem->dbg_data_9c8c.data))

// Groups 30-34: Telemetry & Setup
IMPL_READ_MEMCPY_RET(ReadSceneInfo_0x9de4,    33, scene_info_9de4, data, sizeof(self.m_pMem->scene_info_9de4.data))
IMPL_WRITE_MEMCPY(WriteSceneInfo_0x9de4,      33, scene_info_9de4, data, sizeof(self.m_pMem->scene_info_9de4.data))
IMPL_READ_MEMCPY_RET(ReadTelDevInfo_0xa984,   30, tel_dev_a984,    data, sizeof(self.m_pMem->tel_dev_a984.data))
IMPL_WRITE_MEMCPY(WriteTelDevInfo_0xa984,     30, tel_dev_a984,    data, sizeof(self.m_pMem->tel_dev_a984.data))
IMPL_READ_MEMCPY_RET(ReadTelTrkInfo_0xaaa0,   31, tel_trk_aaa0,    data, sizeof(self.m_pMem->tel_trk_aaa0.data))
IMPL_WRITE_MEMCPY(WriteTelTrkInfo_0xaaa0,     31, tel_trk_aaa0,    data, sizeof(self.m_pMem->tel_trk_aaa0.data))
IMPL_READ_MEMCPY_RET(ReadTelPcInfo_0xc600,    32, tel_pc_c600,     data, sizeof(self.m_pMem->tel_pc_c600.data))
IMPL_WRITE_MEMCPY(WriteTelPcInfo_0xc600,      32, tel_pc_c600,     data, sizeof(self.m_pMem->tel_pc_c600.data))
IMPL_READ_MEMCPY_RET(ReadInitSetup_0xc73c,    34, init_setup_c73c, data, sizeof(self.m_pMem->init_setup_c73c.data))
IMPL_WRITE_MEMCPY(WriteInitSetup_0xc73c,      34, init_setup_c73c, data, sizeof(self.m_pMem->init_setup_c73c.data))

unsigned __stdcall ShareManager::WorkerThread_Unconditional(void* pContext) {
    GlobalEventContext* pEvtStruct = *reinterpret_cast<GlobalEventContext**>(pContext);
    delete reinterpret_cast<GlobalEventContext**>(pContext);

    if (pEvtStruct) {
        ShareManager::WaitDynamicEvent(&pEvtStruct);
        if (pEvtStruct->threadInfo) {
            CloseHandle(*reinterpret_cast<HANDLE*>(pEvtStruct->threadInfo));
            operator delete(pEvtStruct->threadInfo);
        }
        delete pEvtStruct;
    }

    return 0;
}

unsigned __stdcall ShareManager::EvfWorkerThread_Conditional(void* pContext) {
    ShareManager* self = *reinterpret_cast<ShareManager**>(pContext);

    delete reinterpret_cast<ShareManager**>(pContext);
    if (!self) return 0;

    char sharedMemBuffer[256];

    while (!self->m_exitThreads) {
        for (int i = 0; i < 11; i++) {
            ConfigMapping& map = self->m_configMappings[i];

            std::string sharedStr;
            self->ReadStringConfig(i, sharedStr);

            strncpy_s(sharedMemBuffer, sizeof(sharedMemBuffer), sharedStr.c_str(), _TRUNCATE);

            if (strncmp(map.CachedValue, sharedMemBuffer, sizeof(map.CachedValue)) != 0) {
                WritePrivateProfileStringA(
                    map.AppName,
                    map.KeyName,
                    sharedMemBuffer,
                    self->m_configIniPath
                );

                strncpy_s(map.CachedValue, sizeof(map.CachedValue), sharedMemBuffer, _TRUNCATE);
            }
        }

        Sleep(1000);
    }

    return 0;
}

void ShareManager::InitializeSub_ad30(this ShareManager& self) {
    self.m_configMappings[0] = {"SafetyNotice", "LastRecordedDateTime", ""};
    self.m_configMappings[1] = {"Controller", "VibrationStrength", "1"};
    self.m_configMappings[2] = {"HMD", "ScreenBrightness", "31"};
    self.m_configMappings[3] = {"SafetyNotice", "LimitDisplay", ""};
    self.m_configMappings[4] = {"VRTrace", "MemorySize", "1"};
    self.m_configMappings[5] = {"InitialSetup", "Done", "1"}; // Default is 1 because PSVR2TK already skips Sony's setup.
    self.m_configMappings[6] = {"DataCollection", "State", ""};
    self.m_configMappings[7] = {"HMD", "CrashCount", "0"};
    self.m_configMappings[8] = {"Telemetry", "OutputLog", "0"};
    self.m_configMappings[9] = {"Controller", "BluetoothNotification", "1"};
    self.m_configMappings[10] = {"Controller", "Status", "0"};

    char szPath[MAX_PATH];
    if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, szPath))) {
        std::string dirPath = std::string(szPath) + "\\Sony\\PlayStation VR2";
        SHCreateDirectoryExA(NULL, dirPath.c_str(), NULL);

        std::string iniPath = dirPath + "\\config.ini";
        strncpy_s(self.m_configIniPath, sizeof(self.m_configIniPath), iniPath.c_str(), _TRUNCATE);

        // Check if the file exists, if not create it empty
        DWORD dwAttrib = GetFileAttributesA(iniPath.c_str());
        if (dwAttrib == INVALID_FILE_ATTRIBUTES) {
            FILE* f = nullptr;
            if (fopen_s(&f, iniPath.c_str(), "w") == 0 && f) {
                fclose(f);
            }
            Util::DriverLog("[VrTracker2] create user config.ini");
        }
    } else {
        snprintf(self.m_configIniPath, sizeof(self.m_configIniPath), "\\Sony\\PlayStation VR2\\config.ini");
    }

    self.m_exitThreads = false;

    for (int i = 0; i < 11; i++) {
        self.m_configMappings[i].CachedValue[0] = '\0';
    }
}

void ShareManager::InitializeSub_bdb0(this ShareManager& self) {
    uint64_t baseAddress = HmdDriverLoader::Instance()->GetBaseAddress();
    auto ValidateConfig = reinterpret_cast<bool (*)(const ShareManager* self, int configId, const char* str)>(baseAddress + 0x15c230);

    char tempBuffer[256];
    std::string sharedStr;

    for (int i = 0; i < 11; i++) {
        ConfigMapping& map = self.m_configMappings[i];
        tempBuffer[0] = '\0';

        GetPrivateProfileStringA(
            map.AppName,
            map.KeyName,
            "",
            tempBuffer,
            sizeof(tempBuffer),
            self.m_configIniPath
        );

        // Remove carriage return / line feed if present
        tempBuffer[strcspn(tempBuffer, "\r\n")] = 0;

        // If the INI config is empty or invalid, fall back to default
        if (tempBuffer[0] == '\0' || !ValidateConfig(&self, i, tempBuffer)) {
            WritePrivateProfileStringA(
                map.AppName,
                map.KeyName,
                map.DefaultValue,
                self.m_configIniPath
            );
            strncpy_s(tempBuffer, sizeof(tempBuffer), map.DefaultValue, _TRUNCATE);
        }

        // Read current shared memory config
        sharedStr.clear();
        self.ReadStringConfig(i, sharedStr);

        // If shared memory config is empty or invalid, write INI config to shared memory
        if (sharedStr.empty() || !ValidateConfig(&self, i, sharedStr.c_str())) {
            self.WriteConfigString(i, tempBuffer);
            strncpy_s(map.CachedValue, sizeof(map.CachedValue), tempBuffer, _TRUNCATE);
        } else {
            strncpy_s(map.CachedValue, sizeof(map.CachedValue), sharedStr.c_str(), _TRUNCATE);
        }
    }
}

void ShareManager::Lock(int idx) {
    if (m_hMutexes[idx]) {
        WaitForSingleObject(*m_hMutexes[idx], INFINITE);
    }
}

void ShareManager::Unlock(int idx) {
    if (m_hMutexes[idx]) {
        ReleaseMutex(*m_hMutexes[idx]);
    }
}