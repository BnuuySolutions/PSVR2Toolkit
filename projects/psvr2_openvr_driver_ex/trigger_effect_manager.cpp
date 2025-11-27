#include "trigger_effect_manager.h"
#include "hmd_driver_loader.h"
#include "util.h"

namespace psvr2_toolkit {

  struct AstonContext_t {
    void* vfptr;
    char unk1[0x28];
    int handle; // libpad handle
    char unk3[0xEFE8C];
  };
  struct AstonManager_t {
    void* vfptr;
    AstonContext_t* contexts[2]; // 0 = Right, 1 = Left
    char unk2[8];
  };

  AstonManager_t* (*getAstonManager)();
  int (*scePadSetTriggerEffect)(int handle, ScePadTriggerEffectParam* param);

  TriggerEffectManager* TriggerEffectManager::m_pInstance = nullptr;

  TriggerEffectManager::TriggerEffectManager()
    : m_initialized(false)
  {
  }

  TriggerEffectManager* TriggerEffectManager::Instance() {
    if (!m_pInstance) {
      m_pInstance = new TriggerEffectManager;
    }

    return m_pInstance;
  }

  bool TriggerEffectManager::Initialized() {
    return m_initialized;
  }

  void TriggerEffectManager::Initialize() {
    static HmdDriverLoader* pHmdDriverLoader = HmdDriverLoader::Instance();

    if (m_initialized) {
      return;
    }

    getAstonManager = decltype(getAstonManager)(pHmdDriverLoader->GetBaseAddress() + 0x1189D0);
    scePadSetTriggerEffect = decltype(scePadSetTriggerEffect)(pHmdDriverLoader->GetBaseAddress() + 0x1BF060);

    m_initialized = true;
  }

  void TriggerEffectManager::HandleIpcCommand(uint32_t processId, ipc::CommandHeader_t* pHeader, void* pData) {
    if (!pData || !pHeader)
      return;
    ScePadTriggerEffectCommand command = {};
    switch (pHeader->type) {
    case ipc::Command_ClientTriggerEffectOff: {
      if (pHeader->dataLen == sizeof(ipc::CommandDataClientTriggerEffectOff_t)) {
        ipc::CommandDataClientTriggerEffectOff_t* pRequest = reinterpret_cast<ipc::CommandDataClientTriggerEffectOff_t*>(pData);
        command.mode = SCE_PAD_TRIGGER_EFFECT_MODE_OFF;
        SetTriggerEffectCommand(processId, pRequest->controllerType, command);
      }
      break;
    }
    case ipc::Command_ClientTriggerEffectFeedback: {
      if (pHeader->dataLen == sizeof(ipc::CommandDataClientTriggerEffectFeedback_t)) {
        ipc::CommandDataClientTriggerEffectFeedback_t* pRequest = reinterpret_cast<ipc::CommandDataClientTriggerEffectFeedback_t*>(pData);
        command.mode = SCE_PAD_TRIGGER_EFFECT_MODE_FEEDBACK;
        command.commandData.feedbackParam.position = pRequest->position;
        command.commandData.feedbackParam.strength = pRequest->strength;
        SetTriggerEffectCommand(processId, pRequest->controllerType, command);
      }
      break;
    }
    case ipc::Command_ClientTriggerEffectWeapon: {
      if (pHeader->dataLen == sizeof(ipc::CommandDataClientTriggerEffectWeapon_t)) {
        ipc::CommandDataClientTriggerEffectWeapon_t* pRequest = reinterpret_cast<ipc::CommandDataClientTriggerEffectWeapon_t*>(pData);
        command.mode = SCE_PAD_TRIGGER_EFFECT_MODE_WEAPON;
        command.commandData.weaponParam.startPosition = pRequest->startPosition;
        command.commandData.weaponParam.endPosition = pRequest->endPosition;
        command.commandData.weaponParam.strength = pRequest->strength;
        SetTriggerEffectCommand(processId, pRequest->controllerType, command);
      }
      break;
    }
    case ipc::Command_ClientTriggerEffectVibration: {
      if (pHeader->dataLen == sizeof(ipc::CommandDataClientTriggerEffectVibration_t)) {
        ipc::CommandDataClientTriggerEffectVibration_t* pRequest = reinterpret_cast<ipc::CommandDataClientTriggerEffectVibration_t*>(pData);
        command.mode = SCE_PAD_TRIGGER_EFFECT_MODE_VIBRATION;
        command.commandData.vibrationParam.position = pRequest->position;
        command.commandData.vibrationParam.amplitude = pRequest->amplitude;
        command.commandData.vibrationParam.frequency = pRequest->frequency;
        SetTriggerEffectCommand(processId, pRequest->controllerType, command);
      }
      break;
    }
    case ipc::Command_ClientTriggerEffectMultiplePositionFeedback: {
      if (pHeader->dataLen == sizeof(ipc::CommandDataClientTriggerEffectMultiplePositionFeedback_t)) {
        ipc::CommandDataClientTriggerEffectMultiplePositionFeedback_t* pRequest = reinterpret_cast<ipc::CommandDataClientTriggerEffectMultiplePositionFeedback_t*>(pData);
        command.mode = SCE_PAD_TRIGGER_EFFECT_MODE_MULTIPLE_POSITION_FEEDBACK;
        for (int i = 0; i < ipc::k_unTriggerEffectControlPoint; i++) {
          command.commandData.multiplePositionFeedbackParam.strength[i] = pRequest->strength[i];
        }
        SetTriggerEffectCommand(processId, pRequest->controllerType, command);
      }
      break;
    }
    case ipc::Command_ClientTriggerEffectSlopeFeedback: {
      if (pHeader->dataLen == sizeof(ipc::CommandDataClientTriggerEffectSlopeFeedback_t)) {
        ipc::CommandDataClientTriggerEffectSlopeFeedback_t* pRequest = reinterpret_cast<ipc::CommandDataClientTriggerEffectSlopeFeedback_t*>(pData);
        command.mode = SCE_PAD_TRIGGER_EFFECT_MODE_SLOPE_FEEDBACK;
        command.commandData.slopeFeedbackParam.startPosition = pRequest->startPosition;
        command.commandData.slopeFeedbackParam.endPosition = pRequest->endPosition;
        command.commandData.slopeFeedbackParam.startStrength = pRequest->startStrength;
        command.commandData.slopeFeedbackParam.endStrength = pRequest->endStrength;
        SetTriggerEffectCommand(processId, pRequest->controllerType, command);
      }
      break;
    }
    case ipc::Command_ClientTriggerEffectMultiplePositionVibration: {
      if (pHeader->dataLen == sizeof(ipc::CommandDataClientTriggerEffectMultiplePositionVibration_t)) {
        ipc::CommandDataClientTriggerEffectMultiplePositionVibration_t* pRequest = reinterpret_cast<ipc::CommandDataClientTriggerEffectMultiplePositionVibration_t*>(pData);
        command.mode = SCE_PAD_TRIGGER_EFFECT_MODE_MULTIPLE_POSITION_FEEDBACK;
        command.commandData.multiplePositionVibrationParam.frequency = pRequest->frequency;
        for (int i = 0; i < ipc::k_unTriggerEffectControlPoint; i++) {
          command.commandData.multiplePositionVibrationParam.amplitude[i] = pRequest->amplitude[i];
        }
        SetTriggerEffectCommand(processId, pRequest->controllerType, command);
      }
      break;
    }
    }
  }

  void TriggerEffectManager::SetTriggerEffectCommand(uint32_t processId, ipc::EVRControllerType controllerType, ScePadTriggerEffectCommand command) {
    static AstonManager_t* pAstonManager = getAstonManager();

    (void)processId;

    ScePadTriggerEffectParam param = {};
    switch (controllerType) {
    case ipc::VRController_Left: {
      param.triggerMask = SCE_PAD_TRIGGER_EFFECT_TRIGGER_MASK_L2;
      param.command[SCE_PAD_TRIGGER_EFFECT_PARAM_INDEX_FOR_L2] = command;
      break;
    }
    case ipc::VRController_Right: {
      param.triggerMask = SCE_PAD_TRIGGER_EFFECT_TRIGGER_MASK_R2;
      param.command[SCE_PAD_TRIGGER_EFFECT_PARAM_INDEX_FOR_R2] = command;
      break;
    }
    case ipc::VRController_Both: {
      param.triggerMask = SCE_PAD_TRIGGER_EFFECT_TRIGGER_MASK_L2 | SCE_PAD_TRIGGER_EFFECT_TRIGGER_MASK_R2;
      param.command[SCE_PAD_TRIGGER_EFFECT_PARAM_INDEX_FOR_L2] = command;
      param.command[SCE_PAD_TRIGGER_EFFECT_PARAM_INDEX_FOR_R2] = command;
      break;
    }
    }

    if (pAstonManager) {
      if (controllerType == ipc::VRController_Left || controllerType == ipc::VRController_Both) {
        int leftPadHandle = pAstonManager->contexts[1]->handle;
        if (leftPadHandle > -1) {
          scePadSetTriggerEffect(leftPadHandle, &param);
        }
      }
      if (controllerType == ipc::VRController_Right || controllerType == ipc::VRController_Both) {
        int rightPadHandle = pAstonManager->contexts[0]->handle;
        if (rightPadHandle > -1) {
          scePadSetTriggerEffect(rightPadHandle, &param);
        }
      }
    }
  }

} // psvr2_toolkit
