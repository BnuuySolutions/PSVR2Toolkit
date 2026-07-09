#include "trigger_effect_manager.h"
#include "hmd_driver_loader.h"
#include "custom_share_manager.h"

#include <cstring>

#include "driver_interface/aston_manager.h"

namespace psvr2_toolkit {

  // State tracking for the 4 slots
  static TriggerEffectCommandPayload g_slotEffects[k_maxSlots][2];
  static bool g_slotAlive[k_maxSlots];
  static TriggerEffectCommandPayload g_lastLeft = { VRControllerType::Left, {} };
  static TriggerEffectCommandPayload g_lastRight = { VRControllerType::Right, {} };

  int (*scePadSetTriggerEffect)(int handle, ScePadTriggerEffectParam *param);

  TriggerEffectManager *TriggerEffectManager::m_pInstance = nullptr;

  TriggerEffectManager::TriggerEffectManager()
    : m_initialized(false)
  {}

  TriggerEffectManager *TriggerEffectManager::Instance() {
    if (!m_pInstance) {
      m_pInstance = new TriggerEffectManager;
    }

    return m_pInstance;
  }

  bool TriggerEffectManager::IsInitialized() {
    return m_initialized;
  }

  void TriggerEffectManager::Initialize() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    if (m_initialized) {
      return;
    }

    scePadSetTriggerEffect = decltype(scePadSetTriggerEffect)(pHmdDriverLoader->GetBaseAddress() + 0x1BF060);

    m_initialized = true;
  }

  void TriggerEffectManager::Update() {
    CustomShareManager* pShareManager = CustomShareManager::getSingleton();
    if (!pShareManager) return;

    TriggerEffectCommand cmd;
    while (pShareManager->popTriggerEffect(cmd)) {
      if (cmd.slot >= 0 && cmd.slot < k_maxSlots) {
        if (cmd.payload.controllerType == VRControllerType::Left || cmd.payload.controllerType == VRControllerType::Both) {
          g_slotEffects[cmd.slot][0] = cmd.payload;
          g_slotEffects[cmd.slot][0].controllerType = VRControllerType::Left;
        }
        if (cmd.payload.controllerType == VRControllerType::Right || cmd.payload.controllerType == VRControllerType::Both) {
          g_slotEffects[cmd.slot][1] = cmd.payload;
          g_slotEffects[cmd.slot][1].controllerType = VRControllerType::Right;
        }
      }
    }

    for (int i = 0; i < k_maxSlots; i++) {
      bool alive = pShareManager->isSlotAlive(i);
      if (g_slotAlive[i] && !alive) {
        g_slotEffects[i][0].command.mode = ScePadTriggerEffectMode::SCE_PAD_TRIGGER_EFFECT_MODE_OFF;
        g_slotEffects[i][1].command.mode = ScePadTriggerEffectMode::SCE_PAD_TRIGGER_EFFECT_MODE_OFF;
      }
      g_slotAlive[i] = alive;
    }

    TriggerEffectCommandPayload finalLeft = {};
    finalLeft.controllerType = VRControllerType::Left;
    TriggerEffectCommandPayload finalRight = {};
    finalRight.controllerType = VRControllerType::Right;

    for (int i = k_maxSlots - 1; i >= 0; i--) {
      if (g_slotAlive[i]) {
        if (g_slotEffects[i][0].command.mode != ScePadTriggerEffectMode::SCE_PAD_TRIGGER_EFFECT_MODE_OFF) finalLeft = g_slotEffects[i][0];
        if (g_slotEffects[i][1].command.mode != ScePadTriggerEffectMode::SCE_PAD_TRIGGER_EFFECT_MODE_OFF) finalRight = g_slotEffects[i][1];
      }
    }

    if (std::memcmp(&finalLeft, &g_lastLeft, sizeof(TriggerEffectCommandPayload)) != 0) {
      SetTriggerEffectCommand(finalLeft.controllerType, finalLeft.command);
      g_lastLeft = finalLeft;
    }
    if (std::memcmp(&finalRight, &g_lastRight, sizeof(TriggerEffectCommandPayload)) != 0) {
      SetTriggerEffectCommand(finalRight.controllerType, finalRight.command);
      g_lastRight = finalRight;
    }
  }

  void TriggerEffectManager::SetTriggerEffectCommand(VRControllerType controllerType, ScePadTriggerEffectCommand command) {
    AstonManager *pAstonManager = AstonManager::getSingleton();

    ScePadTriggerEffectParam param = {};
    switch (controllerType) {
      case VRControllerType::Left: {
        param.triggerMask = SCE_PAD_TRIGGER_EFFECT_TRIGGER_MASK_L2;
        param.command[SCE_PAD_TRIGGER_EFFECT_PARAM_INDEX_FOR_L2] = command;
        break;
      }
      case VRControllerType::Right: {
        param.triggerMask = SCE_PAD_TRIGGER_EFFECT_TRIGGER_MASK_R2;
        param.command[SCE_PAD_TRIGGER_EFFECT_PARAM_INDEX_FOR_R2] = command;
        break;
      }
      case VRControllerType::Both: {
        param.triggerMask = SCE_PAD_TRIGGER_EFFECT_TRIGGER_MASK_L2 | SCE_PAD_TRIGGER_EFFECT_TRIGGER_MASK_R2;
        param.command[SCE_PAD_TRIGGER_EFFECT_PARAM_INDEX_FOR_L2] = command;
        param.command[SCE_PAD_TRIGGER_EFFECT_PARAM_INDEX_FOR_R2] = command;
        break;
      }
    }

    if (pAstonManager) {
      if (controllerType == VRControllerType::Left || controllerType == VRControllerType::Both) {
        if (pAstonManager->contexts[1]) {
          int leftPadHandle = pAstonManager->contexts[1]->handle;
          if (leftPadHandle > -1) {
            scePadSetTriggerEffect(leftPadHandle, &param);
          }
        }
      }
      if (controllerType == VRControllerType::Right || controllerType == VRControllerType::Both) {
        if (pAstonManager->contexts[0]) {
          int rightPadHandle = pAstonManager->contexts[0]->handle;
          if (rightPadHandle > -1) {
            scePadSetTriggerEffect(rightPadHandle, &param);
          }
        }
      }
    }
  }

} // psvr2_toolkit
