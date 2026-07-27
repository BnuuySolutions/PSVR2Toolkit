#include "trigger_effect_manager.h"
#include "hmd_driver_loader.h"
#include "custom_share_manager.h"

#include <cstring>

#include "driver_interface/aston_manager.h"

namespace psvr2_toolkit {

TriggerEffectManager *TriggerEffectManager::m_pInstance = nullptr;

TriggerEffectManager::TriggerEffectManager()
    : m_initialized(false), m_sequenceCounter(0), m_lastLeft{VRControllerType::Left, {}}, m_lastRight{VRControllerType::Right, {}},
      m_scePadSetTriggerEffect(nullptr) {
  std::memset(m_slotEffects, 0, sizeof(m_slotEffects));
  std::memset(m_slotSequence, 0, sizeof(m_slotSequence));
  std::memset(m_slotAlive, 0, sizeof(m_slotAlive));
}

TriggerEffectManager *TriggerEffectManager::Instance() {
  if (!m_pInstance) {
    m_pInstance = new TriggerEffectManager;
  }

  return m_pInstance;
}

bool TriggerEffectManager::IsInitialized() { return m_initialized; }

void TriggerEffectManager::Initialize() {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

  if (m_initialized) {
    return;
  }

  m_scePadSetTriggerEffect = decltype(m_scePadSetTriggerEffect)(pHmdDriverLoader->GetBaseAddress() + 0x1BF060);

  m_initialized = true;
}

void TriggerEffectManager::SetSlotEffect(int slot, const TriggerEffectCommandPayload &payload) {
  if (slot >= 0 && slot < k_maxSlots) {
    m_sequenceCounter++;
    if (payload.controllerType == VRControllerType::Left || payload.controllerType == VRControllerType::Both) {
      m_slotEffects[slot][0] = payload;
      m_slotEffects[slot][0].controllerType = VRControllerType::Left;
      m_slotSequence[slot][0] = m_sequenceCounter;
    }
    if (payload.controllerType == VRControllerType::Right || payload.controllerType == VRControllerType::Both) {
      m_slotEffects[slot][1] = payload;
      m_slotEffects[slot][1].controllerType = VRControllerType::Right;
      m_slotSequence[slot][1] = m_sequenceCounter;
    }
  }

  Update();
}

void TriggerEffectManager::Update() {
  CustomShareManager *pShareManager = CustomShareManager::getSingleton();
  if (!pShareManager)
    return;

  for (int i = 0; i < k_maxSlots; i++) {
    bool alive = pShareManager->isSlotAlive(i);
    if (m_slotAlive[i] && !alive) {
      m_slotEffects[i][0].command.mode = ScePadTriggerEffectMode::SCE_PAD_TRIGGER_EFFECT_MODE_OFF;
      m_slotEffects[i][1].command.mode = ScePadTriggerEffectMode::SCE_PAD_TRIGGER_EFFECT_MODE_OFF;
      m_slotSequence[i][0] = 0;
      m_slotSequence[i][1] = 0;
    }
    m_slotAlive[i] = alive;
  }

  TriggerEffectCommandPayload finalLeft = {};
  finalLeft.controllerType = VRControllerType::Left;
  TriggerEffectCommandPayload finalRight = {};
  finalRight.controllerType = VRControllerType::Right;

  uint64_t highestLeftSeq = 0;
  uint64_t highestRightSeq = 0;

  for (int i = 0; i < k_maxSlots; i++) {
    if (m_slotAlive[i]) {
      if (m_slotEffects[i][0].command.mode != ScePadTriggerEffectMode::SCE_PAD_TRIGGER_EFFECT_MODE_OFF) {
        if (m_slotSequence[i][0] > highestLeftSeq) {
          highestLeftSeq = m_slotSequence[i][0];
          finalLeft = m_slotEffects[i][0];
        }
      }
      if (m_slotEffects[i][1].command.mode != ScePadTriggerEffectMode::SCE_PAD_TRIGGER_EFFECT_MODE_OFF) {
        if (m_slotSequence[i][1] > highestRightSeq) {
          highestRightSeq = m_slotSequence[i][1];
          finalRight = m_slotEffects[i][1];
        }
      }
    }
  }

  if (std::memcmp(&finalLeft, &m_lastLeft, sizeof(TriggerEffectCommandPayload)) != 0) {
    SetTriggerEffectCommand(finalLeft.controllerType, finalLeft.command);
    m_lastLeft = finalLeft;
  }
  if (std::memcmp(&finalRight, &m_lastRight, sizeof(TriggerEffectCommandPayload)) != 0) {
    SetTriggerEffectCommand(finalRight.controllerType, finalRight.command);
    m_lastRight = finalRight;
  }
}

void TriggerEffectManager::SetTriggerEffectCommand(VRControllerType controllerType, ScePadTriggerEffectCommand command) {
  AstonManager *pAstonManager = AstonManager::getSingleton();

  if (!m_scePadSetTriggerEffect)
    return;

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
          m_scePadSetTriggerEffect(leftPadHandle, &param);
        }
      }
    }
    if (controllerType == VRControllerType::Right || controllerType == VRControllerType::Both) {
      if (pAstonManager->contexts[0]) {
        int rightPadHandle = pAstonManager->contexts[0]->handle;
        if (rightPadHandle > -1) {
          m_scePadSetTriggerEffect(rightPadHandle, &param);
        }
      }
    }
  }
}

} // namespace psvr2_toolkit
