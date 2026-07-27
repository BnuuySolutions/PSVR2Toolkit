#pragma once

#include "common.h"
#include "custom_share_manager.h"
#include "pad_trigger_effect.h"

namespace psvr2_toolkit {

class TriggerEffectManager {
public:
  TriggerEffectManager();

  static TriggerEffectManager *Instance();

  bool IsInitialized();
  void Initialize();

  void SetSlotEffect(int slot, const TriggerEffectCommandPayload &payload);
  void Update();

private:
  static psvr2_toolkit::TriggerEffectManager *m_pInstance;

  bool m_initialized;

  TriggerEffectCommandPayload m_slotEffects[k_maxSlots][2];
  uint64_t m_slotSequence[k_maxSlots][2];
  bool m_slotAlive[k_maxSlots];
  uint64_t m_sequenceCounter;
  TriggerEffectCommandPayload m_lastLeft;
  TriggerEffectCommandPayload m_lastRight;

  int (*m_scePadSetTriggerEffect)(int handle, ScePadTriggerEffectParam *param);

  void SetTriggerEffectCommand(VRControllerType controllerType, ScePadTriggerEffectCommand command);
};

} // namespace psvr2_toolkit
