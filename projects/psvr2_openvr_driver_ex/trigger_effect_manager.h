#pragma once

#include "common.h"
#include "pad_trigger_effect.h"

namespace psvr2_toolkit {

  class TriggerEffectManager {
  public:
    TriggerEffectManager();

    static TriggerEffectManager *Instance();

    bool IsInitialized();
    void Initialize();

    void Update();

  private:
    static psvr2_toolkit::TriggerEffectManager *m_pInstance;

    bool m_initialized;

    void SetTriggerEffectCommand(VRControllerType controllerType, ScePadTriggerEffectCommand command);
  };

} // psvr2_toolkit
