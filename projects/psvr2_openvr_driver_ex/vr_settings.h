#pragma once

#include <openvr_driver.h>

#define STEAMVR_SETTINGS_SECTION_PLAYSTATION_VR2_EX "playstation_vr2_ex"

#define STEAMVR_SETTINGS_ENABLE_EYELID_ESTIMATION "enableEyelidEstimation"
#define STEAMVR_SETTINGS_DISABLE_CHAPERONE "disableChaperone"
#define STEAMVR_SETTINGS_DISABLE_OVERLAY "disableOverlay"
#define STEAMVR_SETTINGS_DISABLE_DIALOG "disableDialog"
#define STEAMVR_SETTINGS_DISABLE_SENSE "disableSense"
#define STEAMVR_SETTINGS_DISABLE_GAZE "disableGaze"

#define SETTING_ENABLE_EYELID_ESTIMATION_DEFAULT_VALUE false
#define SETTING_DISABLE_CHAPERONE_DEFAULT_VALUE false
#define SETTING_DISABLE_OVERLAY_DEFAULT_VALUE false
#define SETTING_DISABLE_DIALOG_DEFAULT_VALUE false
#define SETTING_DISABLE_SENSE_DEFAULT_VALUE false
#define SETTING_DISABLE_GAZE_DEFAULT_VALUE false

namespace psvr2_toolkit {

  class VRSettings {
  public:
    static bool GetBool(const char *pchSettingsKey, bool defaultValue) {
      vr::EVRSettingsError error;
      bool value = vr::VRSettings()->GetBool(STEAMVR_SETTINGS_SECTION_PLAYSTATION_VR2_EX, pchSettingsKey, &error);
      if (error != vr::EVRSettingsError::VRSettingsError_None) {
        value = defaultValue;
      }
      return value;
    }

    static int GetInt32(const char *pchSettingsKey, int defaultValue) {
      vr::EVRSettingsError error;
      int value = vr::VRSettings()->GetInt32(STEAMVR_SETTINGS_SECTION_PLAYSTATION_VR2_EX, pchSettingsKey, &error);
      if (error != vr::EVRSettingsError::VRSettingsError_None) {
        value = defaultValue;
      }
      return value;
    }

  };

} // psvr2_toolkit
