#pragma once

#include <openvr_driver.h>

#define STEAMVR_SETTINGS_SECTION_PLAYSTATION_VR2_EX "playstation_vr2_ex"

#define STEAMVR_SETTINGS_DISABLE_CHAPERONE "disableChaperone"
#define STEAMVR_SETTINGS_DISABLE_SENSE "disableSense"
#define STEAMVR_SETTINGS_DISABLE_GAZE "disableGaze"
#define STEAMVR_SETTINGS_USE_TOOLKIT_SYNC "useToolkitSync"
#define STEAMVR_SETTINGS_USE_ENHANCED_HAPTICS "useEnhancedHaptics"
#define STEAMVR_SETTINGS_DISABLE_HIDDEN_AREA_MESH "disableHiddenAreaMesh"

// Eye tracking. See EYE_TRACKING_PLAN.md 2.4.
#define STEAMVR_SETTINGS_GAZE_DEFAULT_FIXATION_DISTANCE_M "gazeDefaultFixationDistanceM"
#define STEAMVR_SETTINGS_GAZE_FILTER_ENABLED "gazeFilterEnabled"
#define STEAMVR_SETTINGS_GAZE_FILTER_MIN_CUTOFF "gazeFilterMinCutoff"
#define STEAMVR_SETTINGS_GAZE_FILTER_BETA "gazeFilterBeta"
#define STEAMVR_SETTINGS_GAZE_FILTER_SACCADE_DEG_PER_SEC "gazeFilterSaccadeDegPerSec"
#define STEAMVR_SETTINGS_GAZE_BLINK_HOLD_MS "gazeBlinkHoldMs"
#define STEAMVR_SETTINGS_GAZE_FIT_GUIDANCE_ENABLED "gazeFitGuidanceEnabled"
#define STEAMVR_SETTINGS_GAZE_IMAGE_STREAM_ENABLED "gazeImageStreamEnabled"

#define SETTING_DISABLE_CHAPERONE_DEFAULT_VALUE false
#define SETTING_DISABLE_SENSE_DEFAULT_VALUE false
#define SETTING_DISABLE_GAZE_DEFAULT_VALUE false
#define SETTING_USE_TOOLKIT_SYNC_DEFAULT_VALUE true
#define SETTING_USE_ENHANCED_HAPTICS_DEFAULT_VALUE true
#define SETTING_DISABLE_HIDDEN_AREA_MESH_DEFAULT_VALUE false

// Fallback fixation distance when the headset reports no usable convergence distance.
#define SETTING_GAZE_DEFAULT_FIXATION_DISTANCE_M_DEFAULT_VALUE 2.0f
#define SETTING_GAZE_FILTER_ENABLED_DEFAULT_VALUE true
// One-euro filter tuning. Raising minCutoff reduces lag at the cost of jitter; beta is expressed in Hz
// of cutoff per deg/s of gaze velocity, so raising it opens the filter up sooner as the eye moves.
// The saccade threshold must stay above the tracker's noise floor -- see gaze_filter.h.
#define SETTING_GAZE_FILTER_MIN_CUTOFF_DEFAULT_VALUE 1.0f
#define SETTING_GAZE_FILTER_BETA_DEFAULT_VALUE 0.12f
#define SETTING_GAZE_FILTER_SACCADE_DEG_PER_SEC_DEFAULT_VALUE 120.0f
#define SETTING_GAZE_BLINK_HOLD_MS_DEFAULT_VALUE 200
#define SETTING_GAZE_FIT_GUIDANCE_ENABLED_DEFAULT_VALUE true
#define SETTING_GAZE_IMAGE_STREAM_ENABLED_DEFAULT_VALUE true

namespace psvr2_toolkit {

class DriverSettings {
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

  static float GetFloat(const char *pchSettingsKey, float defaultValue) {
    vr::EVRSettingsError error;
    float value = vr::VRSettings()->GetFloat(STEAMVR_SETTINGS_SECTION_PLAYSTATION_VR2_EX, pchSettingsKey, &error);
    if (error != vr::EVRSettingsError::VRSettingsError_None) {
      value = defaultValue;
    }
    return value;
  }

  // Single source of truth for whether the gaze pipeline is running. The hooks that feed it and the
  // properties that advertise it are installed in different translation units, and they must agree:
  // advertising eye tracking that never updates is worse for applications than reporting none.
  static bool IsGazeEnabled() { return !GetBool(STEAMVR_SETTINGS_DISABLE_GAZE, SETTING_DISABLE_GAZE_DEFAULT_VALUE); }
};

} // namespace psvr2_toolkit
