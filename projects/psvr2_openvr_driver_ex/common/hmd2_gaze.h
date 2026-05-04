#pragma once

#include <cstdint>

typedef enum hmd2_gaze_bool_t {
  HMD2_GAZE_BOOL_FALSE,
  HMD2_GAZE_BOOL_TRUE
} hmd2_gaze_bool_t;

typedef struct hmd2_gaze_vec2_t {
  float x, y;
} hmd2_gaze_vec2_t;

typedef struct hmd2_gaze_vec3_t {
  float x, y, z;
} hmd2_gaze_vec3_t;

typedef struct hmd2_gaze_wearable_eye_t {
  hmd2_gaze_bool_t is_gaze_origin_valid;
  hmd2_gaze_vec3_t gaze_origin_mm;

  hmd2_gaze_bool_t is_gaze_dir_valid;
  hmd2_gaze_vec3_t gaze_dir_norm;

  hmd2_gaze_bool_t is_pupil_dia_valid;
  float pupil_dia_mm;

  hmd2_gaze_bool_t is_pupil_pos_in_sensor_area_valid;
  hmd2_gaze_vec2_t pupil_pos_in_sensor_area;

  hmd2_gaze_bool_t is_pos_guide_valid;
  hmd2_gaze_vec2_t pos_guide;

  hmd2_gaze_bool_t is_blink_valid;
  hmd2_gaze_bool_t blink;
} hmd2_gaze_wearable_eye_t;

struct Hmd2GazeCombined {
  hmd2_gaze_bool_t isGazeOriginValid;
  hmd2_gaze_vec3_t gazeOriginMm;

  hmd2_gaze_bool_t isGazeDirValid;
  hmd2_gaze_vec3_t gazeDirNorm;

  hmd2_gaze_bool_t isValid;
  uint32_t timestamp;

  hmd2_gaze_bool_t unk06;
  float unk07;

  hmd2_gaze_bool_t unk08;

  // All gazes seem to be repeated here?
  // No obvious differences here.
  hmd2_gaze_vec3_t leftGazeDirNormal;
  hmd2_gaze_vec3_t rightGazeDirNormal;
  hmd2_gaze_vec3_t combinedGazeDirNormal;

  float unk09;
};

struct Hmd2GazeState {
  char magic[2];
  uint16_t version;
  uint32_t size;

  float unk03; // 0.700
  float unk04; // 0.700

  uint32_t unk05; // 0xFFFFF

  // Appears to be related to some sort of timestamp?
  uint32_t timestamp06;
  uint32_t timestamp07;
  uint32_t timestamp08;

  uint32_t unk09;

  float leftEyeXPosMm; // Example: -32.000 (64mm IPD)

  uint32_t unk11;
  uint32_t unk12;

  float rightEyeXPosMm; // Example: 32.000 (64mm IPD)

  uint32_t unk14;
  uint32_t unk15;
  uint32_t unk16;
  uint32_t unk17;
  uint32_t unk18;

  float eyeZPosMm; // -27.000

  // More unknown garbage.
  uint32_t unk20;
  uint32_t timestamp21;
  uint32_t unk22;
  uint32_t timestamp23;

  hmd2_gaze_wearable_eye_t leftEye;
  hmd2_gaze_wearable_eye_t rightEye;
  Hmd2GazeCombined combined;
};
