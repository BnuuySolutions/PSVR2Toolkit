#pragma once

#include <stdint.h>

typedef enum hmd2_gaze_bool_t : uint32_t {
  HMD2_GAZE_BOOL_FALSE,
  HMD2_GAZE_BOOL_TRUE
} hmd2_gaze_bool_t;

typedef enum hmd2_gaze_enabled_eye_t : uint8_t {
  HMD2_GAZE_ENABLED_EYE_LEFT,
  HMD2_GAZE_ENABLED_EYE_RIGHT,
  HMD2_GAZE_ENABLED_EYE_BOTH
} hmd2_gaze_enabled_eye_t;

typedef struct hmd2_gaze_vec2_t {
  float x, y;
} hmd2_gaze_vec2_t;

typedef struct hmd2_gaze_vec3_t {
  float x, y, z;
} hmd2_gaze_vec3_t;

typedef struct hmd2_gaze_lens_config_t {
  hmd2_gaze_vec3_t left;
  hmd2_gaze_vec3_t right;
} hmd2_gaze_lens_config_t;

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

typedef struct hmd2_gaze_wearable_data_t {
  int64_t timestamp;
  uint32_t frame_counter;

  hmd2_gaze_wearable_eye_t left;
  hmd2_gaze_wearable_eye_t right;

  hmd2_gaze_bool_t is_gaze_origin_combined_valid;
  hmd2_gaze_vec3_t gaze_origin_combined_mm;

  hmd2_gaze_bool_t is_gaze_dir_combined_valid;
  hmd2_gaze_vec3_t gaze_dir_combined_norm;

  hmd2_gaze_bool_t is_convergence_distance_valid;
} hmd2_gaze_wearable_data_t;

typedef struct hmd2_gaze_foveated_gaze_t {
  int64_t timestamp;
  uint32_t frame_counter;
  uint32_t tracking_state;

  hmd2_gaze_vec3_t gaze_dir_left_norm;
  hmd2_gaze_vec3_t gaze_dir_right_norm;
  hmd2_gaze_vec3_t gaze_dir_combined_norm;

  float convergence_distance_mm; // I don't know it's like this, but this is correct.
} hmd2_gaze_foveated_gaze_t;

typedef struct hmd2_gaze_status_t {
  uint8_t magic[2];
  uint16_t version;
  uint32_t size;

  // Exposure
  float exp_l;
  float exp_r;

  uint32_t led_status;

  uint32_t exp_counter_l;
  uint32_t exp_counter_r;

  uint32_t led_counter;

  int dsp_return_code;

  hmd2_gaze_lens_config_t lens_config;

  uint32_t user_calibration_id;

  hmd2_gaze_vec3_t fr_gaze_origin;

  hmd2_gaze_enabled_eye_t enabled_eye;

  uint8_t motor_sequence;
  uint8_t motor_strength;

  hmd2_gaze_wearable_data_t wearable;
  hmd2_gaze_foveated_gaze_t foveated;
} hmd2_gaze_status_t;
static_assert(sizeof(hmd2_gaze_status_t) == 0x148, "Size of hmd2_gaze_status_t is not 0x148 bytes!");
