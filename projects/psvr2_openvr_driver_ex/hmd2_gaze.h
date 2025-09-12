#pragma once

#include <cstdint>

typedef enum {
  HMD2_BOOL_FALSE = 0,
  HMD2_BOOL_TRUE = 1
} Hmd2Bool;

typedef struct {
  float x, y;
} Hmd2Vector2;

typedef struct {
  float x, y, z;
} Hmd2Vector3;

typedef struct {
  Hmd2Bool isGazeOriginValid;
  Hmd2Vector3 gazeOriginMm;

  Hmd2Bool isGazeDirValid;
  Hmd2Vector3 gazeDirNorm;

  Hmd2Bool isPupilDiaValid;
  float pupilDiaMm;

  // No clue what this extra stuff is.
  Hmd2Bool unk06;
  Hmd2Vector2 unk07;
  Hmd2Bool unk08;
  Hmd2Vector2 unk09;

  Hmd2Bool isBlinkValid;
  Hmd2Bool blink;
} Hmd2GazeEye;

struct Hmd2GazeCombined {
    Hmd2Bool isGazeOriginValid;
    Hmd2Vector3 gazeOriginMm;

    Hmd2Bool isGazeDirValid;
    Hmd2Vector3 gazeDirNorm;

    Hmd2Bool isValid;
    uint32_t timestamp;

    Hmd2Bool unk06;
    float unk07;

    Hmd2Bool unk08;

    // All gazes seem to be repeated here?
    // No obvious differences here.
    Hmd2Vector3 leftGazeDirNormal;
    Hmd2Vector3 rightGazeDirNormal;
    Hmd2Vector3 combinedGazeDirNormal;

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

  Hmd2GazeEye leftEye;
  Hmd2GazeEye rightEye;
  Hmd2GazeCombined combined;
};
