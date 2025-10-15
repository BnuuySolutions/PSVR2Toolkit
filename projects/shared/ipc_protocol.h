#pragma once

#include <cstdint>

#define IPC_SERVER_PORT 3364

namespace psvr2_toolkit {
  namespace ipc {

    static constexpr uint16_t k_unIpcVersion = 2;
    static constexpr uint32_t k_unTriggerEffectControlPoint = 10;

    enum ECommandType : uint16_t {
      Command_ClientPing, // No command data.
      Command_ServerPong, // No command data.

      Command_ClientRequestHandshake, // CommandDataClientRequestHandshake_t
      Command_ServerHandshakeResult, // CommandDataServerHandshakeResult_t

      Command_ClientRequestGazeData, // No command data.
      Command_ServerGazeDataResult, // CommandDataServerGazeDataResult2_t (or if IPC version is 1, CommandDataServerGazeDataResult_t)

      Command_ClientTriggerEffectOff, // CommandDataClientTriggerEffectOff_t
      Command_ClientTriggerEffectFeedback, // CommandDataClientTriggerEffectFeedback_t
      Command_ClientTriggerEffectWeapon, // CommandDataClientTriggerEffectWeapon_t
      Command_ClientTriggerEffectVibration, // CommandDataClientTriggerEffectVibration_t
      Command_ClientTriggerEffectMultiplePositionFeedback, // CommandDataClientTriggerEffectMultiplePositionFeedback_t
      Command_ClientTriggerEffectSlopeFeedback, // CommandDataClientTriggerEffectSlopeFeedback_t
      Command_ClientTriggerEffectMultiplePositionVibration, // CommandDataClientTriggerEffectMultiplePositionVibration_t
    };

    enum EHandshakeResultType : uint8_t {
      HandshakeResult_Failed,
      HandshakeResult_Success,
      HandshakeResult_Outdated,
    };

    enum EVRControllerType : uint8_t {
      VRController_Left,
      VRController_Right,
      VRController_Both,
    };

    struct CommandDataClientRequestHandshake_t {
      uint16_t ipcVersion; // The IPC version this client is using.
      uint32_t processId;
    };

    struct CommandDataServerHandshakeResult_t {
      EHandshakeResultType result;
      uint16_t ipcVersion; // The IPC version the server is using.
    };

    #pragma pack(push, 1)
    // for bool marshaling; force 1 byte wide bools
    struct GazeVector3 {
      float x, y, z;
    };

    struct GazeEyeResult {
      bool isGazeOriginValid;
      GazeVector3 gazeOriginMm;

      bool isGazeDirValid;
      GazeVector3 gazeDirNorm;

      bool isPupilDiaValid;
      float pupilDiaMm;

      bool isBlinkValid;
      bool blink;
    };

    struct GazeEyeResult2 {
      bool isGazeOriginValid;
      GazeVector3 gazeOriginMm;

      bool isGazeDirValid;
      GazeVector3 gazeDirNorm;

      bool isPupilDiaValid;
      float pupilDiaMm;

      bool isBlinkValid;
      bool blink;

      bool isOpenEnabled;
      float open;
    };

    struct CommandDataServerGazeDataResult_t {
      // TODO: Include timestamp, etc.

      GazeEyeResult leftEye;
      GazeEyeResult rightEye;
    };

    struct CommandDataServerGazeDataResult2_t {
      // TODO: Include timestamp, etc.

      GazeEyeResult2 leftEye;
      GazeEyeResult2 rightEye;
    };
    #pragma pack(pop)

    struct CommandDataClientTriggerEffectOff_t {
      EVRControllerType controllerType;
    };

    struct CommandDataClientTriggerEffectFeedback_t {
      EVRControllerType controllerType;
      uint8_t position;
      uint8_t strength;
    };

    struct CommandDataClientTriggerEffectWeapon_t {
      EVRControllerType controllerType;
      uint8_t startPosition;
      uint8_t endPosition;
      uint8_t strength;
    };

    struct CommandDataClientTriggerEffectVibration_t {
      EVRControllerType controllerType;
      uint8_t position;
      uint8_t amplitude;
      uint8_t frequency;
    };

    struct CommandDataClientTriggerEffectMultiplePositionFeedback_t {
      EVRControllerType controllerType;
      uint8_t strength[k_unTriggerEffectControlPoint];
    };

    struct CommandDataClientTriggerEffectSlopeFeedback_t {
      EVRControllerType controllerType;
      uint8_t startPosition;
      uint8_t endPosition;
      uint8_t startStrength;
      uint8_t endStrength;
    };

    struct CommandDataClientTriggerEffectMultiplePositionVibration_t {
      EVRControllerType controllerType;
      uint8_t frequency;
      uint8_t amplitude[k_unTriggerEffectControlPoint];
    };

    struct CommandHeader_t {
      ECommandType type;
      int32_t dataLen;
    };

  } // ipc
} // psvr2_toolkit
