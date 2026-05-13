#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#endif

#include <iostream>
#include <thread>
#include <map>
#include <atomic>
#include <cstdint>
#include <mutex>

#include "psvr2tk_capi_loader.h"
#include "pad_trigger_effect.h"
#include "ipc_protocol.h"

#ifndef _WIN32
typedef int SOCKET;
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR (-1)
#define closesocket close
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
#endif

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

inline int GetSocketError() {
#ifdef _WIN32
    return WSAGetLastError();
#else
    return errno;
#endif
}

namespace psvr2_toolkit {
  namespace ipc {
    struct ConnectionData {
      sockaddr_in clientAddr;
      uint16_t ipcVersion;
      uint32_t processId;
    };

    class IpcServer {
    public:
      static IpcServer* Instance() {
        if (!m_pInstance) {
          m_pInstance = new IpcServer;
        }
        return m_pInstance;
      }

      bool Initialized() { return m_initialized; }

      void Initialize() {
        if (m_initialized) return;

#ifdef _WIN32
        WSADATA wsaData = {};
        int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (result != 0) {
          printf("[IPC_SERVER] WSAStartup failed. Result = %d\n", result);
          return;
        }
#endif

        m_initialized = true;
        m_doGaze = true;
        m_doOpenness = true;
      }

      void Start() {
        if (!m_initialized && m_running) return;

        m_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (m_socket == INVALID_SOCKET) {
          printf("[IPC_SERVER] Creating socket failed. LastError = %d\n", GetSocketError());
          return;
        }

        m_serverAddr.sin_family = AF_INET;
        m_serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        m_serverAddr.sin_port = htons(IPC_SERVER_PORT);

#ifndef _WIN32
        int opt = 1;
        setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif

        if (bind(m_socket, reinterpret_cast<SOCKADDR*>(&m_serverAddr), sizeof(m_serverAddr)) == SOCKET_ERROR) {
          printf("[IPC_SERVER] Bind failed. LastError = %d\n", GetSocketError());
          closesocket(m_socket);
          return;
        }

        if (listen(m_socket, SOMAXCONN) == SOCKET_ERROR) {
          printf("[IPC_SERVER] Listen failed. LastError = %d\n", GetSocketError());
          closesocket(m_socket);
          return;
        }

        m_running = true;
        m_receiveThread = std::thread(&IpcServer::ReceiveLoop, this);
        m_gazeThread = std::thread(&IpcServer::GazePollLoop, this);
      }

      void Stop() {
        if (!m_running) return;

        m_running = false;
        closesocket(m_socket);
        if (m_receiveThread.joinable()) m_receiveThread.join();
        if (m_gazeThread.joinable()) m_gazeThread.join();
#ifdef _WIN32
        WSACleanup();
#endif
      }

    private:
      IpcServer() : m_initialized(false), m_running(false), m_doGaze(false), m_doOpenness(false), m_socket{}, m_serverAddr{} {}

      static IpcServer* m_pInstance;

      bool m_initialized;
      std::atomic<bool> m_running;
      bool m_doGaze;
      bool m_doOpenness;

      SOCKET m_socket;
      sockaddr_in m_serverAddr;
      std::thread m_receiveThread;
      std::thread m_gazeThread;

      std::map<uint16_t, ConnectionData> m_connections;
      std::mutex m_connectionsMutex;

      hmd2_gaze_status_t m_gazeStatus = {};
      std::atomic<float> m_leftEyelidOpenness = 1.0f;
      std::atomic<float> m_rightEyelidOpenness = 1.0f;

      void GazePollLoop() {
        while (m_running) {
          psvr2_toolkit_gaze_status(&m_gazeStatus, 1000);
          
          m_leftEyelidOpenness = (m_gazeStatus.wearable.left.blink == HMD2_GAZE_BOOL_TRUE) ? 0.0f : 1.0f;
          m_rightEyelidOpenness = (m_gazeStatus.wearable.right.blink == HMD2_GAZE_BOOL_TRUE) ? 0.0f : 1.0f;
        }
      }

      void ReceiveLoop() {
        while (m_running) {
          SOCKADDR_IN clientAddr = {};
#ifdef _WIN32
          int clientAddrLen = sizeof(clientAddr);
#else
          socklen_t clientAddrLen = sizeof(clientAddr);
#endif

          SOCKET clientSocket = accept(m_socket, reinterpret_cast<SOCKADDR*>(&clientAddr), &clientAddrLen);
          if (clientSocket == INVALID_SOCKET) {
            int error = GetSocketError();
            if (m_running) {
                printf("[IPC_SERVER] Accept failed. LastError = %d\n", error);
            } else {
              printf("[IPC_SERVER] Server socket closed. Exiting receive loop.\n");
            }
            break;
          }

          std::thread clientThread(&IpcServer::HandleClient, this, clientSocket, clientAddr);
          clientThread.detach();
        }
      }

      void HandleClient(SOCKET clientSocket, SOCKADDR_IN clientAddr) {
        char pBuffer[1024] = {};
        int clientPort = ntohs(clientAddr.sin_port);

        while (m_running) {
          int dwBufferSize = recv(clientSocket, pBuffer, sizeof(pBuffer), 0);
          if (dwBufferSize <= 0) {
            uint32_t pid = 0;
            {
              std::lock_guard<std::mutex> lock(m_connectionsMutex);
              if (m_connections.count(clientPort)) {
                pid = m_connections[clientPort].processId;
              }
            }

            if (dwBufferSize == 0) {
              if (pid != 0) {
                printf("[IPC_SERVER] Client on port %d (PID %d) disconnected.\n", clientPort, pid);
              } else {
                printf("[IPC_SERVER] Client on port %d disconnected.\n", clientPort);
              }
            } else {
              if (pid != 0) {
                printf("[IPC_SERVER] Receive failed for client on port %d (PID %d). LastError = %d\n", clientPort, pid, GetSocketError());
              } else {
                printf("[IPC_SERVER] Receive failed for client on port %d. LastError = %d\n", clientPort, GetSocketError());
              }
            }
            {
              std::lock_guard<std::mutex> lock(m_connectionsMutex);
              m_connections.erase(clientPort);
            }
            break;
          }

          if (dwBufferSize < sizeof(CommandHeader_t)) {
            printf("[IPC_SERVER] Received invalid command header size from client on port %d.\n", clientPort);
            continue;
          }

          HandleIpcCommand(clientSocket, clientAddr, pBuffer);
        }
        closesocket(clientSocket);
      }

      void HandleIpcCommand(SOCKET clientSocket, const sockaddr_in &clientAddr, char *pBuffer) {
        uint16_t clientPort = ntohs(clientAddr.sin_port);

        CommandHeader_t *pHeader = reinterpret_cast<CommandHeader_t *>(pBuffer);
        void *pData = pBuffer + sizeof(CommandHeader_t);

        switch (pHeader->type) {
          case Command_ClientPing: {
            bool containsPort = false;
            {
              std::lock_guard<std::mutex> lock(m_connectionsMutex);
              containsPort = m_connections.count(clientPort) > 0;
            }
            if (pHeader->dataLen == 0 && containsPort) {
              SendIpcCommand(clientSocket, Command_ServerPong);
            }
            break;
          }

          case Command_ClientRequestHandshake: {
            CommandDataServerHandshakeResult_t response;
            response.result = HandshakeResult_Failed;
            response.ipcVersion = k_unIpcVersion;

            bool containsPort = false;
            {
              std::lock_guard<std::mutex> lock(m_connectionsMutex);
              containsPort = m_connections.count(clientPort) > 0;
            }

            if (pHeader->dataLen >= sizeof(CommandDataClientRequestHandshake_t) && !containsPort) {
              CommandDataClientRequestHandshake_t *pRequest = reinterpret_cast<CommandDataClientRequestHandshake_t *>(pData);

              if (pRequest->ipcVersion > k_unIpcVersion) {
                response.result = HandshakeResult_Outdated;
                printf("[IPC_SERVER] Handshake failed on port %d: Client IPC version %u is newer than server version %u.\n", clientPort, pRequest->ipcVersion, k_unIpcVersion);
              } else {
                std::lock_guard<std::mutex> lock(m_connectionsMutex);
                m_connections[clientPort] = { clientAddr, pRequest->ipcVersion, pRequest->processId };
                response.result = HandshakeResult_Success;
                printf("[IPC_SERVER] Client on port %d (PID %d) connected with IPC version %u.\n", clientPort, pRequest->processId, pRequest->ipcVersion);
              }
            }
            SendIpcCommand(clientSocket, Command_ServerHandshakeResult, &response, sizeof(response));
            break;
          }

          case Command_ClientRequestGazeData: {
            bool containsPort = false;
            uint16_t ipcVersion = 0;
            {
              std::lock_guard<std::mutex> lock(m_connectionsMutex);
              if (m_connections.count(clientPort)) {
                containsPort = true;
                ipcVersion = m_connections[clientPort].ipcVersion;
              }
            }
            if (pHeader->dataLen == 0 && containsPort) {
              if (m_doGaze) {
                float currentLeftOpenness = m_leftEyelidOpenness.load();
                float currentRightOpenness = m_rightEyelidOpenness.load();
                
                GazeEyeResult3 leftData = {
                  m_gazeStatus.wearable.left.is_gaze_origin_valid == HMD2_GAZE_BOOL_TRUE,
                  { m_gazeStatus.wearable.left.gaze_origin_mm.x, m_gazeStatus.wearable.left.gaze_origin_mm.y, m_gazeStatus.wearable.left.gaze_origin_mm.z },
                  m_gazeStatus.wearable.left.is_gaze_dir_valid == HMD2_GAZE_BOOL_TRUE,
                  { m_gazeStatus.wearable.left.gaze_dir_norm.x, m_gazeStatus.wearable.left.gaze_dir_norm.y, m_gazeStatus.wearable.left.gaze_dir_norm.z },
                  m_gazeStatus.wearable.left.is_pupil_dia_valid == HMD2_GAZE_BOOL_TRUE,
                  m_gazeStatus.wearable.left.pupil_dia_mm,
                  m_gazeStatus.wearable.left.is_pupil_pos_in_sensor_area_valid == HMD2_GAZE_BOOL_TRUE,
                  { m_gazeStatus.wearable.left.pupil_pos_in_sensor_area.x, m_gazeStatus.wearable.left.pupil_pos_in_sensor_area.y },
                  m_gazeStatus.wearable.left.is_pos_guide_valid == HMD2_GAZE_BOOL_TRUE,
                  { m_gazeStatus.wearable.left.pos_guide.x, m_gazeStatus.wearable.left.pos_guide.y },
                  m_gazeStatus.wearable.left.is_blink_valid == HMD2_GAZE_BOOL_TRUE,
                  m_gazeStatus.wearable.left.blink == HMD2_GAZE_BOOL_TRUE,
                  m_doOpenness,
                  m_doOpenness ? currentLeftOpenness : 0.0f
                };

                GazeEyeResult3 rightData = {
                  m_gazeStatus.wearable.right.is_gaze_origin_valid == HMD2_GAZE_BOOL_TRUE,
                  { m_gazeStatus.wearable.right.gaze_origin_mm.x, m_gazeStatus.wearable.right.gaze_origin_mm.y, m_gazeStatus.wearable.right.gaze_origin_mm.z },
                  m_gazeStatus.wearable.right.is_gaze_dir_valid == HMD2_GAZE_BOOL_TRUE,
                  { m_gazeStatus.wearable.right.gaze_dir_norm.x, m_gazeStatus.wearable.right.gaze_dir_norm.y, m_gazeStatus.wearable.right.gaze_dir_norm.z },
                  m_gazeStatus.wearable.right.is_pupil_dia_valid == HMD2_GAZE_BOOL_TRUE,
                  m_gazeStatus.wearable.right.pupil_dia_mm,
                  m_gazeStatus.wearable.right.is_pupil_pos_in_sensor_area_valid == HMD2_GAZE_BOOL_TRUE,
                  { m_gazeStatus.wearable.right.pupil_pos_in_sensor_area.x, m_gazeStatus.wearable.right.pupil_pos_in_sensor_area.y },
                  m_gazeStatus.wearable.right.is_pos_guide_valid == HMD2_GAZE_BOOL_TRUE,
                  { m_gazeStatus.wearable.right.pos_guide.x, m_gazeStatus.wearable.right.pos_guide.y },
                  m_gazeStatus.wearable.right.is_blink_valid == HMD2_GAZE_BOOL_TRUE,
                  m_gazeStatus.wearable.right.blink == HMD2_GAZE_BOOL_TRUE,
                  m_doOpenness,
                  m_doOpenness ? currentRightOpenness : 0.0f
                };

                if (ipcVersion == 1) {
                  CommandDataServerGazeDataResult_t response = {
                    { leftData.isGazeOriginValid, leftData.gazeOriginMm, leftData.isGazeDirValid, leftData.gazeDirNorm, leftData.isPupilDiaValid, leftData.pupilDiaMm, leftData.isBlinkValid, leftData.blink },
                    { rightData.isGazeOriginValid, rightData.gazeOriginMm, rightData.isGazeDirValid, rightData.gazeDirNorm, rightData.isPupilDiaValid, rightData.pupilDiaMm, rightData.isBlinkValid, rightData.blink }
                  };
                  SendIpcCommand(clientSocket, Command_ServerGazeDataResult, &response, sizeof(response));
                } else if (ipcVersion == 2) {
                  CommandDataServerGazeDataResult2_t response = {
                    { leftData.isGazeOriginValid, leftData.gazeOriginMm, leftData.isGazeDirValid, leftData.gazeDirNorm, leftData.isPupilDiaValid, leftData.pupilDiaMm, leftData.isBlinkValid, leftData.blink, leftData.isOpenEnabled, leftData.open },
                    { rightData.isGazeOriginValid, rightData.gazeOriginMm, rightData.isGazeDirValid, rightData.gazeDirNorm, rightData.isPupilDiaValid, rightData.pupilDiaMm, rightData.isBlinkValid, rightData.blink, rightData.isOpenEnabled, rightData.open }
                  };
                  SendIpcCommand(clientSocket, Command_ServerGazeDataResult, &response, sizeof(response));
                } else {
                  CommandDataServerGazeDataResult3_t response = { leftData, rightData };
                  SendIpcCommand(clientSocket, Command_ServerGazeDataResult, &response, sizeof(response));
                }
              } else {
                if (ipcVersion == 1) {
                  CommandDataServerGazeDataResult_t response = {};
                  SendIpcCommand(clientSocket, Command_ServerGazeDataResult, &response, sizeof(response));
                } else if (ipcVersion == 2) {
                  CommandDataServerGazeDataResult2_t response = {};
                  SendIpcCommand(clientSocket, Command_ServerGazeDataResult, &response, sizeof(response));
                } else {
                  CommandDataServerGazeDataResult3_t response = {};
                  SendIpcCommand(clientSocket, Command_ServerGazeDataResult, &response, sizeof(response));
                }
              }
            }
            break;
          }

          case Command_ClientTriggerEffectOff:
          case Command_ClientTriggerEffectFeedback:
          case Command_ClientTriggerEffectWeapon:
          case Command_ClientTriggerEffectVibration:
          case Command_ClientTriggerEffectMultiplePositionFeedback:
          case Command_ClientTriggerEffectSlopeFeedback:
          case Command_ClientTriggerEffectMultiplePositionVibration: {
            bool containsPort = false;
            {
              std::lock_guard<std::mutex> lock(m_connectionsMutex);
              containsPort = m_connections.count(clientPort) > 0;
            }

            if (containsPort) {
              if (pHeader->dataLen >= sizeof(EVRControllerType)) {
                EVRControllerType controllerType = *reinterpret_cast<EVRControllerType*>(pData);
                VRControllerType mappedControllerType = VRControllerType::Both;
                switch (controllerType) {
                    case VRController_Left: mappedControllerType = VRControllerType::Left; break;
                    case VRController_Right: mappedControllerType = VRControllerType::Right; break;
                    case VRController_Both: mappedControllerType = VRControllerType::Both; break;
                }

                ScePadTriggerEffectCommand cmd = {};
                
                switch (pHeader->type) {
                  case Command_ClientTriggerEffectOff: {
                      cmd.mode = SCE_PAD_TRIGGER_EFFECT_MODE_OFF;
                      break;
                  }
                  case Command_ClientTriggerEffectFeedback: {
                      cmd.mode = SCE_PAD_TRIGGER_EFFECT_MODE_FEEDBACK;
                      if (pHeader->dataLen >= sizeof(CommandDataClientTriggerEffectFeedback_t)) {
                          CommandDataClientTriggerEffectFeedback_t* req = reinterpret_cast<CommandDataClientTriggerEffectFeedback_t*>(pData);
                          cmd.commandData.feedbackParam.position = req->position;
                          cmd.commandData.feedbackParam.strength = req->strength;
                      }
                      break;
                  }
                  case Command_ClientTriggerEffectWeapon: {
                      cmd.mode = SCE_PAD_TRIGGER_EFFECT_MODE_WEAPON;
                      if (pHeader->dataLen >= sizeof(CommandDataClientTriggerEffectWeapon_t)) {
                          CommandDataClientTriggerEffectWeapon_t* req = reinterpret_cast<CommandDataClientTriggerEffectWeapon_t*>(pData);
                          cmd.commandData.weaponParam.startPosition = req->startPosition;
                          cmd.commandData.weaponParam.endPosition = req->endPosition;
                          cmd.commandData.weaponParam.strength = req->strength;
                      }
                      break;
                  }
                  case Command_ClientTriggerEffectVibration: {
                      cmd.mode = SCE_PAD_TRIGGER_EFFECT_MODE_VIBRATION;
                      if (pHeader->dataLen >= sizeof(CommandDataClientTriggerEffectVibration_t)) {
                          CommandDataClientTriggerEffectVibration_t* req = reinterpret_cast<CommandDataClientTriggerEffectVibration_t*>(pData);
                          cmd.commandData.vibrationParam.position = req->position;
                          cmd.commandData.vibrationParam.amplitude = req->amplitude;
                          cmd.commandData.vibrationParam.frequency = req->frequency;
                      }
                      break;
                  }
                  case Command_ClientTriggerEffectMultiplePositionFeedback: {
                      cmd.mode = SCE_PAD_TRIGGER_EFFECT_MODE_MULTIPLE_POSITION_FEEDBACK;
                      if (pHeader->dataLen >= sizeof(CommandDataClientTriggerEffectMultiplePositionFeedback_t)) {
                          CommandDataClientTriggerEffectMultiplePositionFeedback_t* req = reinterpret_cast<CommandDataClientTriggerEffectMultiplePositionFeedback_t*>(pData);
                          memcpy(cmd.commandData.multiplePositionFeedbackParam.strength, req->strength, sizeof(req->strength));
                      }
                      break;
                  }
                  case Command_ClientTriggerEffectSlopeFeedback: {
                      cmd.mode = SCE_PAD_TRIGGER_EFFECT_MODE_SLOPE_FEEDBACK;
                      if (pHeader->dataLen >= sizeof(CommandDataClientTriggerEffectSlopeFeedback_t)) {
                          CommandDataClientTriggerEffectSlopeFeedback_t* req = reinterpret_cast<CommandDataClientTriggerEffectSlopeFeedback_t*>(pData);
                          cmd.commandData.slopeFeedbackParam.startPosition = req->startPosition;
                          cmd.commandData.slopeFeedbackParam.endPosition = req->endPosition;
                          cmd.commandData.slopeFeedbackParam.startStrength = req->startStrength;
                          cmd.commandData.slopeFeedbackParam.endStrength = req->endStrength;
                      }
                      break;
                  }
                  case Command_ClientTriggerEffectMultiplePositionVibration: {
                      cmd.mode = SCE_PAD_TRIGGER_EFFECT_MODE_MULTIPLE_POSITION_VIBRATION;
                      if (pHeader->dataLen >= sizeof(CommandDataClientTriggerEffectMultiplePositionVibration_t)) {
                          CommandDataClientTriggerEffectMultiplePositionVibration_t* req = reinterpret_cast<CommandDataClientTriggerEffectMultiplePositionVibration_t*>(pData);
                          cmd.commandData.multiplePositionVibrationParam.frequency = req->frequency;
                          memcpy(cmd.commandData.multiplePositionVibrationParam.amplitude, req->amplitude, sizeof(req->amplitude));
                      }
                      break;
                  }
                }
                
                psvr2_toolkit_set_trigger_effect(mappedControllerType, cmd);
              }
            }
            break;
          }
        }
      }

      void SendIpcCommand(SOCKET clientSocket, ECommandType type, void *pData = nullptr, int dataLen = 0) {
        char pBuffer[1024] = {};

        int actualDataLen = pData ? dataLen : 0;
        int actualBufferLen = sizeof(CommandHeader_t) + actualDataLen;

        CommandHeader_t *pHeader = reinterpret_cast<CommandHeader_t *>(pBuffer);
        pHeader->type = type;
        pHeader->dataLen = actualDataLen;
        if (pData && actualDataLen > 0) {
          memcpy(pBuffer + sizeof(CommandHeader_t), pData, actualDataLen);
        }

        send(clientSocket, pBuffer, actualBufferLen, MSG_NOSIGNAL);
      }
    };

    IpcServer* IpcServer::m_pInstance = nullptr;

  } // ipc
} // psvr2_toolkit

int main() {
  psvr2_toolkit_loader_init_functions(psvr2_toolkit_loader_get_module_handle());
  if (psvr2_toolkit_init() < 0) {
    std::cerr << "Failed to initialize PSVR2 CAPI!" << std::endl;
    return -1;
  }

  psvr2_toolkit::ipc::IpcServer* server = psvr2_toolkit::ipc::IpcServer::Instance();
  server->Initialize();
  server->Start();

  std::cout << "PSVR2 Legacy IPC Server is running using CAPI on port " << IPC_SERVER_PORT << "..." << std::endl;
  std::cout << "Press Enter to exit." << std::endl;

  std::cin.get();

  server->Stop();
  psvr2_toolkit_deinit();

  return 0;
}