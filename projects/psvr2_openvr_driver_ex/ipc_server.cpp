#include "ipc_server.h"

#include "trigger_effect_manager.h"
#include "util.h"
#include "vr_settings.h"

#include <cstdio>

namespace psvr2_toolkit {
  namespace ipc {

    IpcServer *IpcServer::m_pInstance = nullptr;

    IpcServer::IpcServer()
      : m_initialized(false)
      , m_running(false)
      , m_doGaze(false)
      , m_socket{}
      , m_serverAddr{}
      , m_pGazeState(nullptr)
    {}

    IpcServer *IpcServer::Instance() {
      if (!m_pInstance) {
        m_pInstance = new IpcServer;
      }

      return m_pInstance;
    }

    bool IpcServer::Initialized() {
      return m_initialized;
    }

    void IpcServer::Initialize() {
      if (m_initialized) {
        return;
      }

      WSADATA wsaData = {};
      int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
      if (result != 0) {
        Util::DriverLog("[IPC_SERVER] WSAStartup failed. Result = {}", result);
        return;
      }

      m_initialized = true;
      m_doGaze = !VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_GAZE, SETTING_DISABLE_GAZE_DEFAULT_VALUE);
      m_doOpenness = VRSettings::GetBool(STEAMVR_SETTINGS_ENABLE_EYELID_ESTIMATION, SETTING_ENABLE_EYELID_ESTIMATION_DEFAULT_VALUE);
    }

    void IpcServer::Start() {
      if (!m_initialized && m_running) {
        return;
      }

      m_socket = socket(AF_INET, SOCK_STREAM, 0);
      if (m_socket == INVALID_SOCKET) {
        Util::DriverLog("[IPC_SERVER] Creating socket failed. LastError = {}", WSAGetLastError());
        return;
      }

      m_serverAddr.sin_family = AF_INET;
      m_serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
      m_serverAddr.sin_port = htons(IPC_SERVER_PORT);

      if (bind(m_socket, reinterpret_cast<SOCKADDR * >(&m_serverAddr), sizeof(m_serverAddr)) == SOCKET_ERROR) {
        Util::DriverLog("[IPC_SERVER] Bind failed. LastError = {}", WSAGetLastError());
        closesocket(m_socket);
        return;
      }

      if (listen(m_socket, SOMAXCONN) == SOCKET_ERROR) {
        Util::DriverLog("[IPC_SERVER] Listen failed. LastError = {}", WSAGetLastError());
        closesocket(m_socket);
        return;
      }

      m_running = true;
      m_receiveThread = std::thread(&IpcServer::ReceiveLoop, this);
    }

    void IpcServer::Stop() {
      if (!m_running) {
        return;
      }

      m_running = false;
      closesocket(m_socket);
      m_receiveThread.join();
    }

    void IpcServer::UpdateGazeState(Hmd2GazeState *pGazeState, float leftEyelidOpenness, float rightEyelidOpenness) {
      if (!m_pGazeState) {
        m_pGazeState = reinterpret_cast<Hmd2GazeState *>(malloc(sizeof(Hmd2GazeState)));
      }
      if (m_pGazeState) {
        memcpy(m_pGazeState, pGazeState, sizeof(Hmd2GazeState)); // Realistically, we should have a mutex here. Sadly, we do not have the time.
      }
      m_leftEyelidOpenness = leftEyelidOpenness;
      m_rightEyelidOpenness = rightEyelidOpenness;
    }

    void IpcServer::ReceiveLoop() {
      char pBuffer[1024] = {}; // This does not need to be static.
      sockaddr_in clientAddr = {};
      int clientAddrLen = sizeof(clientAddr);

      while (m_running) {
        SOCKADDR_IN clientAddr = {};
        int clientAddrLen = sizeof(clientAddr);

        SOCKET clientSocket = accept(m_socket, reinterpret_cast<SOCKADDR*>(&clientAddr), &clientAddrLen);
        if (clientSocket == INVALID_SOCKET) {
          int error = WSAGetLastError();
          if (m_running) {
              Util::DriverLog("[IPC_SERVER] Accept failed. LastError = {}", error);
          }
          else {
            Util::DriverLog("[IPC_SERVER] Server socket closed. Exiting receive loop.");
          }
          break;
        }

        std::thread clientThread(&IpcServer::HandleClient, this, clientSocket, clientAddr);
        clientThread.detach(); // client thread shouldnt block receive thread
      }
    }

    void IpcServer::HandleClient(SOCKET clientSocket, SOCKADDR_IN clientAddr) {
      char pBuffer[1024] = {};
      int clientPort = ntohs(clientAddr.sin_port);

      while (m_running) {

        int dwBufferSize = recv(clientSocket, pBuffer, sizeof(pBuffer), 0);
        if (dwBufferSize <= 0) {
          if (dwBufferSize == 0) {
            Util::DriverLog("[IPC_SERVER] Client on port {} disconnected.", clientPort);
          } else {
            Util::DriverLog("[IPC_SERVER] Receive failed for client on port {}. LastError = {}", clientPort, WSAGetLastError());
          }

          break;
        }

        if (dwBufferSize < sizeof(CommandHeader_t)) {
          Util::DriverLog("[IPC_SERVER] Received invalid command header size from client on port {}.", clientPort);
          continue;
        }

        HandleIpcCommand(clientSocket, clientAddr, pBuffer);
      }
      closesocket(clientSocket);
    }

    void IpcServer::HandleIpcCommand(SOCKET clientSocket, const sockaddr_in &clientAddr, char *pBuffer) {
      static TriggerEffectManager *pTriggerEffectManager = TriggerEffectManager::Instance();

      uint16_t clientPort = ntohs(clientAddr.sin_port);

      CommandHeader_t *pHeader = reinterpret_cast<CommandHeader_t *>(pBuffer);
      void *pData = pBuffer + sizeof(CommandHeader_t);

      switch (pHeader->type) {
        case Command_ClientPing: {
          if (pHeader->dataLen == 0 && m_connections.contains(clientPort)) {
            SendIpcCommand(clientSocket, Command_ServerPong); // TODO
          }
          break;
        }

        case Command_ClientRequestHandshake: {
          CommandDataServerHandshakeResult_t response;
          response.result = HandshakeResult_Failed;
          response.ipcVersion = k_unIpcVersion; // Communicate the IPC version the server supports.

          if (pHeader->dataLen == sizeof(CommandDataClientRequestHandshake_t) && !m_connections.contains(clientPort)) {
            CommandDataClientRequestHandshake_t *pRequest = reinterpret_cast<CommandDataClientRequestHandshake_t *>(pData);

            // We only want real running processes to handshake with us.
            if (Util::IsProcessRunning(pRequest->processId)) {
              m_connections[clientPort] = {
                .clientAddr = clientAddr,
                .ipcVersion = pRequest->ipcVersion,
                .processId = pRequest->processId
              };

              response.result = HandshakeResult_Success;
            }
          }

          SendIpcCommand(clientSocket, Command_ServerHandshakeResult, &response, sizeof(response));
          break;
        }

        case Command_ClientRequestGazeData: {
          if (pHeader->dataLen == 0 && m_connections.contains(clientPort)) {

            if (m_doGaze && m_pGazeState) {
              // Handle old client IPC version requests here.
              if (m_connections[clientPort].ipcVersion == 1) {
                CommandDataServerGazeDataResult_t response = {
                  .leftEye = {
                    .isGazeOriginValid = m_pGazeState->leftEye.isGazeOriginValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .gazeOriginMm = {
                      .x = m_pGazeState->leftEye.gazeOriginMm.x,
                      .y = m_pGazeState->leftEye.gazeOriginMm.y,
                      .z = m_pGazeState->leftEye.gazeOriginMm.z,
                    },
                    .isGazeDirValid = m_pGazeState->leftEye.isGazeDirValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .gazeDirNorm {
                      .x = m_pGazeState->leftEye.gazeDirNorm.x,
                      .y = m_pGazeState->leftEye.gazeDirNorm.y,
                      .z = m_pGazeState->leftEye.gazeDirNorm.z,
                    },
                    .isPupilDiaValid = m_pGazeState->leftEye.isPupilDiaValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .pupilDiaMm = m_pGazeState->leftEye.pupilDiaMm,
                    .isBlinkValid = m_pGazeState->leftEye.isBlinkValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .blink = m_pGazeState->leftEye.blink == Hmd2Bool::HMD2_BOOL_TRUE,
                  },

                  .rightEye = {
                    .isGazeOriginValid = m_pGazeState->rightEye.isGazeOriginValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .gazeOriginMm = {
                      .x = m_pGazeState->rightEye.gazeOriginMm.x,
                      .y = m_pGazeState->rightEye.gazeOriginMm.y,
                      .z = m_pGazeState->rightEye.gazeOriginMm.z,
                    },
                    .isGazeDirValid = m_pGazeState->rightEye.isGazeDirValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .gazeDirNorm {
                      .x = m_pGazeState->rightEye.gazeDirNorm.x,
                      .y = m_pGazeState->rightEye.gazeDirNorm.y,
                      .z = m_pGazeState->rightEye.gazeDirNorm.z,
                    },
                    .isPupilDiaValid = m_pGazeState->rightEye.isPupilDiaValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .pupilDiaMm = m_pGazeState->rightEye.pupilDiaMm,
                    .isBlinkValid = m_pGazeState->rightEye.isBlinkValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .blink = m_pGazeState->rightEye.blink == Hmd2Bool::HMD2_BOOL_TRUE,
                  }
                };
                SendIpcCommand(clientSocket, Command_ServerGazeDataResult, &response, sizeof(response));
              } else {
                CommandDataServerGazeDataResult2_t response = {
                  .leftEye = {
                    .isGazeOriginValid = m_pGazeState->leftEye.isGazeOriginValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .gazeOriginMm = {
                      .x = m_pGazeState->leftEye.gazeOriginMm.x,
                      .y = m_pGazeState->leftEye.gazeOriginMm.y,
                      .z = m_pGazeState->leftEye.gazeOriginMm.z,
                    },
                    .isGazeDirValid = m_pGazeState->leftEye.isGazeDirValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .gazeDirNorm {
                      .x = m_pGazeState->leftEye.gazeDirNorm.x,
                      .y = m_pGazeState->leftEye.gazeDirNorm.y,
                      .z = m_pGazeState->leftEye.gazeDirNorm.z,
                    },
                    .isPupilDiaValid = m_pGazeState->leftEye.isPupilDiaValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .pupilDiaMm = m_pGazeState->leftEye.pupilDiaMm,
                    .isBlinkValid = m_pGazeState->leftEye.isBlinkValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .blink = m_pGazeState->leftEye.blink == Hmd2Bool::HMD2_BOOL_TRUE,
                    .isOpenEnabled = m_doOpenness,
                    .open = m_doOpenness ? m_leftEyelidOpenness : 0.0f,
                  },

                  .rightEye = {
                    .isGazeOriginValid = m_pGazeState->rightEye.isGazeOriginValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .gazeOriginMm = {
                      .x = m_pGazeState->rightEye.gazeOriginMm.x,
                      .y = m_pGazeState->rightEye.gazeOriginMm.y,
                      .z = m_pGazeState->rightEye.gazeOriginMm.z,
                    },
                    .isGazeDirValid = m_pGazeState->rightEye.isGazeDirValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .gazeDirNorm {
                      .x = m_pGazeState->rightEye.gazeDirNorm.x,
                      .y = m_pGazeState->rightEye.gazeDirNorm.y,
                      .z = m_pGazeState->rightEye.gazeDirNorm.z,
                    },
                    .isPupilDiaValid = m_pGazeState->rightEye.isPupilDiaValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .pupilDiaMm = m_pGazeState->rightEye.pupilDiaMm,
                    .isBlinkValid = m_pGazeState->rightEye.isBlinkValid == Hmd2Bool::HMD2_BOOL_TRUE,
                    .blink = m_pGazeState->rightEye.blink == Hmd2Bool::HMD2_BOOL_TRUE,
                    .isOpenEnabled = m_doOpenness,
                    .open = m_doOpenness ? m_rightEyelidOpenness : 0.0f,
                  }
                };
                SendIpcCommand(clientSocket, Command_ServerGazeDataResult, &response, sizeof(response));
              }
            } else {
              // Handle old client IPC version requests here, as well.
              if (m_connections[clientPort].ipcVersion == 1) {
                CommandDataServerGazeDataResult_t response = {};
                SendIpcCommand(clientSocket, Command_ServerGazeDataResult, &response, sizeof(response));
              } else {
                CommandDataServerGazeDataResult2_t response = {};
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
          if (m_connections.contains(clientPort)) {
            pTriggerEffectManager->HandleIpcCommand(m_connections[clientPort].processId, pHeader, pData);
          }
          break;
        }
      }
    }

    void IpcServer::SendIpcCommand(SOCKET clientSocket, ECommandType type, void *pData, int dataLen) {
      // Reduce the allocations by making the buffer static.
      // I'm sure this isn't thread-safe, but we only have one receive thread anyways.
      static char pBuffer[1024] = {};

      int actualDataLen = pData ? dataLen : 0;
      int actualBufferLen = sizeof(CommandHeader_t) + actualDataLen;

      CommandHeader_t *pHeader = reinterpret_cast<CommandHeader_t *>(pBuffer);
      pHeader->type = type;
      pHeader->dataLen = actualDataLen;
      memcpy(pBuffer + sizeof(CommandHeader_t), pData, actualDataLen);

      send(clientSocket, pBuffer, actualBufferLen, 0);
    }

  } // ipc
} // psvr2_toolkit
