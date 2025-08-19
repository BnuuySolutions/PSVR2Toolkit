#pragma once

#include "hmd2_gaze.h"
#include "../shared/ipc_protocol.h"

#include <windows.h>

#include <cstdint>
#include <map>
#include <thread>

namespace psvr2_toolkit {
  namespace ipc {

    class IpcServer {
    public:
      IpcServer();

      static IpcServer *Instance();

      bool Initialized();
      void Initialize();

      void Start();
      void Stop();

      void UpdateGazeState(Hmd2GazeState *pGazeState);

    private:
      struct ConnectionInfo_t {
        sockaddr_in clientAddr;
        uint16_t ipcVersion;
        uint32_t processId;
      };

      static IpcServer *m_pInstance;

      bool m_initialized;
      bool m_running;
      bool m_doGaze;
      SOCKET m_socket;
      sockaddr_in m_serverAddr;
      std::thread m_receiveThread;
      std::map<uint16_t, ConnectionInfo_t> m_connections;

      Hmd2GazeState *m_pGazeState;

      void ReceiveLoop();
      void HandleClient(SOCKET clientSocket, SOCKADDR_IN clientAddr);

      void HandleIpcCommand(SOCKET clientSocket, const sockaddr_in &clientAddr, char *pBuffer);
      void SendIpcCommand(SOCKET clientSocket, ECommandType type, void *pData = nullptr, int dataSize = 0);
    };

  } // ipc
} // psvr2_toolkit
