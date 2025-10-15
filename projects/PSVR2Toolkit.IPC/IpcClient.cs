using System;
using System.Diagnostics;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;

namespace PSVR2Toolkit.CAPI {
    public class IpcClient {
        private const ushort IPC_SERVER_PORT = 3364;
        private const ushort k_unIpcVersion = 2;

        private static IpcClient m_pInstance;

        private bool m_running = false;
        private TcpClient? m_client;
        private NetworkStream? m_stream;
        private Thread? m_receiveThread;

        private readonly object m_gazeStateLock = new object();
        private TaskCompletionSource<CommandDataServerGazeDataResult>? m_gazeTask;
        private CancellationTokenSource m_forceShutdownToken;
        private ushort m_serverIpcVersion = k_unIpcVersion;
        private int m_gazePumpPeriodMs = 8; // 120Hz
        private CommandDataServerGazeDataResult2? m_lastGazeState = null;

        public static IpcClient Instance() {
            if ( m_pInstance == null ) {
                m_pInstance = new IpcClient();
            }
            return m_pInstance;
        }
        public bool Start() {
            if ( m_running ) {
                return false;
            }

            try {
                m_client = new TcpClient();
                m_client.Connect("127.0.0.1", IPC_SERVER_PORT);
                if ( m_client.Connected ) {
                    m_stream = m_client.GetStream();

                    m_running = true;
                    m_forceShutdownToken = new CancellationTokenSource();
                    m_receiveThread = new Thread(() => ReceiveLoop(m_forceShutdownToken.Token));
                    m_receiveThread.Start();
                    return true;
                }
                return false;
            } catch ( SocketException ex ) {
                Console.WriteLine($"[IPC_CLIENT] Connection failed. LastError = {ex.SocketErrorCode}");
                return false;
            }
        }

        public void Stop() {
            if ( !m_running ) {
                return;
            }

            m_running = false;
            m_forceShutdownToken.Cancel();

            lock ( m_gazeStateLock ) {
                m_gazeTask?.TrySetCanceled();
                m_gazeTask = null;
            }

            try {
                m_stream?.Close();
                m_client?.Close();
            } catch { }

            if ( m_receiveThread != null && m_receiveThread.IsAlive ) {
                if ( !m_receiveThread.Join(2000) ) {
                    m_receiveThread.Interrupt();
                }
            }

            m_stream?.Dispose();
            m_client?.Dispose();
            m_forceShutdownToken.Dispose();
        }

        private void ReceiveLoop(CancellationToken token) {
            byte[] buffer = new byte[1024];

            try {
                var clientSocket = m_client!.Client;
                m_stream!.ReadTimeout = 1; // make the underlying stream non-blocking

                CommandDataClientRequestHandshake clientHandshakeRequest = new CommandDataClientRequestHandshake() {
                    ipcVersion = k_unIpcVersion,
                    processId = ( uint ) Process.GetCurrentProcess().Id
                };
                SendIpcCommand(ECommandType.ClientRequestHandshake, clientHandshakeRequest);

                var sw = Stopwatch.StartNew();
                long nextPumpMs = sw.ElapsedMilliseconds;

                while ( m_running && !token.IsCancellationRequested ) {
                    // query gaze state every so often
                    var now = sw.ElapsedMilliseconds;
                    if ( now >= nextPumpMs ) {
                        SendIpcCommand(ECommandType.ClientRequestGazeData);
                        nextPumpMs = now + m_gazePumpPeriodMs;
                    }

                    bool readable = clientSocket.Poll(1000 /* 1ms */, SelectMode.SelectRead);
                    if ( readable && clientSocket.Available > 0 ) {
                        int available = clientSocket.Available;
                        if ( available > buffer.Length ) {
                            buffer = new byte[Math.Max(available, buffer.Length * 2)];
                        }

                        int bytesRead = m_stream.Read(buffer, 0, Math.Min(buffer.Length, available));
                        if ( bytesRead <= 0 ) {
                            Console.WriteLine("[IPC_CLIENT] Disconnected from server.");
                            break;
                        }

                        if ( bytesRead < Marshal.SizeOf<CommandHeader>() ) {
                            Console.WriteLine("[IPC_CLIENT] Received invalid command header size.");
                            continue;
                        }

                        HandleIpcCommand(buffer, bytesRead);
                        continue;
                    }

                    Thread.Sleep(1);
                }
            } catch ( OperationCanceledException ) {
                // nothing special, this is from shutdown most likely
            } catch ( Exception ex ) {
                if ( m_running ) {
                    Console.WriteLine($"[IPC_CLIENT] Error in receive loop: {ex.Message}");
                }
            }
        }

        private GazeEyeResult2 UpgradeGazeEyeResult(GazeEyeResult eye)
        {
            return new GazeEyeResult2
            {
                isGazeOriginValid = eye.isGazeOriginValid,
                gazeOriginMm = eye.gazeOriginMm,
                isGazeDirValid = eye.isGazeDirValid,
                gazeDirNorm = eye.gazeDirNorm,
                isPupilDiaValid = eye.isPupilDiaValid,
                pupilDiaMm = eye.pupilDiaMm,
                isBlinkValid = eye.isBlinkValid,
                blink = eye.blink,
                isOpenEnabled = false,
                open = 0f
            };
        }

        private CommandDataServerGazeDataResult2 UpgradeGazeDataResult(CommandDataServerGazeDataResult result)
        {
            return new CommandDataServerGazeDataResult2
            {
                leftEye = UpgradeGazeEyeResult(result.leftEye),
                rightEye = UpgradeGazeEyeResult(result.rightEye)
            };
        }

        private void HandleIpcCommand(byte[] pBuffer, int bytesReceived) {
            CommandHeader header = ByteArrayToStructure<CommandHeader>(pBuffer, 0);

            switch ( header.type ) {
                case ECommandType.ServerPong: {
                        Console.WriteLine("[IPC_CLIENT] Received Pong from server.");
                        break;
                    }

                case ECommandType.ServerHandshakeResult: {
                        if ( header.dataLen == Marshal.SizeOf<CommandDataServerHandshakeResult>() ) {
                            CommandDataServerHandshakeResult response = ByteArrayToStructure<CommandDataServerHandshakeResult>(pBuffer, Marshal.SizeOf<CommandHeader>());
                            m_serverIpcVersion = response.ipcVersion;
                            switch ( response.result ) {
                                case EHandshakeResult.Success: {
                                        Console.WriteLine("[IPC_CLIENT] Handshake successful!");
                                        break;
                                    }
                                case EHandshakeResult.Failed: {
                                        Console.WriteLine("[IPC_CLIENT] Handshake failed!");
                                        break;
                                    }
                                case EHandshakeResult.Outdated: {
                                        Console.WriteLine($"[IPC_CLIENT] Handshake failed with reason: Outdated client. Please upgrade to an IPC version of {response.ipcVersion}");
                                        break;
                                    }
                            }
                        }
                        break;
                    }
                case ECommandType.ServerGazeDataResult: {
                        if ( m_serverIpcVersion == 1 ) {
                            if ( header.dataLen == Marshal.SizeOf<CommandDataServerGazeDataResult>() ) {
                                CommandDataServerGazeDataResult response = ByteArrayToStructure<CommandDataServerGazeDataResult>(pBuffer, Marshal.SizeOf<CommandHeader>());
                                m_lastGazeState = UpgradeGazeDataResult(response);

                            }
                        } else {
                            if ( header.dataLen == Marshal.SizeOf<CommandDataServerGazeDataResult2>() ) {
                                CommandDataServerGazeDataResult2 response = ByteArrayToStructure<CommandDataServerGazeDataResult2>(pBuffer, Marshal.SizeOf<CommandHeader>());
                                m_lastGazeState = response;

                            }
                        }
                        break;
                    }
            }
        }

        private void SendIpcCommand<T>(ECommandType type, T data = default) where T : struct {
            if ( !m_running )
                return;

            int dataLen = data.Equals(default(T)) ? 0 : Marshal.SizeOf<T>();
            int bufferLen = Marshal.SizeOf<CommandHeader>() + dataLen;
            byte[] buffer = new byte[bufferLen];

            CommandHeader header = new CommandHeader
            {
                type = type,
                dataLen = dataLen
            };

            IntPtr headerPtr = Marshal.AllocHGlobal(Marshal.SizeOf<CommandHeader>());
            Marshal.StructureToPtr(header, headerPtr, false);
            Marshal.Copy(headerPtr, buffer, 0, Marshal.SizeOf<CommandHeader>());
            Marshal.FreeHGlobal(headerPtr);

            if ( dataLen > 0 ) {
                IntPtr dataPtr = Marshal.AllocHGlobal(dataLen);
                Marshal.StructureToPtr(data, dataPtr, false);
                Marshal.Copy(dataPtr, buffer, Marshal.SizeOf<CommandHeader>(), dataLen);
                Marshal.FreeHGlobal(dataPtr);
            }

            m_stream.Write(buffer, 0, buffer.Length);
        }

        // no data
        private void SendIpcCommand(ECommandType type) {
            if ( !m_running )
                return;

            int bufferLen = Marshal.SizeOf<CommandHeader>();
            byte[] buffer = new byte[bufferLen];

            CommandHeader header = new CommandHeader
            {
                type = type,
                dataLen = 0
            };

            IntPtr headerPtr = Marshal.AllocHGlobal(Marshal.SizeOf<CommandHeader>());
            Marshal.StructureToPtr(header, headerPtr, false);
            Marshal.Copy(headerPtr, buffer, 0, Marshal.SizeOf<CommandHeader>());
            Marshal.FreeHGlobal(headerPtr);

            m_stream.Write(buffer, 0, buffer.Length);
        }

        private T ByteArrayToStructure<T>(byte[] bytes, int offset) where T : struct {
            int size = Marshal.SizeOf<T>();
            if ( size > bytes.Length - offset ) {
                throw new ArgumentException("Byte array is too small to contain the structure.");
            }

            IntPtr ptr = Marshal.AllocHGlobal(size);
            Marshal.Copy(bytes, offset, ptr, size);
            T structure = (T)Marshal.PtrToStructure(ptr, typeof(T));
            Marshal.FreeHGlobal(ptr);

            return structure;
        }

        public CommandDataServerGazeDataResult2 RequestEyeTrackingData() {

            if ( !m_running ) {
                return new CommandDataServerGazeDataResult2();
            }

            return m_lastGazeState ?? new CommandDataServerGazeDataResult2();
        }

        public void TriggerEffectDisable(EVRControllerType controllerType) {
            if ( !m_running ) {
                return;
            }

            CommandDataClientTriggerEffectOff effectOff = new CommandDataClientTriggerEffectOff() {
                controllerType = controllerType
            };
            SendIpcCommand(ECommandType.ClientTriggerEffectOff, effectOff);
        }

        public void TriggerEffectFeedback(EVRControllerType controllerType, byte position, byte strength) {
            if ( !m_running ) {
                return;
            }

            CommandDataClientTriggerEffectFeedback effectFeedback = new CommandDataClientTriggerEffectFeedback() {
                controllerType = controllerType,
                position = position,
                strength = strength,
            };
            SendIpcCommand(ECommandType.ClientTriggerEffectFeedback, effectFeedback);
        }
        public void TriggerEffectWeapon(EVRControllerType controllerType, byte startPosition, byte endPosition, byte strength) {
            if ( !m_running ) {
                return;
            }

            CommandDataClientTriggerEffectWeapon effectWeapon = new CommandDataClientTriggerEffectWeapon() {
                controllerType = controllerType,
                startPosition = startPosition,
                endPosition = endPosition,
                strength = strength,
            };
            SendIpcCommand(ECommandType.ClientTriggerEffectWeapon, effectWeapon);
        }
        public void TriggerEffectVibration(EVRControllerType controllerType, byte position, byte amplitude, byte frequency) {
            if ( !m_running ) {
                return;
            }

            CommandDataClientTriggerEffectVibration effectVibration = new CommandDataClientTriggerEffectVibration() {
                controllerType = controllerType,
                position = position,
                amplitude = amplitude,
                frequency = frequency,
            };
            SendIpcCommand(ECommandType.ClientTriggerEffectVibration, effectVibration);
        }
        public void TriggerEffectMultiplePositionFeedback(EVRControllerType controllerType, byte[] strength) {
            if ( !m_running ) {
                return;
            }

            CommandDataClientTriggerEffectMultiplePositionFeedback effectVibration = new CommandDataClientTriggerEffectMultiplePositionFeedback() {
                controllerType = controllerType,
                strength = strength,
            };
            SendIpcCommand(ECommandType.ClientTriggerEffectMultiplePositionFeedback, effectVibration);
        }
        public void TriggerEffectSlopeFeedback(EVRControllerType controllerType, byte startPosition, byte endPosition, byte startStrength, byte endStrength) {
            if ( !m_running ) {
                return;
            }

            CommandDataClientTriggerEffectSlopeFeedback effectVibration = new CommandDataClientTriggerEffectSlopeFeedback() {
                controllerType = controllerType,
                startPosition = startPosition,
                endPosition = endPosition,
                startStrength = startStrength,
                endStrength = endStrength,
            };
            SendIpcCommand(ECommandType.ClientTriggerEffectSlopeFeedback, effectVibration);
        }
        public void TriggerEffectMultiplePositionVibration(EVRControllerType controllerType, byte frequency, byte[] amplitude) {
            if ( !m_running ) {
                return;
            }

            CommandDataClientTriggerEffectMultiplePositionVibration effectVibration = new CommandDataClientTriggerEffectMultiplePositionVibration() {
                controllerType = controllerType,
                frequency = frequency,
                amplitude = amplitude,
            };
            SendIpcCommand(ECommandType.ClientTriggerEffectMultiplePositionVibration, effectVibration);
        }
    }
}
