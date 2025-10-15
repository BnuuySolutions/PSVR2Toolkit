using System.Runtime.InteropServices;

namespace PSVR2Toolkit.CAPI {
    public enum ECommandType : ushort {
        ClientPing, // No command data.
        ServerPong, // No command data.

        ClientRequestHandshake, // CommandDataClientRequestHandshake
        ServerHandshakeResult, // CommandDataServerHandshakeResult

        ClientRequestGazeData, // No command data.
        ServerGazeDataResult, // CommandDataServerGazeDataResult

        ClientTriggerEffectOff, // CommandDataClientTriggerEffectOff
        ClientTriggerEffectFeedback, // CommandDataClientTriggerEffectFeedback
        ClientTriggerEffectWeapon, // CommandDataClientTriggerEffectWeapon
        ClientTriggerEffectVibration, // CommandDataClientTriggerEffectVibration
        ClientTriggerEffectMultiplePositionFeedback, // CommandDataClientTriggerEffectMultiplePositionFeedback
        ClientTriggerEffectSlopeFeedback, // CommandDataClientTriggerEffectSlopeFeedback
        ClientTriggerEffectMultiplePositionVibration, // CommandDataClientTriggerEffectMultiplePositionVibration
    };

    public enum EHandshakeResult : byte {
        Failed,
        Success,
        Outdated,
    };

    public enum EVRControllerType : byte {
        Left,
        Right,
        Both,
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CommandDataClientRequestHandshake {
        public ushort ipcVersion; // The IPC version this client is using.
        public uint processId;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CommandDataServerHandshakeResult {
        public EHandshakeResult result;
        public ushort ipcVersion; // The IPC version the server is using.
    };

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct GazeVector3 {
        public float x, y, z;
    };

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct GazeEyeResult {
        [MarshalAs(UnmanagedType.I1)]
        public bool isGazeOriginValid;
        public GazeVector3 gazeOriginMm;

        [MarshalAs(UnmanagedType.I1)]
        public bool isGazeDirValid;
        public GazeVector3 gazeDirNorm;

        [MarshalAs(UnmanagedType.I1)]
        public bool isPupilDiaValid;
        public float pupilDiaMm;

        [MarshalAs(UnmanagedType.I1)]
        public bool isBlinkValid;
        [MarshalAs(UnmanagedType.I1)]
        public bool blink;
    };

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct GazeEyeResult2 {
        [MarshalAs(UnmanagedType.I1)]
        public bool isGazeOriginValid;
        public GazeVector3 gazeOriginMm;

        [MarshalAs(UnmanagedType.I1)]
        public bool isGazeDirValid;
        public GazeVector3 gazeDirNorm;

        [MarshalAs(UnmanagedType.I1)]
        public bool isPupilDiaValid;
        public float pupilDiaMm;

        [MarshalAs(UnmanagedType.I1)]
        public bool isBlinkValid;
        [MarshalAs(UnmanagedType.I1)]
        public bool blink;

        [MarshalAs(UnmanagedType.I1)]
        public bool isOpenEnabled;
        public float open;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CommandDataServerGazeDataResult {
        public GazeEyeResult leftEye;
        public GazeEyeResult rightEye;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CommandDataServerGazeDataResult2 {
        public GazeEyeResult2 leftEye;
        public GazeEyeResult2 rightEye;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CommandHeader {
        public ECommandType type;
        public int dataLen;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CommandDataClientTriggerEffectOff {
        public EVRControllerType controllerType;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CommandDataClientTriggerEffectFeedback {
        public EVRControllerType controllerType;
        public byte position;
        public byte strength;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CommandDataClientTriggerEffectWeapon {
        public EVRControllerType controllerType;
        public byte startPosition;
        public byte endPosition;
        public byte strength;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CommandDataClientTriggerEffectVibration {
        public EVRControllerType controllerType;
        public byte position;
        public byte amplitude;
        public byte frequency;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CommandDataClientTriggerEffectMultiplePositionFeedback {
        public EVRControllerType controllerType;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
        public byte[] strength;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CommandDataClientTriggerEffectSlopeFeedback {
        public EVRControllerType controllerType;
        public byte startPosition;
        public byte endPosition;
        public byte startStrength;
        public byte endStrength;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct CommandDataClientTriggerEffectMultiplePositionVibration {
        public EVRControllerType controllerType;
        public byte frequency;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
        public byte[] amplitude;
    };
}
