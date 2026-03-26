using System.Runtime.InteropServices;

namespace PSVR2Toolkit;

internal class CAPI
{
    private const string DllName = "PSVR2Toolkit.CAPI.dll";

    /// <summary>
    /// Initializes CAPI for usage, required to call other functions.
    /// </summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void CAPI_Initialize();

    /// <summary>
    /// Gets the current gaze image and loads it into the image buffer.
    /// </summary>
    /// <param name="imageBuffer">An array to load the current gaze image into, must be 0x200100 bytes in length.</param>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void CAPI_GetGazeImage(byte[] imageBuffer);
}
