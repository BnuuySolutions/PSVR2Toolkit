using Avalonia.Controls;
using System;
using System.Runtime.InteropServices;

namespace PSVR2Toolkit.App;

internal class Utilities
{
    [DllImport("dwmapi.dll", PreserveSig = true)]
    private static extern int DwmSetWindowAttribute(IntPtr hwnd, int attr, ref int value, int size);

    private static bool IsWindows11OrNewer() => OperatingSystem.IsWindowsVersionAtLeast(10, 0, 22000);

    // https://docs.avaloniaui.net/troubleshooting/platform-specific-issues/windows#theme-and-appearance
    public static void SetDarkTitleBar(Window window, bool isDark)
    {
        if (!OperatingSystem.IsWindows()) return;
        if (IsWindows11OrNewer()) return; // Not required on Windows 11.
        if (!OperatingSystem.IsWindowsVersionAtLeast(10, 0, 18985)) return;

        nint? handle = window.TryGetPlatformHandle()?.Handle;
        if (handle is null) return;

        int value = isDark ? 1 : 0;
        // Attribute 20: DWMWA_USE_IMMERSIVE_DARK_MODE
        DwmSetWindowAttribute(handle.Value, 20, ref value, sizeof(int));
    }

    public static string GetFwVersionString(uint version)
    {
        byte major = (byte)((version >> 24) & 0xFF);
        byte minor = (byte)((version >> 16) & 0xFF);
        return $"{major:X}.{minor:X2}";
    }

    public static string GetFwVersionStringFull(uint version)
    {
        byte patch = (byte)((version >> 8) & 0xFF);
        byte build = (byte)(version & 0xFF);
        return $"{GetFwVersionString(version)}.{patch:X2}.{build:X2}";
    }
}
