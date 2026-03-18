using Avalonia.Controls;
using Avalonia.Styling;
using System;

namespace PSVR2Toolkit.App;

public partial class MainWindow : Window
{
    public MainWindow()
    {
        InitializeComponent();
        SetTitleBarTheme();

        var cup = new CaesarUpdatePackage(@"C:\Users\bnuuy\Documents\PSVR2 fw\HMD2_FIRMWARE_V06_00.CUP");

        versionString.Text = $"Firmware version {cup.VersionString} ({cup.VersionStringFull})";

        ActualThemeVariantChanged += MainWindow_ActualThemeVariantChanged;
    }

    private void MainWindow_ActualThemeVariantChanged(object? sender, EventArgs e)
    {
        SetTitleBarTheme();
    }

    private void SetTitleBarTheme()
    {
        Utilities.SetDarkTitleBar(this, ActualThemeVariant == ThemeVariant.Dark);
    }
}