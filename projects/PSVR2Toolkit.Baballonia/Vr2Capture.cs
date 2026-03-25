using Microsoft.Extensions.Logging;
using OpenCvSharp;
using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using Capture = Baballonia.SDK.Capture;

namespace PSVR2Toolkit.Baballonia;

public sealed class Vr2Capture(string source, ILogger<Vr2Capture> logger) : Capture(source, logger)
{
    private const string DllName = "customshare.dll";

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void CustomShare_Initialize();

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void CustomShare_GetGazeImage(byte[] buffer);

    private CancellationTokenSource? _cts;
    private Task? _captureTask;

    static Vr2Capture()
    {
        CustomShare_Initialize();
    }

    public override Task<bool> StartCapture()
    {
        IsReady = true;

        _cts = new CancellationTokenSource();
        var token = _cts.Token;

        _captureTask = Task.Run(() => VideoCapture_UpdateLoop(token), token);

        return Task.FromResult(true);
    }

    private async Task VideoCapture_UpdateLoop(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            try
            {
                byte[] buffer = new byte[0x200100];
                CustomShare_GetGazeImage(buffer);
                byte[] img = null!;
                using (var ms = new MemoryStream(buffer))
                using (var reader = new BinaryReader(ms))
                {
                    ms.Seek(4, SeekOrigin.Begin);
                    int len = reader.ReadInt32();
                    ms.Seek(0x100, SeekOrigin.Begin);
                    img = reader.ReadBytes(len - 0x100);
                }
                var mat = new Mat((int)200, (int)400, MatType.CV_8UC1);
                Marshal.Copy(img, 0, mat.Data, img.Length);
                SetRawMat(mat);
                /*if (_device.CaptureFrame(out byte[]? frame))
                {
                    if (frame is { Length: > 0 })
                    {
                        switch (_device.PixelFormat)
                        {
                            case v4l2_pix_fmt.V4L2_PIX_FMT_MJPEG:
                                DecodeMJPEG(frame);
                                break;
                            case v4l2_pix_fmt.V4L2_PIX_FMT_YUYV:
                                var pix = _device.CurrentFormat.pix;
                                DecodeYUYV(frame, pix.width, pix.height);
                                break;
                            default:
                                throw new ArgumentOutOfRangeException();
                        }
                    }
                }
                else*/
                {
                    await Task.Delay(1, ct);
                }
            }
            // catch (TaskCanceledException)
            // {
            //     return;
            // }
            catch (Exception e)
            {
                SetRawMat(new Mat());
                IsReady = false;
                Logger.LogError(e.ToString());
                break;
            }
        }
    }

    public override Task<bool> StopCapture()
    {
        if (_captureTask != null)
        {
            _cts?.Cancel();
            _captureTask.Wait();
        }

        IsReady = false;
        return Task.FromResult(true);
    }
}
