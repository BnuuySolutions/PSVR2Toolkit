using Microsoft.Extensions.Logging;
using OpenCvSharp;
using System;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using Capture = Baballonia.SDK.Capture;

namespace PSVR2Toolkit.Baballonia;

public sealed class Vr2Capture(string source, ILogger<Vr2Capture> logger) : Capture(source, logger)
{
    private const int IMAGE_WIDTH = 400;
    private const int IMAGE_HEIGHT = 200;
    private const int IMAGE_HEADER_SIZE = 0x100;
    // The image from the buffer is BC4, there is only one channel.
    private const int IMAGE_DATA_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT;

    private byte[] _imageBuffer = new byte[0x200100];

    private CancellationTokenSource? _cts;
    private Task? _captureTask;

    static Vr2Capture()
    {
        CAPI.CAPI_Initialize();
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
                CAPI.CAPI_GetGazeImage(_imageBuffer);

                // Check for VI in header.
                if (_imageBuffer[0] == 0x56 && _imageBuffer[1] == 0x49)
                {
                    var mat = new Mat(IMAGE_HEIGHT, IMAGE_WIDTH, MatType.CV_8UC1);
                    // We can skip the header when copying the image data to the matrix data.
                    Marshal.Copy(_imageBuffer, IMAGE_HEADER_SIZE, mat.Data, IMAGE_DATA_SIZE);
                    SetRawMat(mat);
                }
                else
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
