using Baballonia.SDK;
using Microsoft.Extensions.Logging;
using System.Threading.Tasks;

namespace PSVR2Toolkit.Baballonia;

public sealed class Vr2Capture(string source, ILogger<Vr2Capture> logger) : Capture(source, logger)
{
    public override Task<bool> StartCapture()
    {
        throw new System.NotImplementedException();
    }

    public override Task<bool> StopCapture()
    {
        throw new System.NotImplementedException();
    }
}
