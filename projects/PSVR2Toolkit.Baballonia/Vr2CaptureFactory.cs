using Baballonia.SDK;
using Microsoft.Extensions.Logging;

namespace PSVR2Toolkit.Baballonia;

public class Vr2CaptureFactory(ILoggerFactory loggerFactory) : ICaptureFactory
{
    public Capture Create(string address)
    {
        return new Vr2Capture(address, loggerFactory.CreateLogger<Vr2Capture>());
    }

    public bool CanConnect(string address)
    {
        // TODO
        return true;
    }

    public string GetProviderName()
    {
        return "PlayStation VR2";
    }
}
