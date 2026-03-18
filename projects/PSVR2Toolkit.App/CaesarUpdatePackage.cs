using System;
using System.IO;

namespace PSVR2Toolkit.App;

internal class CaesarUpdatePackage
{
    private const uint CUP_MAGIC = 0x21505543;

    public uint Version { get; private set; }
    public string VersionString => Utilities.GetFwVersionString(Version);
    public string VersionStringFull => Utilities.GetFwVersionStringFull(Version);

    public CaesarUpdatePackage(string filename)
        : this(File.OpenRead(filename))
    { }

    public CaesarUpdatePackage(Stream stream)
        : this(new BinaryReader(stream))
    { }

    public CaesarUpdatePackage(BinaryReader reader)
    {
        if (reader.ReadUInt32() != CUP_MAGIC)
            throw new Exception("Invalid CUP file magic.");

        Version = reader.ReadUInt32();
    }
}
