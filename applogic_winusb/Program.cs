using System;
using System.Runtime.InteropServices;
using System.Text;
using Microsoft.Win32.SafeHandles;

class Program
{
    // GUID_DEVINTERFACE_USB_DEVICE
    static readonly Guid GUID_DEVINTERFACE_USB_DEVICE = new Guid("A5DCBF10-6530-11D2-901F-00C04FB951ED");

    const uint GENERIC_WRITE = 0x40000000;
    const uint GENERIC_READ = 0x80000000;
    const uint FILE_SHARE_READ = 0x1;
    const uint FILE_SHARE_WRITE = 0x2;
    const uint OPEN_EXISTING = 3;
    const uint FILE_FLAG_OVERLAPPED = 0x40000000;

    static void Main()
    {
        ushort vid = 0x1D6B;
        ushort pid = 0x0105;
        byte outPipeId = 0x02; // endpoint addresses (default)
        byte inPipeId = 0x82;

        // Parse command-line args: [vid] [pid] [outEp] [inEp] (hex)
        var a = Environment.GetCommandLineArgs();
        if (a.Length >= 3) ushort.TryParse(a[1].Replace("0x", ""), System.Globalization.NumberStyles.HexNumber, null, out vid);
        if (a.Length >= 4) ushort.TryParse(a[2].Replace("0x", ""), System.Globalization.NumberStyles.HexNumber, null, out pid);
        if (a.Length >= 5) outPipeId = Convert.ToByte(a[3], 16);
        if (a.Length >= 6) inPipeId = Convert.ToByte(a[4], 16);

        var devicePath = FindDevicePath(vid, pid);
        if (devicePath == null)
        {
            Console.WriteLine("Device not found");
            return;
        }

        Console.WriteLine($"Found device: {devicePath}");

        using var handle = CreateFile(devicePath, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, IntPtr.Zero, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, IntPtr.Zero);
        if (handle.IsInvalid)
        {
            Console.WriteLine($"Failed to open device: {Marshal.GetLastWin32Error()}");
            return;
        }

        if (!WinUsb_Initialize(handle, out IntPtr winUsbHandle))
        {
            Console.WriteLine("WinUSB initialize failed");
            return;
        }

        // Example: send "START" on EP2 OUT and read ACK from EP2 IN using timeouts/overlapped cancellation
        var cmd = Encoding.ASCII.GetBytes("START");
        if (!WriteWithRetry(winUsbHandle, outPipeId, cmd, 3, out uint bytesWritten))
        {
            Console.WriteLine("Write failed");
            WinUsb_Free(winUsbHandle);
            return;
        }
        Console.WriteLine($"Wrote {bytesWritten} bytes to OUT pipe");

        var buf = new byte[512];
        bool readOk = await ReadWithTimeoutAsync(handle, winUsbHandle, inPipeId, buf, 3000, out uint bytesRead);
        if (readOk)
        {
            Console.WriteLine($"Read {bytesRead} bytes: {Encoding.ASCII.GetString(buf,0,(int)bytesRead)}");
        }
        else
        {
            Console.WriteLine("Read timed out or failed");
        }

        WinUsb_Free(winUsbHandle);
    }

    static bool WriteWithRetry(IntPtr winUsbHandle, byte pipeId, byte[] buffer, int retries, out uint transferred)
    {
        transferred = 0;
        for (int i = 0; i < retries; i++)
        {
            if (WinUsb_WritePipe(winUsbHandle, pipeId, buffer, (uint)buffer.Length, out transferred, IntPtr.Zero)) return true;
            System.Threading.Thread.Sleep(100);
        }
        return false;
    }

    static async Task<bool> ReadWithTimeoutAsync(SafeFileHandle deviceHandle, IntPtr winUsbHandle, byte pipeId, byte[] buffer, int timeoutMs, out uint transferred)
    {
        transferred = 0;
        var t = Task.Run(() => WinUsb_ReadPipe(winUsbHandle, pipeId, buffer, (uint)buffer.Length, out transferred, IntPtr.Zero));
        try
        {
            var completed = await t.WaitAsync(TimeSpan.FromMilliseconds(timeoutMs));
            return completed && t.Result;
        }
        catch (TimeoutException)
        {
            // Attempt to cancel pending io
            CancelIoEx(deviceHandle, IntPtr.Zero);
            try { await t; } catch { }
            return false;
        }
    }

    static string? FindDevicePath(ushort vid, ushort pid)
    {
        IntPtr hDevInfo = SetupDiGetClassDevs(ref GUID_DEVINTERFACE_USB_DEVICE, IntPtr.Zero, IntPtr.Zero, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
        if (hDevInfo == IntPtr.Zero) return null;

        try
        {
            SP_DEVICE_INTERFACE_DATA interfaceData = new SP_DEVICE_INTERFACE_DATA();
            interfaceData.cbSize = Marshal.SizeOf(interfaceData);
            for (uint i = 0; SetupDiEnumDeviceInterfaces(hDevInfo, IntPtr.Zero, ref GUID_DEVINTERFACE_USB_DEVICE, i, ref interfaceData); i++)
            {
                uint requiredSize = 0;
                SetupDiGetDeviceInterfaceDetail(hDevInfo, ref interfaceData, IntPtr.Zero, 0, out requiredSize, IntPtr.Zero);
                IntPtr detailDataBuffer = Marshal.AllocHGlobal((int)requiredSize);
                try
                {
                    // set cbSize (depends on architecture)
                    Marshal.WriteInt32(detailDataBuffer, Marshal.SizeOf<IntPtr>() == 8 ? 8 + 8 : 5 * 4);
                    if (SetupDiGetDeviceInterfaceDetail(hDevInfo, ref interfaceData, detailDataBuffer, requiredSize, out _, IntPtr.Zero))
                    {
                        // structure: cbSize + DevicePath (null-terminated)
                        IntPtr pDevicePathName = IntPtr.Add(detailDataBuffer, 4 + Marshal.SizeOf<IntPtr>());
                        string devicePath = Marshal.PtrToStringAuto(pDevicePathName) ?? string.Empty;
                        if (devicePath.ToLower().Contains($"vid_{vid:x4}") && devicePath.ToLower().Contains($"pid_{pid:x4}"))
                            return devicePath;
                    }
                }
                finally { Marshal.FreeHGlobal(detailDataBuffer); }
            }
        }
        finally { SetupDiDestroyDeviceInfoList(hDevInfo); }
        return null;
    }

    #region PInvoke
    const int DIGCF_PRESENT = 0x00000002;
    const int DIGCF_DEVICEINTERFACE = 0x00000010;

    [StructLayout(LayoutKind.Sequential)]
    struct SP_DEVICE_INTERFACE_DATA
    {
        public int cbSize;
        public Guid InterfaceClassGuid;
        public int Flags;
        public IntPtr Reserved;
    }

    [DllImport("setupapi.dll", SetLastError = true)]
    static extern IntPtr SetupDiGetClassDevs(ref Guid ClassGuid, IntPtr Enumerator, IntPtr hwndParent, uint Flags);

    [DllImport("setupapi.dll", SetLastError = true)]
    static extern bool SetupDiEnumDeviceInterfaces(IntPtr DeviceInfoSet, IntPtr DeviceInfoData, ref Guid InterfaceClassGuid, uint MemberIndex, ref SP_DEVICE_INTERFACE_DATA DeviceInterfaceData);

    [DllImport("setupapi.dll", SetLastError = true)]
    static extern bool SetupDiGetDeviceInterfaceDetail(IntPtr DeviceInfoSet, ref SP_DEVICE_INTERFACE_DATA DeviceInterfaceData, IntPtr DeviceInterfaceDetailData, uint DeviceInterfaceDetailDataSize, out uint RequiredSize, IntPtr DeviceInfoData);

    [DllImport("setupapi.dll", SetLastError = true)]
    static extern bool SetupDiGetDeviceInterfaceDetail(IntPtr DeviceInfoSet, ref SP_DEVICE_INTERFACE_DATA DeviceInterfaceData, IntPtr DeviceInterfaceDetailData, uint DeviceInterfaceDetailDataSize, IntPtr RequiredSize, IntPtr DeviceInfoData);

    [DllImport("setupapi.dll", SetLastError = true)]
    static extern bool SetupDiDestroyDeviceInfoList(IntPtr DeviceInfoSet);

    [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Auto)]
    static extern SafeFileHandle CreateFile(string lpFileName, uint dwDesiredAccess, uint dwShareMode, IntPtr lpSecurityAttributes, uint dwCreationDisposition, uint dwFlagsAndAttributes, IntPtr hTemplateFile);

    [DllImport("winusb.dll", SetLastError = true)]
    static extern bool WinUsb_Initialize(SafeFileHandle DeviceHandle, out IntPtr InterfaceHandle);

    [DllImport("winusb.dll", SetLastError = true)]
    static extern bool WinUsb_Free(IntPtr InterfaceHandle);

    [DllImport("winusb.dll", SetLastError = true)]
    static extern bool WinUsb_WritePipe(IntPtr InterfaceHandle, byte PipeID, byte[] Buffer, uint BufferLength, out uint LengthTransferred, IntPtr Overlapped);

    [DllImport("winusb.dll", SetLastError = true)]
    static extern bool WinUsb_ReadPipe(IntPtr InterfaceHandle, byte PipeID, byte[] Buffer, uint BufferLength, out uint LengthTransferred, IntPtr Overlapped);
    #endregion
}
