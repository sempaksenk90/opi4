using System;
using System.Text;
using System.Threading.Tasks;
using System.Linq;
using Device.Net;
using Usb.Net;
using Usb.Net.Windows;

class Program
{
    static async Task<int> Main()
    {
        ushort vid = 0x1D6B;
        ushort pid = 0x0105;

        Console.WriteLine("Usb.Net example - searching for device...");

        try
        {
            // Create device filter
            var filter = new FilterDeviceDefinition { VendorId = vid, ProductId = pid };

            // Create factory for Windows USB devices
            using var usbFactory = new UsbDeviceFactory(new[] { filter });

            // Attempt to get the device
            var device = await usbFactory.GetDeviceAsync(filter);
            if (device == null)
            {
                Console.WriteLine("Device not found");
                return 1;
            }

            await device.InitializeAsync();
            Console.WriteLine("Device initialized");

            // Send START command on command endpoint (device mapping handles endpoints)
            var cmd = Encoding.ASCII.GetBytes("START");
            await device.WriteAsync(cmd);
            Console.WriteLine($"Wrote {cmd.Length} bytes (START)");

            // Read ACK (non-blocking with timeout pattern)
            var ackBuf = new byte[64];
            var read = await device.ReadAsync(ackBuf);
            if (read != null && read.Length > 0)
            {
                Console.WriteLine($"Read {read.Length} bytes: {Encoding.ASCII.GetString(read)}");
            }

            // Stream sample data on data endpoint
            var data = Enumerable.Repeat((byte)0xAA, 1024 * 16).ToArray();
            await device.WriteAsync(data);
            Console.WriteLine($"Streamed {data.Length} bytes");

            await device.CloseAsync();
            return 0;
        }
        catch (Exception ex)
        {
            Console.WriteLine("Error: " + ex.Message);
            return 2;
        }
    }
}
