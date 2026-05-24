// C# pseudo-code using LibUsbDotNet
UsbDeviceFinder finder = new UsbDeviceFinder(0x1D6B, 0x0105);
UsbDevice device = UsbDevice.OpenUsbDevice(finder);

// Claim Interface 0
UsbEndpointReader dataReader = device.OpenEndpointReader(ReadEndpointID.Ep01);  // EP1 IN (0x81)
UsbEndpointWriter dataWriter = device.OpenEndpointWriter(WriteEndpointID.Ep01); // EP1 OUT (0x01)

UsbEndpointReader cmdReader = device.OpenEndpointReader(ReadEndpointID.Ep02);  // EP2 IN (0x82)
UsbEndpointWriter cmdWriter = device.OpenEndpointWriter(WriteEndpointID.Ep02); // EP2 OUT (0x02)

// Send Command on EP2
cmdWriter.Write(Encoding.ASCII.GetBytes("START"), 1000, out int written);
byte[] cmdBuf = new byte[64];
cmdReader.Read(cmdBuf, 1000, out int readLen); // Read ACK

// Stream Data on EP1 (High Speed)
byte[] data = new byte[1024 * 1024];
dataWriter.Write(data, 5000, out written);