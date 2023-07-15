using System;
using System.Collections.Generic;
using LibUsbDotNet;
using LibUsbDotNet.Main;

namespace BitFab.KW1281Test.Interface
{
    class AMTFlashInterface : IInterface
    {
        private readonly TimeSpan DefaultTimeOut = TimeSpan.FromSeconds(8);

        private readonly AMT _port;

        public AMTFlashInterface(string portName, int baudRate)
        {
            _port = new AMT();
            _port.Open();
            _port.Handshake();
            _port.SetLineProperty(8, AMT.Parity.NONE, AMT.StopBits.ONE, false);
            _port.SetBaudRate(baudRate);
            Log.WriteLine($"AMT Flash version {_port.GetVer()}");
            Log.WriteLine($"AMT Flash voltage {_port.GetVoltage()}");
        }

        public void Dispose()
        {
        }

        public byte ReadByte()
        {
            var data = _port.Read(1);
            if (data.Length == 0)
            {
                throw new TimeoutException();
            }
            return data[0];
        }

        public void WriteByteRaw(byte b)
        {
            var buf = new byte[3] { 0x25, 0x4, b };
            _port.Write(buf);
            var ok = _port.Read(1);
            if (ok[0] != 'U')
            {
                throw new InvalidOperationException($"WriteByteRaw failed. Received {ok[0]}.");
            }
        }

        public void SetBreak(bool on)
        {
            //_port.SetLineProperty(8, AMT.Parity.NONE, AMT.StopBits.ONE, on);
        }

        public bool CanBitBang()
        {
            return true;
        }

        public void BitBang(byte data, byte delay)
        {
            _port.Purge();
            var buf = new byte[4] { 0x25, 0x3, delay, data };
            _port.Write(buf);
            System.Threading.Thread.Sleep(2000);
            /*var ok = _port.Read(1);
            while (ok.Length != 0 && (ok[0] == 0 || ok[0] == 0xFF))
            {
                ok = _port.Read(1);
            }
            if (ok.Length > 0 && ok[0] != 'U')
            {
                Log.WriteLine($"BitBang failed. Received {ok[0]}.");
            } else if (ok.Length == 0)
            {
                Log.WriteLine($"BitBang failed. Received nothing.");
            }*/
        }

        public void ClearReceiveBuffer()
        {
            //_port.Purge();
        }

        public void SetBaudRate(int baudRate)
        {
            _port.SetBaudRate(baudRate);
        }

        public void SetDtr(bool on)
        {
            _port.SetDtr(on);
        }

        public void SetRts(bool on)
        {
            _port.SetRts(on);
        }
    }

    class AMT : IDisposable
    {
        private UsbDevice _device;
        private UsbDeviceFinder _deviceFinder;
        private IUsbDevice _wholeDevice;

        private UsbEndpointReader _reader;
        private UsbEndpointWriter _writer;


        private byte readBitmask;
        private byte writeBitmask;
        private Queue<byte> _readBuffer = new Queue<byte>();

        private readonly int[] _baudFracDivCode = new int[] { 0, 3, 2, 4, 1, 5, 6, 7 };

        public enum Parity : int
        {
            NONE = 0x00 << 8,
            ODD = 0x01 << 8,
            EVEN = 0x02 << 8,
            MARK = 0x03 << 8,
            SPACE = 0x04 << 8
        }

        public enum StopBits : int
        {
            ONE = 0x00 << 11,
            ONE5 = 0x01 << 11,
            TWO = 0x02 << 11
        }


        public AMT()
        {
            _deviceFinder = new UsbDeviceFinder(0x1c43, 0x0500);
        }

        public void Open()
        {
            
            _device = UsbDevice.OpenUsbDevice(_deviceFinder);
            if (_device == null)
            {
                throw new InvalidOperationException("Device not found.");
            }

            _wholeDevice = _device as IUsbDevice;
            if (!ReferenceEquals(_wholeDevice, null))
            {
                _wholeDevice.SetConfiguration(1);
                _wholeDevice.ClaimInterface(0);
            }

            // Get configuration
            var configDesc = _device.Configs[0].Descriptor;

            // Open endpoint 1.
            _reader = _device.OpenEndpointReader(ReadEndpointID.Ep01);
            _writer = _device.OpenEndpointWriter(WriteEndpointID.Ep02);

            PurgeBuffers();
            ResetDevice();
            SetLatencyTimer(1);
        }

        public void Handshake()
        {
            var magic = ReadEEPROM(0x1000, 2);
            if (magic[0] != 0x33)
            {
                throw new InvalidOperationException($"Invalid magic {magic[0]:X2} {magic[1]:X2} received.");
            }

            var bitmasks = ReadEEPROM(0x2000, 2);
            readBitmask = bitmasks[1];
            writeBitmask = bitmasks[0];

            Purge();
            Write(new byte[] { 0x21, 0x55 });
            var response = Read(2);

            // Apply XOR
            for (int i = 0; i < response.Length; i++)
            {
                response[i] ^= 0xFF;
                response[i] ^= 0x33;
            }

            Write(new byte[] { 0x21, 0x56, response[0], response[1] });
            var ok = Read(1);
            if (ok[0] != 0x33)
            {
                throw new InvalidOperationException($"Handshake failed. Received {ok[0]}.");
            }

            WriteEEPROM(0x5001, new byte[] { });

            Purge();
            Write(new byte[] { 0x26, 0x0, 0x1, 0x0, 0x0 });
            ok = Read(1);
            if (ok[0] != 'U')
            {
                throw new InvalidOperationException($"Handshake last phase failed. Received {ok[0]}.");
            }
        }

        public byte[] ReadEEPROM(int address, int length)
        {
            var data = new byte[length];

            var packet = new UsbSetupPacket((byte)UsbRequestType.TypeVendor | 0x80, 0x90, 0x0, address, (ushort)length);
            var received = 0;
            var ok = _device.ControlTransfer(ref packet, data, length, out received);
            if (!ok || received != length)
            {
                throw new InvalidOperationException($"Read EE pos {address} failed.");
            }

            return data;
        }

        public void WriteEEPROM(int address, byte[] data)
        {
            var packet = new UsbSetupPacket((byte)UsbRequestType.TypeVendor | 0x00, 0x91, 0x0, address, (ushort)data.Length);
            var sent = 0;
            var ok = _device.ControlTransfer(ref packet, data, data.Length, out sent);
            if (!ok || sent != data.Length)
            {
                throw new InvalidOperationException($"Write EE pos {address} failed.");
            }
        }

        public void Write(byte[] data)
        {
            // Apply write bitmask
            for (int i = 0; i < data.Length; i++)
            {
                data[i] ^= writeBitmask;
            }

            _write(data);
        }

        public byte[] Read(int length)
        {
            var data = _read(length);

            // Apply read bitmask
            for (int i = 0; i < data.Length; i++)
            {
                data[i] ^= readBitmask;
            }

            return data;
        }

        public void Purge()
        {
            var readed = _read(1, true);
            while (readed.Length > 0)
            {
                readed = _read(1, true);
            }
        }

        public void PurgeBuffers()
        {
            _sendCtl(0x00, 0x01);
            _sendCtl(0x00, 0x02);
        }

        public void ResetDevice()
        {
            _sendCtl(0x00, 0x00);
        }

        public void SetLatencyTimer(int latency)
        {
            _sendCtl(0x09, (byte)latency);
        }

        public void SetDtr(bool on)
        {
            _sendCtl(0x01, (byte)(on ? 0x101 : 0x100));
        }

        public void SetRts(bool on)
        {
            _sendCtl(0x01, (byte)(on ? 0x202 : 0x200));
        }

        public void SetLineProperty(byte databits, Parity parity, StopBits stopbits, bool setBreak)
        {
            int data = 0x00;
            data |= (byte)(databits & 0x0F);
            data |= (int)parity;
            data |= (int)stopbits;
            if (setBreak)
            {
                data |= 0x01 << 14;
            }

            _sendCtl(0x04, data);
        }

        public void SetBaudRate(int baudrate)
        {
            int clock = 3000000;
            int div8 = (int)Math.Round((8 * clock) / (double)baudrate);
            int div = div8 >> 3;
            div |= _baudFracDivCode[div8 & 0x7] << 14;
            int value = div & 0xFFFF;
            int index = (div >> 16) & 0xFFFF;

            var packet = new UsbSetupPacket((byte)UsbRequestType.TypeVendor, 0x3, value, index, 0x0000);
            var ok = _device.ControlTransfer(ref packet, new byte[] {}, 0, out int sent);
            if (!ok)
            {
                throw new InvalidOperationException($"Set Baud failed.");
            }
        }

        public String GetVer()
        {
            Purge();
            Write(new byte[] { 0x22 });
            var len = Read(1)[0];
            var data = Read(len);
            Log.WriteLine($"Got version {data.Length}");
            return System.Text.Encoding.ASCII.GetString(data);
        }

        public float GetVoltage()
        {
            var data = ReadEEPROM(0x3000, 2);
            return ((data[0] << 8) + data[1]) / 52.01f;
        }


        private void _write(byte[] data)
        {
            var written = 0;
            Log.WriteLine($"Write: {BitConverter.ToString(data)}");
            var err = _writer.Write(data, 5000, out written);
            if (err != ErrorCode.None)
            {
                throw new InvalidOperationException($"Write failed with error {err}.");
            }
            if (written != data.Length)
            {
                throw new InvalidOperationException($"Write failed, wrote {written} bytes, expected {data.Length}.");
            }
        }

        private byte[] _read(int length, bool ignoreTimeout = false)
        {
            var finalBuf = new byte[length];
            var pos = 0;

            // Extract from buffer
            while (_readBuffer.Count > 0 && pos < length)
            {
                finalBuf[pos++] = _readBuffer.Dequeue();
            }

            if (pos >= length)
            {
                return finalBuf;
            }

            for (int retrys = 1000; retrys > 0; retrys--)
            {
                var tempBuf = new byte[4096];
                var received = 0;
                var err = _reader.Read(tempBuf, 5000, out received);
                if (err != ErrorCode.None)
                {
                    throw new InvalidOperationException($"Read failed with error {err}.");
                }

                if (received > 2)
                {
                    if (received > length + 2 && !ignoreTimeout)
                    {
                        Log.WriteLine($"Warning: Received {received - 2} bytes, expected {length}.");
                        //throw new InvalidOperationException($"Received {received - 2} bytes, expected {length}.");
                    }
                    if ((int)(tempBuf[1] & 0x8E) != 0)
                    {
                        throw new InvalidOperationException($"Received error bits {tempBuf[1]:X2}.");
                    }

                    int move = (received-2 > length)? length : received - 2;
                    Array.Copy(tempBuf, 2, finalBuf, pos, move);
                    pos += move;

                    if (received > length + 2)
                    {
                        for (int i = length + 2; i < received; i++)
                        {
                            _readBuffer.Enqueue(tempBuf[i]);
                        }
                    }

                    if (pos >= length-1)
                    {
                        Log.WriteLine($"Read : {BitConverter.ToString(finalBuf)}");
                        return finalBuf;
                    } else
                    {
                        retrys = 1000;
                        continue;
                    }
                }

                System.Threading.Thread.Sleep(1);
                
            }

            if (!ignoreTimeout)
            {
                Log.WriteLine("Warning: Read timeout.");   
            }
        
            return new byte[] { };
        }

        private void _sendCtl(byte request, int val, byte[] data = null)
        {
            data ??= new byte[] { };
            var packet = new UsbSetupPacket((byte)UsbRequestType.TypeVendor, request, val, 0x0, 0x0000);
            var ok = _device.ControlTransfer(ref packet, data, data.Length, out var _);
            if (!ok)
            {
                throw new InvalidOperationException($"Control transfer {request:X2} failed.");
            }
        }

        public void Dispose()
        {
        }
    }
}
