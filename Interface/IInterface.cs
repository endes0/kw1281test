﻿using System;

namespace BitFab.KW1281Test.Interface
{
    public interface IInterface : IDisposable
    {
        /// <summary>
        /// Read a byte from the interface.
        /// </summary>
        /// <returns>The byte.</returns>
        byte ReadByte();

        /// <summary>
        /// Write a byte to the interface but do not read/discard its echo.
        /// </summary>
        void WriteByteRaw(byte b);

        void SetBreak(bool on);

        bool CanBitBang();

        void BitBang(byte data, byte delay);

        void ClearReceiveBuffer();

        void SetBaudRate(int baudRate);

        void SetDtr(bool on);

        void SetRts(bool on);
    }
}
