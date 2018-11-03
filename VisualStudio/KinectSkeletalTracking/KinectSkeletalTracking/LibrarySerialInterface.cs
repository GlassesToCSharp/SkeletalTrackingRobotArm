using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KinectSkeletalTracking
{
    public class SerialInterface
    {
        private SerialPort port = null;
        private Action<byte[]> onDataReceivedHandler = null;


        // Singleton Implementation
        private static SerialInterface instance = null;
        private static readonly object padlock = new object();
        public static SerialInterface Instance
        {
            get
            {
                lock (padlock)
                {
                    if (instance == null)
                    {
                        instance = new SerialInterface();
                    }
                    return instance;
                }
            }
        }


        private SerialInterface() { }

        private static void ThrowNewException(string message)
        {
            throw new Exception("[SerialInterface] " + message);
        }

        public void Init()
        {
            Init(3);
        }

        public void Init(int comPort)
        {
            if (comPort > 0)
            {
                port = new SerialPort($"COM{comPort}", 115200, Parity.None, 8, StopBits.One);
                port.DataReceived += Port_DataReceived;
                port.Open();
            }
            else
            {
                ThrowNewException("Invalid COM port defined.");
            }
        }

        private void Port_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            string receivedDataStream = port.ReadExisting();
            byte[] receivedDataBytes = Encoding.UTF8.GetBytes(receivedDataStream);
            onDataReceivedHandler?.Invoke(receivedDataBytes);

            // Show all the incoming data in the port's buffer
            System.Diagnostics.Debug.WriteLine(port.ReadExisting());
        }

        public void Write(string stream)
        {
            if (port == null)
            {
                ThrowNewException("Port is not initialised.");
            }
            else if (!port.IsOpen)
            {
                ThrowNewException("Port is not open.");
            }

            System.Diagnostics.Debug.WriteLine($"Sending data: {stream}");

            port.Write(stream);
        }
    }
}
