using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KinectSkeletalTracking
{
    public class SerialInterface
    {
        private const byte minTimeBetweenSend = 100;
        private SerialPort port = null;
        private readonly Stopwatch stopWatch = new Stopwatch();


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

        public bool CanWrite => stopWatch.ElapsedMilliseconds > minTimeBetweenSend;


        private SerialInterface() { }

        private static void ThrowNewException(string message)
        {
            throw new Exception("[SerialInterface] " + message);
        }

        public void Init()
        {
            Init(10);
        }

        public void Init(int comPort)
        {
            if (comPort > 0)
            {
                port = new SerialPort($"COM{comPort}", 115200, Parity.None, 8, StopBits.One);
                port.DataReceived += Port_DataReceived;
                port.Open();
                stopWatch.Start();
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

            // Show all the incoming data in the port's buffer
            Debug.WriteLine(Encoding.ASCII.GetString(receivedDataBytes));
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

            try
            {
                port.Write(stream);
            }
            catch (Exception e)
            {
                port.Close();
                Console.WriteLine("Exception occurred: " + e.Message);
            }
            stopWatch.Restart();

            //Debug.WriteLine($"Data sent at {DateTime.Now}");
        }
    }
}
