using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using Newtonsoft.Json;


namespace Dharmik.UDP
{
    public class UDPReceiver : MonoBehaviour
    {
        public int listenPort = 9999;
        public string allowedSenderIP = "192.168.1.161";  // Your sender's IP address

        private UdpClient udpClient;
        private Thread receiveThread;

        public static event Action<UDPMessage> OnUDPMessage;

        void Start()
        {
            StartReceiver();
        }

        public void StartReceiver()
        {
            try
            {
                udpClient = new UdpClient(listenPort);
                receiveThread = new Thread(ReceiveLoop);
                receiveThread.IsBackground = true;
                receiveThread.Start();
                Debug.Log($"[UDP] Listening on port {listenPort}");
            }
            catch (Exception ex)
            {
                Debug.LogError($"[UDP] Start error: {ex.Message}");
            }
        }

        private void ReceiveLoop()
        {
            IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, listenPort);
            try
            {
                while (true)
                {
                    byte[] data = udpClient.Receive(ref remoteEP);
                    string senderIP = remoteEP.Address.ToString();

                    // Filter by allowed IP
                    if (senderIP != allowedSenderIP)
                    {
                        Debug.LogWarning($"[UDP] Ignored packet from {senderIP}");
                        continue;
                    }

                    string json = Encoding.UTF8.GetString(data);
                    

                    try
                    {
                        UDPMessage message = JsonConvert.DeserializeObject<UDPMessage>(json);
                        print(json);
                        UDPDispatcher.Instance.Enqueue(() => OnUDPMessage?.Invoke(message));
                    }
                    catch (Exception ex)
                    {
                        Debug.LogError($"[UDP] JSON parse failed: {ex.Message}");
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"[UDP] Receive loop error: {ex.Message}");
            }
        }

        void OnApplicationQuit()
        {
            receiveThread?.Abort();
            udpClient?.Close();
        }
    }
}