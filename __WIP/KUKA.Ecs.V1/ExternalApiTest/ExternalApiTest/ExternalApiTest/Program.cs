// See https://aka.ms/new-console-template for more information
using Grpc.Net.Client;
using System.Net.Sockets;
using System.Net;
using System.Text;


namespace KUKAprcTest
{
    public class Program
    {
        static async Task Main(string[] args)
        {
            string koniIP = "192.168.1.100";
            //string clientIP = "1.1.1.2";

            

            //Console.WriteLine("ExternalAPI Test: Robot's KONI/XF7 IP is " + koniIP + ". Press any key to execute the GRPC request to StartMonitoring.");
            //Console.ReadKey();

            ////connect to the server at the IP address 1.1.1.1:49335 and call the OpenControlChannel method

            //var channel = GrpcChannel.ForAddress("http://" + koniIP + ":49335");

            //try
            //{
            //    var client = new Kuka.Ecs.V1.ExternalControlService.ExternalControlServiceClient(channel);

            //    //var response = await client.OpenControlChannelAsync(new Kuka.Ecs.V1.OpenControlChannelRequest
            //    //{
            //    //    CycleTime = 4,
            //    //    ExternalControlMode = Kuka.Motion.External.ExternalControlMode.JointPositionControl,
            //    //    IpAddress = clientIP,
            //    //    IsSecure = false,
            //    //    Timeout = 1000
            //    //});

            //    var response = await client.StartMonitoringAsync(new Kuka.Ecs.V1.StartMonitoringRequest
            //    {
            //    });

            //    Console.WriteLine("GRPC call succeeded. Response: " + response.ToString());
            //}
            //catch (Exception e)
            //{
            //    Console.WriteLine("GRPC call failed: " + e.Message);
            //    throw;
            //}

            //Console.WriteLine("Press any key to start listening for multicast messages.");
            //Console.ReadKey();

          

            IPAddress multicastAddress = IPAddress.Parse("239.255.123.250");
            int port = 44446;

            // Create a UDP client
            using (UdpClient udpClient = new UdpClient())
            {
                // Join the multicast group
                udpClient.JoinMulticastGroup(multicastAddress);

                // Bind the UDP client to the local port
                IPEndPoint localEp = new IPEndPoint(IPAddress.Any, port);
                udpClient.Client.Bind(localEp);

                Console.WriteLine($"Listening for multicast messages on {multicastAddress}:{port}...");

                // Listen for multicast messages
                while (true)
                {
                    try
                    {
                        // Receive bytes
                        byte[] bytesReceived = udpClient.Receive(ref localEp);

                        Kuka.Ecs.V1.MotionStateExternal motionStateExternal = Kuka.Ecs.V1.MotionStateExternal.Parser.ParseFrom(bytesReceived);

                        List<double> measuredPositions = motionStateExternal.MotionState.MeasuredPositions.Values.ToList();
                        
                        Console.WriteLine($"Received motion state: {string.Join(", ", measuredPositions)}");
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"An exception occurred: {ex.Message}");
                        break;
                    }
                }
            }

        }
    }
}



