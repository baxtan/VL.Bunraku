using Grpc.Core;
using System.Net.Sockets;
using Google.Protobuf;
using Kuka.Ecs.V1;
using Kuka.Motion.External;
using System.Net;

public enum ControlMode
{
    JointPositionControl = 0,
    JointImpedanceControl = 1
}

public class KukaRobotInterface : IDisposable
{
    private ExternalControlService.ExternalControlServiceClient grpcClient;
    private UdpClient udpClient;
    private IPEndPoint robotUdpEndPoint;

    public KukaRobotInterface(string robotGrpcIp, int grpcPort, string robotUdpIp, int udpPort)
    {
        var channel = new Channel($"{robotGrpcIp}:{grpcPort}", ChannelCredentials.Insecure);
        grpcClient = new ExternalControlService.ExternalControlServiceClient(channel);

        robotUdpEndPoint = new IPEndPoint(System.Net.IPAddress.Parse(robotUdpIp), udpPort);
        udpClient = new UdpClient();
        udpClient.Connect(robotUdpEndPoint);
    }

    public void StartControlSession(string clientIp, ControlMode mode, int timeout = 5000, int cycleTime = 4)
    {
        var request = new OpenControlChannelRequest
        {
            IpAddress = clientIp,
            Timeout = (uint)timeout,
            CycleTime = (uint)cycleTime,
            ExternalControlMode = (ExternalControlMode)mode,
            IsSecure = false
        };
        grpcClient.OpenControlChannel(request);
    }

    public void StopControlSession()
    {
        grpcClient.CloseControlChannel(new CloseControlChannelRequest());
    }

    public void SendJointPositions(double[] positions, long ipoc)
    {
        var controlSignal = new ControlSignalExternal
        {
            Header = new ExternalHeader { Ipoc = (uint)ipoc },
            ControlSignal = new ControlSignalProxy
            {
                JointPosition = new JointPositionValues()
            }
        };
        controlSignal.ControlSignal.JointPosition.Positions.AddRange(positions);

        SendControlSignal(controlSignal);
    }

    public void SetJointImpedance(double[] stiffness, double[] damping, long ipoc)
    {
        var impedanceAttributes = new JointImpedanceAttributes();
        impedanceAttributes.Stiffness.AddRange(stiffness);
        impedanceAttributes.Damping.AddRange(damping);

        var controlSignal = new ControlSignalExternal
        {
            Header = new ExternalHeader { Ipoc = ipoc },
            ControlSignal = new ControlSignalProxy
            {
                JointImpedance = impedanceAttributes
            }
        };

        SendControlSignal(controlSignal);
    }

    public void SwitchControlMode(ControlMode mode, long ipoc)
    {
        var controlSignal = new ControlSignalExternal
        {
            Header = new ExternalHeader { Ipoc = ipoc },
            ControlSignal = new ControlSignalProxy
            {
                ControlModeSwitch = new ControlModeSwitch
                {
                    NewMode = (ExternalControlMode)mode
                }
            }
        };

        SendControlSignal(controlSignal);
    }

    private void SendControlSignal(ControlSignalExternal signal)
    {
        byte[] data = signal.ToByteArray();
        udpClient.Send(data, data.Length);
    }

    public MotionStateExternal ReceiveRobotState(int timeoutMs = 1000)
    {
        var endPoint = new IPEndPoint(System.Net.IPAddress.Any, 0);
        udpClient.Client.ReceiveTimeout = timeoutMs;
        try
        {
            byte[] receivedData = udpClient.Receive(ref endPoint);
            return MotionStateExternal.Parser.ParseFrom(receivedData);
        }
        catch (SocketException)
        {
            throw new TimeoutException("Did not receive robot state within timeout.");
        }
    }

    public void Dispose()
    {
        udpClient.Dispose();
    }
}
