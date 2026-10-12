using BR.ECS.Executor.Device.Domain.Contracts;
using BR.ECS.Executor.Device.Infrastructure;
using System.Collections.Generic;
namespace BR.ECS.DeviceDriver.Centrifuge.renzi_AC380V
{
    public class CommunicationPars : IDeviceCommunication
    {
        public List<DeviceCommunicationDto> DeviceCommunications { get; set; } = new List<DeviceCommunicationDto>
        {
            new DeviceCommunicationDto
            {
                DeviceCommunicationKey = DeviceCommunicationType.SerialPort.ToString(),
                DeviceCommunicationItems = new List<DeviceCommunicationItem>
                {
                    new DeviceCommunicationItem("PortName", "COM10","端口名","", true, true, true,  typeof(int)),
                }
            }
        };
    }
}
