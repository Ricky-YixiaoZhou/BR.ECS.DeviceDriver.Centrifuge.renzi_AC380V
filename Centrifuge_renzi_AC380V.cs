using BR.ECS.Executor.Device.AP.SAR;
using BR.ECS.Executor.Device.Domain.Contracts;
using BR.ECS.Executor.Device.Domain.Share;
using BR.ECS.Executor.Device.Infrastructure;
using System;
using System.Diagnostics.Contracts;
using System.Linq;
using System.Numerics;
using static System.Net.Mime.MediaTypeNames;

namespace BR.ECS.DeviceDriver.Centrifuge.renzi_AC380V
{
    [DeviceClass(brands: "renzi", model: "AC380V", key: "AC380V", deviceType: "Centrifuge", authorName: "周一笑")]
    public class Centrifuge_renzi_AC380V : SARComDeviceBase
    {
        public Centrifuge_renzi_AC380V()
        {
            _serialPortComParam = new SerialPortComParam(9600, 8, 0, 1);
        }

        #region Hex命令集合
        string CMD_ENQUERY_DEVICE_STATE = "01 03 01 00 00 01";
        /*
        01 : 从机地址
        03 : 功能码
        01 : 起始寄存器地址高字节 
        00 : 起始寄存器地址低字节
        00 : 读取寄存器数量高字节
        01 : 读取寄存器数量低字节
        16位地址 = 高8位 + 低8位
        */
        string CMD_OPEN_DOOR = "01 06 08 01 60 00";
        string CMD_ENQUERY_DOOR_STATE = "01 03 01 09 00 01";
        string CMD_SET_POSITION_Read = "01 03 00 0A 00 01";
        string CMD_SLOW_POSITIONING_Rotor = "01 06 08 01 01 00";
        string CMD_QUICK_POSITIONING_Rotor = "01 06 08 01 02 00";
        string CMD_CANCEL_POSITIONING = "01 06 08 01 40 00";
        string CMD_CLOSE_DOOR = "01 06 08 01 70 00";
        string CMD_START_CENTRIFUGE = "01 06 08 00 00 01";
        string CMD_STOP_CENTRIFUGE = "01 06 08 00 00 02";
        string CMD_CLEAN_ERROR = "01 06 08 00 00 04";
        string CMD_Continue_RUNNING_AFTER_ROTOR_RECOGNITION = "01 06 08 00 00 08";
        string CMD_ENQUERY_ERROR_TYPE = "01 03 01 07 00 01";
        string CMD_SET_ROTATE_SPEED = "01 06 00 01 00 01";
        string CMD_SET_TIME_HOUR = "01 06 00 03 00 01";
        string CMD_SET_TIME_MIN = "01 06 00 04 00 01";
        string CMD_SET_TIME_SEC = "01 06 00 05 00 01";
        string CMD_SET_TEMP = "01 06 00 06 00 01";

        #endregion

        #region 私有方法
        /// <summary>
        /// 发送命令并接收响应
        /// </summary>
        /// <param name="command">命令</param>
        /// <returns></returns>
        /// <exception cref="DeviceException"></exception>
        public byte[] SendAndReceiveOne(string command)
        {
            byte[] request = ConstructCompleteFrame(command);
            LogMethodInfo($"发送的数据：[{BitConverter.ToString(request)}]");

            (bool isSuccess, SendAndReceiveModel sendAndReceiveModel) = _sARMachine.SendReceive(request, 5000);

            if (!isSuccess || sendAndReceiveModel == null)
            {
                LogMethodError($"[命令: {command}] 失败");
                throw new Exception($"[命令: {command}] 通信失败或超时");
            }

            byte[] response = sendAndReceiveModel.ReceiveModelDic[1].Content_Bytes;
            LogMethodInfo($"返回的数据：[{BitConverter.ToString(response)}]");

            return response;
        }

        /// <summary>
        /// 构建完整的数据帧
        /// </summary>
        /// <param name="command">命令</param>
        /// <returns></returns>
        /// <exception cref="DeviceException"></exception>
        public static byte[] ConstructCompleteFrame(string command)
        {
            /*Split(' ', ...)：拆成 "01"、"03"、"01"、"00"、"00"、"01"，并忽略多余空格。
                       Convert.ToByte(part, 16)：按十六进制把每一段转成一个字节。
                       ToArray()：得到 byte[] payload = { 0x01, 0x03, 0x01, 0x00, 0x00, 0x01 }*/
            byte[] payload = command.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(part => Convert.ToByte(part, 16)).ToArray();
            /*把 CRC 校验码加到命令末尾，组成完整的 Modbus RTU 发送帧*/
            ushort crc = CalculateCrc(payload);
            byte[] request = new byte[payload.Length + 2];
            Array.Copy(payload, request, payload.Length);
            request[^2] = (byte)crc;         // CRC 低字节 
            request[^1] = (byte)(crc >> 8);  // CRC 高字节
            return request;
        }

        /// <summary>
        /// 计算 CRC 校验码
        /// </summary>
        /// <param name="data">数据</param>
        /// <returns></returns>
        /// <exception cref="DeviceException"></exception>
        public static ushort CalculateCrc(ReadOnlySpan<byte> data)
        {
            ushort crc = 0xFFFF;
            //foreach (byte value in data) //把 data 里的每一个字节依次取出来，每次放到变量 value 里
            //{
            //    crc ^= value; //^= 异或运算 不同的为1 相同为0
            //    for (int bit = 0; bit < 8; bit++) //当前一个 byte 要继续按 bit 处理 8 次。
            //        crc = (crc & 1) != 0 //检查 crc 最低位
            //            ? (ushort)((crc >> 1) ^ 0xA001)
            //            : (ushort)(crc >> 1);
            //}
            foreach (byte value in data)          // 依次取出每一个字节
            {
                crc ^= value;                     // 当前字节和CRC做异或

                for (int bit = 0; bit < 8; bit++) // 一个字节有8个bit，所以处理8次
                {
                    if ((crc & 1) != 0)           // 检查CRC最低位是否为1
                    {
                        crc = (ushort)(crc >> 1); // CRC右移1位
                        crc ^= 0xA001;            // 再和CRC多项式0xA001异或
                    }
                    else
                    {
                        crc = (ushort)(crc >> 1); // 最低位为0，只右移
                    }
                }
            }
            return crc;
        }

        /// <summary>
        /// 校验读取单个寄存器的 Modbus RTU 响应，并返回寄存器值。
        /// </summary>
        private ushort ParseReadSingleRegisterResponse(byte[] frame)
        {
            if (frame == null || frame.Length < 5)
            {
                throw new DeviceException(6001, L["DeviceError"], "离心机响应报文长度不足");
            }

            ushort crc = CalculateCrc(frame.AsSpan(0, frame.Length - 2));
            if (frame[^2] != (byte)crc || frame[^1] != (byte)(crc >> 8))
            {
                throw new DeviceException(6001, L["DeviceError"], "离心机响应报文 CRC 校验失败");
            }

            if (frame[0] != 0x01)
            {
                throw new DeviceException(6001, L["DeviceError"],
                    $"离心机从机地址错误：0x{frame[0]:X2}");
            }

            if ((frame[1] & 0x80) != 0) //正常读取功能码：03
            {
                throw new DeviceException(6001, L["DeviceError"],
                    $"离心机返回 Modbus 异常，功能码：0x{frame[1]:X2}，错误码：0x{frame[2]:X2}");
            }

            if (frame.Length != 7 || frame[1] != 0x03 || frame[2] != 0x02)
            {
                throw new DeviceException(6001, L["DeviceError"],
                    $"离心机读取寄存器响应格式错误：{BitConverter.ToString(frame)}");
            }

            return (ushort)((frame[3] << 8) | frame[4]);
        }

        /// <summary>
        /// 校验写入单个寄存器（功能码 06）的 Modbus RTU 响应。
        /// 功能码 06 的正常响应会原样回显从机地址、寄存器地址和写入值。
        /// </summary>
        /// <param name="frame">离心机返回的完整 Modbus RTU 响应帧。</param>
        /// <param name="expectedRegisterAddress">本次命令期望写入的寄存器地址。</param>
        /// <param name="expectedRegisterValue">本次命令期望写入的寄存器值。</param>
        private void ParseWriteSingleRegisterResponse(byte[] frame,ushort expectedRegisterAddress, ushort expectedRegisterValue)
        {
            if (frame == null || frame.Length < 5)
            {
                throw new DeviceException(6001, L["DeviceError"], "离心机响应报文长度不足");
            }

            ushort crc = CalculateCrc(frame.AsSpan(0, frame.Length - 2));
            if (frame[^2] != (byte)crc || frame[^1] != (byte)(crc >> 8))
            {
                throw new DeviceException(6001, L["DeviceError"], "离心机响应报文 CRC 校验失败");
            }

            if (frame[0] != 0x01)
            {
                throw new DeviceException(6001, L["DeviceError"],
                    $"离心机从机地址错误：0x{frame[0]:X2}");
            }

            if ((frame[1] & 0x80) != 0)
            {
                throw new DeviceException(6001, L["DeviceError"],
                    $"离心机返回 Modbus 异常，功能码：0x{frame[1]:X2}，错误码：0x{frame[2]:X2}");
            }

            if (frame.Length != 8 || frame[1] != 0x06)
            {
                throw new DeviceException(6001, L["DeviceError"],
                    $"离心机写单个寄存器响应格式错误：{BitConverter.ToString(frame)}");
            }

            ushort actualRegisterAddress = (ushort)((frame[2] << 8) | frame[3]);
            ushort actualRegisterValue = (ushort)((frame[4] << 8) | frame[5]);

            if (actualRegisterAddress != expectedRegisterAddress)
            {
                throw new DeviceException(6001, L["DeviceError"],
                    $"离心机回显寄存器地址错误，期望：0x{expectedRegisterAddress:X4}，实际：0x{actualRegisterAddress:X4}");
            }

            if (actualRegisterValue != expectedRegisterValue)
            {
                throw new DeviceException(6001, L["DeviceError"],
                    $"离心机回显寄存器值错误，期望：0x{expectedRegisterValue:X4}，实际：0x{actualRegisterValue:X4}");
            }
        }

        private static string GetPositionStateText(int positionState)
        {
            return positionState switch
            {
                1 => "位置已到达",
                2 => "转子定位中",
                _ => $"未知状态({positionState})"
            };
        }

        private static string GetRunningStateText(int runningState)
        {
            return runningState switch
            {
                0 => "待机中",
                1 => "升速中",
                2 => "降速中",
                3 => "目标转速运行中",
                4 => "停机中",
                5 => "转子识别完成，等待下一步命令",
                _ => $"未知状态({runningState})"
            };
        }

        #endregion

        #region 设备驱动描述方法
        /// <summary>
        /// 查询错误类型
        /// </summary>
        /// <returns>设备状态说明</returns>
        /// <exception cref="DeviceException"></exception>
        [MethodMaintenance(0)]
        [MethodOperations]
        public ushort BY_EnquiryErrorType()
        {
            try
            {
                byte[] response = SendAndReceiveOne(CMD_ENQUERY_ERROR_TYPE);
                ushort statusValue = ParseReadSingleRegisterResponse(response);
                LogMethodInfo($"错误查询指令解析后返回的寄存器的值：[{statusValue}]");
                return statusValue;

            }
            catch (Exception ex)
            {
                throw new Exception($"查询错误类型异常：{ex.Message}");
            }
        }

        /// <summary>
        /// 查询设备状态
        /// </summary>
        /// <returns>设备状态说明</returns>
        /// <exception cref="DeviceException"></exception>
        [MethodMaintenance(1)]
        [MethodOperations]
        public string BY_EnquiryDeviceState() 
        {
            try
            {
                byte[] response = SendAndReceiveOne(CMD_ENQUERY_DEVICE_STATE);

                ushort statusValue = ParseReadSingleRegisterResponse(response);
                LogMethodInfo($"获取仪器指令解析后返回的寄存器的值：[{statusValue}]");

                byte highByte = (byte)(statusValue >> 8);
                byte lowByte = (byte)(statusValue & 0xFF);

                bool hasError = (highByte & 0x80) != 0; // 如果bit7为1，表示存在故障

                int deviceMode = (highByte >> 5) & 0x03;//Bit5和Bit6表示设备模式，00=离线模式，01=联机控制模式，10=远程控制模式，11=保留

                bool isOnlineControlMode = deviceMode == 1;//是否处于联机控制模式
                bool isPositioningModeActive = (highByte & 0x04) != 0;//Bit2表示定位模式是否激活，1=激活，0=未激活
                int positionState = highByte & 0x03;
                int runningState = lowByte & 0x07;
                string stateText;
                if (hasError)
                {
                    BY_EnquiryErrorType(); // 查询错误类型
                    LogMethodError($"查询错误类型: {statusValue}");
                    throw new Exception($"查询错误类型：{statusValue}");
                }

                if (!isOnlineControlMode)
                {
                    LogMethodError($"离心机未处于联机控制模式，当前设备模式值：{deviceMode}");

                    throw new DeviceException(6001, L["DeviceError"],
                        $"离心机未处于联机控制模式，当前设备模式值：{deviceMode}");
                }

                if (!isPositioningModeActive)
                {
                    LogMethodError($"离心机定位模式未激活");
                    throw new DeviceException(6001, L["DeviceError"], "离心机定位模式未激活");
                }


                stateText = "设备状态正常：联机控制模式，定位模式已激活";
                LogMethodInfo(stateText + $"= : isOnlineControlMode = [{ isOnlineControlMode}], isPositioningModeActive = [{ isPositioningModeActive}]");
                //stateText = $"设备状态正常：联机控制模式，定位模式已激活，" +
                //$"定位状态={GetPositionStateText(positionState)}，" +
                //$"运行状态={GetRunningStateText(runningState)}，" +
                //$"P01.00=0x{statusValue:X4}";

                return stateText;
            }
            catch(Exception ex)
            {
                throw new Exception($"查询设备状态异常：{ex.Message}");
            }
        }

        /// <summary>
        /// 查询舱门状态
        /// </summary>
        /// <returns>舱门状态说明</returns>
        /// <exception cref="DeviceException"></exception>
        [MethodMaintenance(2)]
        [MethodOperations]
        public string BY_EnquiryDoorState()
        {
            try
            {
                byte[] response = SendAndReceiveOne(CMD_ENQUERY_DOOR_STATE);
                Thread.Sleep(100); // 等待100毫秒，确保舱门状态稳定
                ushort statusValue = ParseReadSingleRegisterResponse(response);
                LogMethodInfo($"获取仪器指令解析后返回的寄存器的值：[{statusValue}]");

                byte highByte = (byte)(statusValue >> 8);
                byte lowByte = (byte)(statusValue & 0xFF);

                if (lowByte == 0x00)
                {
                    LogMethodInfo($"舱门状态：舱门已打开，P01.09=0x{statusValue:X4}");
                    return "舱门已打开";
                }
                else if(lowByte == 0x01)
                {
                    LogMethodInfo($"舱门状态：舱门已关闭，P01.09=0x{statusValue:X4}");
                    return "舱门已关闭";
                }
                else if (lowByte == 0x02)
                {
                    LogMethodInfo($"舱门状态：舱门正在打开，P01.09=0x{statusValue:X4}");
                    return "舱门正在打开";
                }
                else if (lowByte == 0x03)
                {
                    LogMethodInfo($"舱门状态：舱门正在关闭，P01.09=0x{statusValue:X4}");
                    return "舱门正在关闭";
                }
                else
                {
                    LogMethodInfo($"舱门状态：未知状态，P01.09=0x{statusValue:X4}");
                    return "舱门状态未知";
                }
            }
            catch (Exception ex)
            {
                throw new Exception($"查询舱门状态异常：{ex.Message}");
            }
        }

        /// <summary>
        /// 查询转子状态。
        /// </summary>
        /// <returns>转子定位状态。</returns>
        /// <exception cref="DeviceException"></exception>
        [MethodMaintenance(3)]
        [MethodOperations]
        public string BY_EnquiryRotorState()
        {
            try
            {
                byte[] response = SendAndReceiveOne(CMD_ENQUERY_DEVICE_STATE);
                Thread.Sleep(100); // 等待100毫秒

                ushort statusValue = ParseReadSingleRegisterResponse(response);
                LogMethodInfo($"转子状态寄存器 P01.00=0x{statusValue:X4}");

                byte highByte = (byte)(statusValue >> 8);
                int positionState = highByte & 0x03;

                string stateText = GetPositionStateText(positionState);

                LogMethodInfo($"转子状态：{stateText}，P01.00=0x{statusValue:X4}");
                return stateText;
            }
            catch (Exception ex)
            {
                throw new Exception($"查询转子状态异常：{ex.Message}");
            }
        }

        /// <summary>
        /// 打开舱门
        /// </summary>
        /// <returns>打开舱门</returns>
        /// <exception cref="DeviceException"></exception>
        [MethodMaintenance(4)]
        [MethodOperations]
        public int BY_OpenDoor()//高位60=开舱
        {
            try
            {
                if (BY_EnquiryDoorState() == "舱门已打开")
                {
                    LogMethodInfo("舱门状态：舱门已打开，无需再次打开");
                    return 0;
                } 
                
                byte[] response = SendAndReceiveOne(CMD_OPEN_DOOR);
                Thread.Sleep(100); // 等待100毫秒

                ParseWriteSingleRegisterResponse(response, 0x0801, 0x6000);
                LogMethodInfo("打开舱门命令写入成功：P08.01=0x6000");

                while(BY_EnquiryDoorState() != "舱门已打开") // 查询舱门状态
                {
                    Thread.Sleep(1000); // 等待1秒
                }
                LogMethodInfo("舱门状态：舱门已打开");
                return 0;
            }
            catch (Exception ex)
            {
                throw new Exception($"打开舱门异常：{ex.Message}");
            }
        }

        /// <summary>
        /// 设置目标位置
        /// </summary>
        /// <param name="targetPosition">目标位置编号，允许范围为 1～8。</param>
        /// <returns>置目标位置</returns>
        /// <exception cref="DeviceException"></exception>
        [MethodMaintenance(5)]
        [MethodOperations]
        public string BY_SetTargetPosition(int targetPosition)
        {
            try
            {
                if (targetPosition < 1 || targetPosition > 8)
                {
                    throw new ArgumentOutOfRangeException(
                        nameof(targetPosition),
                        targetPosition,
                        "目标位置编号必须在 1～8 之间");
                }

                ushort targetPositionValue = (ushort)targetPosition;
                string command = $"01 06 00 0A 00 {targetPosition:X2}";
                /*:X：转换为十六进制显示 X2 的含义是：
                   -X：大写十六进制
                   - 2：不足两位时在左边补 0*/

                byte[] response = SendAndReceiveOne(command);
                Thread.Sleep(100); // 等待100毫秒

                ParseWriteSingleRegisterResponse(response, 0x000A, targetPositionValue);
                LogMethodInfo($"目标位置设置成功：P00.0A=0x{targetPositionValue:X4}");
                return $"目标位置 {targetPosition} 设置成功";
            }
            catch (Exception ex)
            {
                throw new Exception($"设置目标位置异常：{ex.Message}");
            }
        }

        /// <summary>
        /// 慢速定位转子
        /// </summary>
        /// <returns>慢速定位转子</returns>
        /// <exception cref="DeviceException"></exception>
        [MethodMaintenance(6)]
        [MethodOperations]
        public string BY_SlowPosRotor()
        {
            try
            {
                byte[] response = SendAndReceiveOne(CMD_SLOW_POSITIONING_Rotor);
                Thread.Sleep(100); // 等待100毫秒

                ParseWriteSingleRegisterResponse(response, 0x0801, 0x0100);
                LogMethodInfo("慢速定位转子命令写入成功：P08.01=0x6000");

                while (BY_EnquiryRotorState() != "位置已到达") // 查询转子状态
                {
                    Thread.Sleep(1000); // 等待1秒
                }
                LogMethodInfo("转子状态：位置已到达，转子慢速定位成功");
                return "转子状态：位置已到达，转子慢速定位成功";
            }
            catch (Exception ex)
            {
                throw new Exception($"转子慢速定位异常：{ex.Message}");
            }
        }

        /// <summary>
        /// 快速定位转子
        /// </summary>
        /// <returns>快速定位转子</returns>
        /// <exception cref="DeviceException"></exception>
        [MethodMaintenance(7)]
        [MethodOperations]
        public string BY_QuickPosRotor()
        {
            try
            {
                byte[] response = SendAndReceiveOne(CMD_QUICK_POSITIONING_Rotor);
                Thread.Sleep(100); // 等待100毫秒

                ParseWriteSingleRegisterResponse(response, 0x0801, 0x0200);
                LogMethodInfo("转子快速定位成功：P08.01=0x0200");
                while (BY_EnquiryRotorState() != "位置已到达") // 查询转子状态
                {
                    Thread.Sleep(1000); // 等待1秒
                }
                LogMethodInfo("转子状态：位置已到达，转子快速定位成功");
                return "转子状态：位置已到达，转子快速定位成功";
            }
            catch (Exception ex)
            {
                throw new Exception($"转子快速定位异常：{ex.Message}");
            }
        }

        /// <summary>
        /// 关闭舱门
        /// </summary>
        /// <returns>关闭舱门</returns>
        /// <exception cref="DeviceException"></exception>
        /// 
        [MethodMaintenance(8)]
        [MethodOperations]
        public int BY_CloseDoor() //高位70=开舱
        {
            try
            {
                if (BY_EnquiryDoorState() == "舱门已关闭")
                {
                    LogMethodInfo("舱门状态：舱门已关闭，无需再次关闭");
                    return 0;
                }

                byte[] response = SendAndReceiveOne(CMD_CLOSE_DOOR);
                Thread.Sleep(100); // 等待100毫秒

                ParseWriteSingleRegisterResponse(response, 0x0801, 0x7000);
                LogMethodInfo("关闭舱门命令写入成功：P08.01=0x7000");

                while (BY_EnquiryDoorState() != "舱门已关闭") // 查询舱门状态
                {
                    Thread.Sleep(1000); // 等待1秒
                }
                LogMethodInfo("舱门状态：舱门已关闭");
                return 0;
            }
            catch (Exception ex)
            {
                throw new Exception($"关闭舱门异常：{ex.Message}");
            }
        }

        /// <summary>
        /// 设置转速-时间-温度
        /// </summary>
        /// <returns>设置转速-时间-温度</returns>
        /// <exception cref="DeviceException"></exception>
        /// 
        [MethodMaintenance(9)]
        [MethodOperations]
        public string BY_SETRUNINFO(string speedpar, string timepar, string temppar)
        {
            try
            {
                if (!int.TryParse(speedpar, out int speed) || speed < 100 || speed > ushort.MaxValue)
                {
                    throw new DeviceException(6001, L["DeviceError"],
                        "转速必须是 100～65535 的整数；实际可设置上限以 P03.00 为准");
                }

                string[] timeParts = timepar?.Split(':') ?? [];
                if (timeParts.Length != 3 ||
                    !int.TryParse(timeParts[0], out int hours) ||
                    !int.TryParse(timeParts[1], out int minutes) ||
                    !int.TryParse(timeParts[2], out int seconds) ||
                    hours < 0 || hours > 99 ||
                    minutes < 0 || minutes > 59 ||
                    seconds < 0 || seconds > 59)
                {
                    throw new DeviceException(6001, L["DeviceError"],
                        "运行时间格式应为 HH:mm:ss，例如 01:30:00；小时范围为 0～99");
                }

                if (!int.TryParse(temppar, out int temperature) ||
                    temperature < 0 || temperature > ushort.MaxValue)
                {
                    throw new DeviceException(6001, L["DeviceError"],
                        "温度必须是 0～65535 的整数；实际可设置范围以 P03.02～P03.03 为准");
                }

                // P00.01=转速；P00.03～P00.05=时、分、秒；P00.06=温度。
                WriteRunParameter(0x0001, (ushort)speed, "转速");
                WriteRunParameter(0x0003, (ushort)hours, "运行时间（时）");
                WriteRunParameter(0x0004, (ushort)minutes, "运行时间（分）");
                WriteRunParameter(0x0005, (ushort)seconds, "运行时间（秒）");
                WriteRunParameter(0x0006, (ushort)temperature, "温度");

                string result = $"运行参数设置成功：转速={speed}，时间={hours:D2}:{minutes:D2}:{seconds:D2}，温度={temperature}";
                LogMethodInfo(result);
                return result;
            }
            catch (Exception ex)
            {
                throw new Exception($"设置运行参数异常：{ex.Message}");
            }
        }

        /// <summary>
        /// 写入一个运行参数。功能码 06 的响应应回显写入的地址和值。
        /// </summary>
        private void WriteRunParameter(ushort registerAddress, ushort registerValue, string parameterName)
        {
            string command = $"01 06 {registerAddress >> 8:X2} {registerAddress & 0xFF:X2} " +
                             $"{registerValue >> 8:X2} {registerValue & 0xFF:X2}";

            byte[] response = SendAndReceiveOne(command);
            ParseWriteSingleRegisterResponse(response, registerAddress, registerValue);
            LogMethodInfo($"{parameterName}写入成功：P{registerAddress >> 8:00}.{registerAddress & 0xFF:X2}=0x{registerValue:X4}");

            // 协议要求主机连续发送两帧数据的间隔不得少于 50 ms。
            Thread.Sleep(100);
        }

        /// <summary>
        /// 启动离心
        /// </summary>
        /// <returns>启动离心</returns>
        /// <exception cref="DeviceException"></exception>
        [MethodMaintenance(10)]
        [MethodOperations]
        public string BY_StartCentrifuge(string speedpar, string timepar, string temppar)
        {
            try
            {   
                //检查 门是否关闭
                if (BY_EnquiryDoorState() != "舱门已关闭")
                {
                    BY_CloseDoor(); // 如果门未关闭，则先关闭门
                }

                BY_SETRUNINFO(speedpar, timepar, temppar);

                byte[] response = SendAndReceiveOne(CMD_START_CENTRIFUGE);
                Thread.Sleep(100); // 等待100毫秒

                ParseWriteSingleRegisterResponse(response, 0x0800, 0x0001);
                LogMethodInfo("启动离心命令写入成功：P08.00=0x0001");

                while (BY_EnquiryDeviceState() != "运行中") // 查询设备状态
                {
                    Thread.Sleep(1000); // 等待1秒
                }
                LogMethodInfo("设备状态：运行中");
                return "运行中";
            }
            catch (Exception ex)
            {
                throw new Exception($"启动离心异常：{ex.Message}");
            }
        }

        /// <summary>
        /// 停止离心
        /// </summary>
        /// <returns>停止离心</returns>
        /// <exception cref="DeviceException"></exception>
        [MethodMaintenance(11)]
        [MethodOperations]
        public string BY_StopCentrifuge()
        {
            try
            {
                byte[] response = SendAndReceiveOne(CMD_STOP_CENTRIFUGE);
                Thread.Sleep(100); // 等待100毫秒

                ParseWriteSingleRegisterResponse(response, 0x0800, 0x0002);
                LogMethodInfo("停止离心命令写入成功：P08.00=0x0002");

                while (BY_EnquiryDeviceState() != "停机中") // 查询设备状态
                {
                    Thread.Sleep(1000); // 等待1秒
                }
                LogMethodInfo("设备状态：停机中");
                return "停机中";
            }
            catch (Exception ex)
            {
                throw new Exception($"停止离心异常：{ex.Message}");
            }
        }


        #endregion

    }
}
