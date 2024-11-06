package com.example.uavground.mavlink;

import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Parser;
import com.MAVLink.ardupilotmega.msg_ahrs;
import com.MAVLink.ardupilotmega.msg_ekf_status_report;
import com.MAVLink.ardupilotmega.msg_meminfo;
import com.MAVLink.ardupilotmega.msg_rpm;
import com.MAVLink.common.*;
import com.MAVLink.enums.MAV_MOUNT_MODE;
import com.MAVLink.minimal.msg_heartbeat;
import com.alibaba.fastjson2.JSON;
import com.example.uavground.mavlink.dto.VehicleInfoDto;
import com.example.uavground.mavlink.websocket.WebSocketClient;
import io.dronefleet.mavlink.Mavlink2Message;
import io.dronefleet.mavlink.MavlinkConnection;
import io.dronefleet.mavlink.MavlinkMessage;
import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.minimal.MavType;
import io.dronefleet.mavlink.protocol.MavlinkPacket;
import io.dronefleet.mavlink.protocol.MavlinkPacketReader;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

import javax.annotation.PostConstruct;
import javax.annotation.Resource;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

import static com.MAVLink.enums.MAV_CMD.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW;

@Component
public class client {

    @Value("${socket.ip}")
    private String ip = "192.168.1.10";

    @Value("${socket.port}")
    private int port = 8235;

    @Value("${webSocket.url}")
    private String websocketUrl;

    @Resource
    private WebSocketClient webSocketClient;

    // private static VehicleInfoDto vehicleInfoDto = new VehicleInfoDto("00:00:00:00:00:00", 0.0, 0.0, 0.0, 0.0, 0.0);
    private static VehicleInfoDto vehicleInfoDto = new VehicleInfoDto("1", 0.0, 0.0, 0.0, 0.0, 0.0);

    @PostConstruct
    public void init() {
        webSocketClient.connect(websocketUrl);
        getData2(ip, port);
    }

    public void getData(String ip, int port) {
        try {
            Socket socket = new Socket(ip, port);
            MavlinkPacketReader reader = new MavlinkPacketReader(socket.getInputStream());
            MavlinkPacket packet;
            while ((packet = reader.next()) != null) {
                System.out.println(packet);
                // Because of the way that Mavlink is built, you will likely require
                // more infrastructure in order to resolve what each packet means.

                // For the sake of example, we will handle a HEARTBEAT message in a hard-coded,
                // manual fashion.
                if (packet.getMessageId() == 0) { // Message ID 0 is a heartbeat message.

                    // The CRC extra of a heartbeat message is 50 at the time of writing of
                    // this example.
                    System.out.println("heartbeat messag payload: ");

                    if (!packet.validateCrc(50)) {
                        // The CRC check did not pass. This means that we did not read the data
                        // the way that it was intended or sent. We may now drop the packet to
                        // try reading from position+1 (that is, we return the bytes of this
                        // packet back and skip one byte, and try again).
                        reader.drop();

                    } else {
                        // CRC validation passed. This is a valid packet that was read the
                        // way that it was intended to be read.
                    }
                } else {
                    // Generally speaking, it's a good idea to drop packets which message ID is not
                    // understood, because they cannot be validated (therefore if data is corrupted,
                    // there is a chance of losing information).
                    byte[] payload = packet.getPayload();
                    System.out.println("payload: " + new String(payload, StandardCharsets.UTF_8));
                    reader.drop();
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void getData2(String ip, int port) {
        try {
            Parser parser = new Parser();
            Socket socket = new Socket(ip, port);
            InputStream stream = socket.getInputStream();
            int byteRead;
            while ((byteRead = stream.read()) != -1) {
                MAVLinkPacket mavLinkPacket = parser.mavlink_parse_char(byteRead);
                if (mavLinkPacket != null) {
                    System.out.println("mavLinkPacket:  " + mavLinkPacket.toString());
                    switch (mavLinkPacket.msgid) {
                        /** 检测信号消息显示系统或组件存在并正在响应。
                         type 和 autopilot 字段（以及消息组件 id）允许接收系统适当地处理来自该系统的进一步消息（例如，通过基于 autopilot 的用户界面布局）。
                         此微服务记录在 https://mavlink.io/en/services/heartbeat.html
                         * */
                        case msg_heartbeat.MAVLINK_MSG_ID_HEARTBEAT: // 心跳包
                            msg_heartbeat heartbeat = (msg_heartbeat) mavLinkPacket.unpack();
                            System.out.println("msg_heartbeat: " + heartbeat.toString());
                            msg_command_long msg_command_long = new msg_command_long();
                            MAVLinkPacket pack = heartbeat.pack();
                            try {
                                com.MAVLink.common.msg_command_long command = new msg_command_long();
                                // command.command = 21;
                                // 设置云台命令类型
                                command.command = MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW;
                                // 设置云台控制参数
                                command.param1 = 30.0f; // Pitch angle 俯仰角 (度)
                                command.param2 = 45.0f; // Yaw angle 偏航角 (度)
                                command.param3 = 0.5f;  // 俯仰角 (度)
                                command.param4 = 0.5f;  // 偏航角 (度)
                                command.param4 = MAV_MOUNT_MODE.MAV_MOUNT_MODE_MAVLINK_TARGETING; // 控制模式
                                command.confirmation = 0;
                                OutputStream outputStream = socket.getOutputStream();
                                outputStream.write(command.pack().encodePacket());
                                outputStream.flush();
                            } catch (IOException e) {
                                System.out.println("send heart error");
                                e.printStackTrace();
                            }
                            break;
                        /** 常规系统状态。如果系统遵循 MAVLink 标准，则系统状态主要由三种正交状态/模式定义：
                         * 系统模式，即 LOCKED（电机关闭和锁定）、MANUAL（RC 控制下的系统）、GUIDED（具有自主位置控制、位置设定点手动控制的系统）
                         * 或 AUTO（由路径/航路点规划器引导的系统）。NAV_MODE定义了当前的飞行状态：LIFTOFF（通常是开环机动）、着陆、WAYPOINTS 或 VECTOR。这表示内部导航状态机。
                         * 系统状态显示系统当前是否处于活动状态，以及是否发生了紧急情况。在 CRITICAL 和 EMERGENCY 状态下，
                         * MAV 仍被视为活动状态，但应自动启动紧急程序。发生故障后，它应首先从活动状态变为紧急状态，以允许人工干预，然后在一定超时后状态变为紧急状态。
                         **/
                        case msg_sys_status.MAVLINK_MSG_ID_SYS_STATUS:
                            msg_sys_status sys_status = (msg_sys_status) mavLinkPacket.unpack();
                            System.out.println("sys_status: " + sys_status.toString());
                            break;
                        /**
                         * 系统时间是主时钟的时间，通常是主机载计算机的计算机时钟。
                         * */
                        case msg_system_time.MAVLINK_MSG_ID_SYSTEM_TIME:
                            msg_system_time msg_system_time = (msg_system_time) mavLinkPacket.unpack();
                            System.out.println("msg_system_time: " + msg_system_time.toString());
                            break;
                        /**
                         * 全球定位系统 （GPS） 返回的全局位置,
                         * 这不是系统的全局位置估计值，而是 RAW 传感器值。有关全局位置估计，请参阅消息 GLOBAL_POSITION_INT。
                         * */
                        case msg_gps_raw_int.MAVLINK_MSG_ID_GPS_RAW_INT: // GPS
                            msg_gps_raw_int msg_gps_raw_int = (msg_gps_raw_int) mavLinkPacket.unpack();
                            System.out.println("msg_gps_raw_int: " + msg_gps_raw_int.toString());
                            break;

                        /**
                         * 9DOF 传感器的 RAW IMU 读数，由 id（默认 IMU1）标识。此消息应始终包含真实的原始值，不进行任何缩放，以允许数据捕获和系统调试。
                         **/
                        case msg_raw_imu.MAVLINK_MSG_ID_RAW_IMU:
                            msg_raw_imu msg_raw_imu = (msg_raw_imu) mavLinkPacket.unpack();
                            System.out.println("msg_raw_imu: " + msg_raw_imu.toString());
                            break;
                        /**
                         * 一个绝压和差压传感器的典型设置的压力读数。单位与每个字段中的指定相同。
                         **/
                        case msg_scaled_pressure.MAVLINK_MSG_ID_SCALED_PRESSURE: // 压力
                            msg_scaled_pressure msg_scaled_pressure = (msg_scaled_pressure) mavLinkPacket.unpack();
                            System.out.println("msg_scaled_pressure: " + msg_scaled_pressure.toString());
                            break;
                        /**
                         * 航空坐标系中的姿态（右手、Z 轴下、Y 轴右、X 轴前、ZYX、内在）。
                         * */
                        case msg_attitude.MAVLINK_MSG_ID_ATTITUDE: // 姿态
                            msg_attitude msg_attitude = (msg_attitude) mavLinkPacket.unpack();
                            System.out.println("msg_attitude: " + msg_attitude.toString());
                            break;
                        /**
                         *过滤后的全局位置（例如，融合的 GPS 和加速度计）。位置在 GPS 框架中（右手，Z 轴向上）。它
                         *被设计为缩放整数消息，因为 float 的分辨率不够。
                         * */
                        case msg_global_position_int.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // 位置信息
                            msg_global_position_int msg_global_position_int = (msg_global_position_int) mavLinkPacket.unpack();
                            if (msg_global_position_int.lon > 0 && msg_global_position_int.lat > 0 && msg_global_position_int.alt != 0) {
                                vehicleInfoDto.setVehicleLong((double) (msg_global_position_int.lon / 10000000));
                                vehicleInfoDto.setVehicleLat((double) (msg_global_position_int.lat / 10000000));
                                vehicleInfoDto.setVehicleAlt((double) msg_global_position_int.alt / 1000000);
                            } else {
                                vehicleInfoDto.setVehicleLong((double) (msg_global_position_int.lon));
                                vehicleInfoDto.setVehicleLat((double) (msg_global_position_int.lat));
                                vehicleInfoDto.setVehicleAlt((double) msg_global_position_int.alt);
                            }
                            System.out.println("msg_global_position_int: " + msg_global_position_int.toString());
                            break;
                        /**
                         * 被 ACTUATOR_OUTPUT_STATUS 取代。伺服输出的 RAW 值（对于来自遥控器的 RC 输入，请使用 RC_CHANNELS 消息）。
                         * 标准 PPM 调制如下：1000 微秒：0%，2000 微秒：100%。
                         * */
                        case msg_servo_output_raw.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
                            msg_servo_output_raw msg_servo_output_raw = (msg_servo_output_raw) mavLinkPacket.unpack();
                            System.out.println("msg_servo_output_raw: " + msg_servo_output_raw.toString());
                            break;
                        /**
                         * 宣布当前目标任务项的序列号的消息（当任务运行时，系统将飞向/执行该序列号）。
                         * 此消息应始终流式传输 （名义上为 1Hz）。
                         * 此消息应在调用 MAV_CMD_DO_SET_MISSION_CURRENT 或 SET_MISSION_CURRENT 后发出。*/
                        case msg_mission_current.MAVLINK_MSG_ID_MISSION_CURRENT:
                            msg_mission_current msg_mission_current = (msg_mission_current) mavLinkPacket.unpack();
                            System.out.println("msg_mission_current: " + msg_mission_current.toString());
                            break;
                        /**
                         * 导航和位置控制器的状态。
                         * */
                        case msg_nav_controller_output.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
                            msg_nav_controller_output msg_nav_controller_output = (msg_nav_controller_output) mavLinkPacket.unpack();
                            System.out.println("msg_nav_controller_output: " + msg_nav_controller_output.toString());
                            break;
                        /**
                         *接收的 RC 通道的 PPM 值。标准 PPM 调制如下：1000 微秒：0%，2000 微秒：100%。 值 UINT16_MAX 表示通道未使用。单个接收器/发射器可能违反此规范。
                         * */
                        case msg_rc_channels.MAVLINK_MSG_ID_RC_CHANNELS:
                            msg_rc_channels msg_rc_channels = (msg_rc_channels) mavLinkPacket.unpack();
                            System.out.println("msg_rc_channels: " + msg_rc_channels.toString());
                            break;
                        /**
                         * 固定翼飞机的 HUD 上通常显示的指标。
                         * */
                        case msg_vfr_hud.MAVLINK_MSG_ID_VFR_HUD:
                            msg_vfr_hud msg_vfr_hud = (msg_vfr_hud) mavLinkPacket.unpack();
                            vehicleInfoDto.setVehicleSpeed((double) msg_vfr_hud.airspeed);
                            System.out.println("msg_vfr_hud: " + msg_vfr_hud.toString());
                            break;
                        /**
                         *
                         * */
                        case msg_timesync.MAVLINK_MSG_ID_TIMESYNC:
                            msg_timesync msg_timesync = (msg_timesync) mavLinkPacket.unpack();
                            System.out.println("msg_timesync: " + msg_timesync.toString());
                            break;
                        /**
                         * 辅助 9DOF 传感器设置的 RAW IMU 读数。此消息应包含按所述单位缩放的值
                         * */
                        case msg_scaled_imu2.MAVLINK_MSG_ID_SCALED_IMU2:
                            msg_scaled_imu2 msg_scaled_imu2 = (msg_scaled_imu2) mavLinkPacket.unpack();
                            System.out.println("msg_scaled_imu2: " + msg_scaled_imu2.toString());
                            break;
                        /**
                         * 电源状态
                         * */
                        case msg_power_status.MAVLINK_MSG_ID_POWER_STATUS: // 电池
                            msg_power_status msg_power_status = (msg_power_status) mavLinkPacket.unpack();
                            vehicleInfoDto.setVehicleSoc((double) msg_power_status.Vservo);
                            System.out.println("msg_power_status: " + msg_power_status.toString());
                            break;
                        /**
                         * 从无人机流式传输以报告地形图下载的进度（由 TERRAIN_REQUEST 发起），
                         * 或作为对 TERRAIN_CHECK 请求的响应发送。请参阅 terrain 协议文档：https://mavlink.io/en/services/terrain.html
                         * */
                        case msg_terrain_report.MAVLINK_MSG_ID_TERRAIN_REPORT:
                            msg_terrain_report msg_terrain_report = (msg_terrain_report) mavLinkPacket.unpack();
                            System.out.println("msg_terrain_report: " + msg_terrain_report.toString());
                            break;
                        /**
                         *
                         * 电池信息。更新 GCS 和飞行控制器电池状态。智能电池也会使用此消息，但可能会额外发送BATTERY_INFO。
                         * */
                        case msg_battery_status.MAVLINK_MSG_ID_BATTERY_STATUS: // 电池
                            msg_battery_status batteryStatus = (msg_battery_status) mavLinkPacket.unpack();
                            System.out.println("msg_battery_status: " + batteryStatus.toString());
                            break;
                        /**
                         * 自动驾驶 RAM 的状态
                         * */
                        case msg_meminfo.MAVLINK_MSG_ID_MEMINFO:
                            msg_meminfo msg_meminfo = (msg_meminfo) mavLinkPacket.unpack();
                            System.out.println("msg_meminfo: " + msg_meminfo.toString());
                            break;
                        /**
                         * DCM 姿态估计器的状态
                         * */
                        case msg_ahrs.MAVLINK_MSG_ID_AHRS:
                            msg_ahrs msg_ahrs = (msg_ahrs) mavLinkPacket.unpack();
                            System.out.println("msg_ahrs: " + msg_ahrs.toString());
                            break;
                        /**
                         * EKF Status 消息，包括标志和变化。
                         * */
                        case msg_ekf_status_report.MAVLINK_MSG_ID_EKF_STATUS_REPORT:
                            msg_ekf_status_report msg_ekf_status_report = (msg_ekf_status_report) mavLinkPacket.unpack();
                            System.out.println("msg_ekf_status_report: " + msg_ekf_status_report.toString());
                            break;
                        /**
                         * RPM 传感器输出
                         * */
                        case msg_rpm.MAVLINK_MSG_ID_RPM:
                            msg_rpm msg_rpm = (msg_rpm) mavLinkPacket.unpack();
                            System.out.println("msg_rpm: " + msg_rpm.toString());
                            break;
                        /**
                         * 振动水平和加速度计削波 */
                        case msg_vibration.MAVLINK_MSG_ID_VIBRATION:
                            msg_vibration msg_vibration = (msg_vibration) mavLinkPacket.unpack();
                            System.out.println("msg_vibration: " + msg_vibration.toString());
                            break;
                        /**
                         *状态文本消息。这些消息在 QGroundControl 的 COMM 控制台中以黄色打印。
                         *警告： 它们会消耗相当多的带宽，因此仅用于重要的状态和错误消息。如果实施得当，这些消息将缓冲在 MCU 上，
                         *并且仅以有限的速率（例如 10 Hz）发送。
                         * */
                        case msg_statustext.MAVLINK_MSG_ID_STATUSTEXT:
                            msg_statustext msg_statustext = (msg_statustext) mavLinkPacket.unpack();
                            System.out.println("msg_statustext: " + msg_statustext.toString());
                            break;
                        /**
                         * 报告命令的状态。包括命令是否已执行的反馈。命令 microservice
                         * 记录在 https://mavlink.io/en/services/command.html
                         */
                        case msg_command_ack.MAVLINK_MSG_ID_COMMAND_ACK:
                            msg_command_ack msg_command_ack = (msg_command_ack) mavLinkPacket.unpack();
                            System.out.println("msg_command_ack: " + msg_command_ack.toString());
                            break;
                        default:
                            System.out.println("defaultTest: " + mavLinkPacket.unpack().toString());
                            break;
                    }
                    if (vehicleInfoDto != null && verifyData(vehicleInfoDto)) {
                        webSocketClient.sendMessage(JSON.toJSONString(vehicleInfoDto));
                    }
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void getData3(String ip, int port) {
        // This example uses a TCP socket, however we may also use a UDP socket by injecting
        // PipedInputStream/PipedOutputStream to MavlinkConnection, or even USB by using any
        // implementation that will eventually yield an InputStream and an OutputStream.
        try (Socket socket = new Socket(ip, port)) {
            // After establishing a connection, we proceed to building a MavlinkConnection instance.
            MavlinkConnection connection = MavlinkConnection.create(
                    socket.getInputStream(),
                    socket.getOutputStream());

            // Now we are ready to read and send messages.
            MavlinkMessage message;
            while ((message = connection.next()) != null) {
                // The received message could be either a Mavlink1 message, or a Mavlink2 message.
                // To check if the message is a Mavlink2 message, we could do the following:
                if (message instanceof Mavlink2Message) {
                    // This is a Mavlink2 message.
                    Mavlink2Message message2 = (Mavlink2Message) message;

                    if (message2.isSigned()) {
                        // This is a signed message. Let's validate its signature.
                        /*if (message2.validateSignature(mySecretKey)) {
                            // Signature is valid.
                        } else {
                            // Signature validation failed. This message is suspicious and
                            // should not be handled. Perhaps we should log this incident.
                        }*/
                    } else {
                        // This is an unsigned message.
                    }
                } else {
                    // This is a Mavlink1 message.
                }

                // When a message is received, its payload type isn't statically available.
                // We can resolve which kind of message it is by its payload, like so:
                if (message.getPayload() instanceof Heartbeat) {
                    // This is a heartbeat message
                    System.out.println(message.getPayload());

                    MavlinkMessage<Heartbeat> heartbeatMessage = (MavlinkMessage<Heartbeat>) message;
                    int systemId = 101;
                    int componentId = 1;
                    Heartbeat heartbeat = Heartbeat.builder()
                            .type(MavType.MAV_TYPE_GCS)
                            .autopilot(MavAutopilot.MAV_AUTOPILOT_INVALID)
                            .systemStatus(MavState.MAV_STATE_UNINIT)
                            .mavlinkVersion(3)
                            .build();

                    // Write an unsigned heartbeat
                    connection.send2(systemId, componentId, heartbeat);

                } else {
                    System.out.println(message.getPayload());
                }
                // We are better off by publishing the payload to a pub/sub mechanism such
                // as RxJava, JMS or any other favorite instead, though.
            }
            // The stream has ended.
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public boolean verifyData(VehicleInfoDto vehicleInfoDto) {
        if (vehicleInfoDto != null) {
            if (vehicleInfoDto.getVehicleSpeed() > 0 && vehicleInfoDto.getVehicleSoc() > 0) {
                return true;
            }
        }
        return false;
    }
}
