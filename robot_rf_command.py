import datetime
import socket
import time
from rf_protocol import *
from robot_rf_packet import *

class robot_rf_command(robot_rf_packet):
    
    def __init__(self, robot_addr: str, rf_getway_ip: str):
        """robot_rf_command

        Args:
            robot_addr (str): 机器人的rf地址
            rf_getway_ip (str): 网关的ip地址(字符串)
        """
        self.packet = robot_rf_packet(rf_addr=robot_addr)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.settimeout(0.02)  # 设置udp接收超时时间
        self.udp_address = (rf_getway_ip, 8080)

    def udp_data_parse(self, msg: bytes) -> int:
        """udp数据解析

        Args:
            msg (bytes): 收到的udp数据

        Returns:
            int: 功能码不同，解析出的数据不同
        """
        for func in rf_protocol.ENUM_FUNCTION_CODE:
            if func.value['value'] == msg[5]:
                return func.value['func'](msg)

    def robot_data_processing(self, function_code: int, data_packet: bytes, rf_protocol_ENUM: Enum) -> str:
        """机器人数据处理

        Args:
            function_code (int): 功能码
            data_packet (bytes): 数据包
            rf_protocol_ENUM (Enum): 相关枚举

        Returns:
            _type_: str
        """
        end_time = datetime.datetime.now() + datetime.timedelta(seconds = 5)
        while True:
            try:
                time.sleep(0.02)
                self.udp_socket.sendto(data_packet, self.udp_address)
                recv_msg, _ = self.udp_socket.recvfrom(1024)
                if int(recv_msg[5]) == function_code:
                    end_time = datetime.datetime.now() + datetime.timedelta(seconds = 5)  # 能收到对应的udp消息，刷新结束时间
                    data = self.udp_data_parse(recv_msg)
                    for state in rf_protocol_ENUM:
                        if state.value["value"] == data:
                            print(state.value["chinese"])
                            return state.value["chinese"]
            except Exception as e:
                print('str(e):\t\t', str(e))
                if datetime.datetime.now() > end_time:
                    print(rf_protocol.ENUM_TIME_OUT.RECV_DATA_TIME_OUT.value["chinese"])
                    return rf_protocol.ENUM_TIME_OUT.RECV_DATA_TIME_OUT.value["chinese"]


    def ping_command(self, function_code: int, state_command: int) -> str:
        """机器人ping命令

        Args:
            function_code (int): 功能码
            state_command (int): 状态命令

        Returns:
            str: 机器人状态
        """
        data_packet = self.packet.robot_ping_package(function_code, state_command)
        return self.robot_data_processing(function_code, data_packet, rf_protocol.ENUM_SHUTTLE_STATE)   

    def homing_command(self, function_code: int, homing_command: int) -> str:
        """机器人回原点

        Args:
            function_code (int): 功能码
            homing_command (int): 对原点命令

        Returns:
            str: 回原点状态
        """
        data_packet = self.packet.robot_homing_package(function_code, homing_command)
        return self.robot_data_processing(function_code, data_packet, rf_protocol.ENUM_HOMING_STATE)
        
    def moving_command(self, function_code: int, absolute_target: int, velocity_in_counts: int) -> list:
        """机器人移动命令

        Args:
            function_code (int): 功能码
            absolute_target (int): 目标位置
            velocity_in_counts (int): 目标速度

        Returns:
            str: [当前速度，当前位置，当前扭矩，附加位]
        """
        data_packet = self.packet.robot_moving_package(function_code, absolute_target, velocity_in_counts)
        while True:
            try:
                self.udp_socket.sendto(data_packet, self.udp_address)
                recv_msg, _ = self.udp_socket.recvfrom(1024)
                if recv_msg[5] == function_code:
                    ret_msg = self.udp_data_parse(recv_msg)
                    return ret_msg
            except Exception as e:
                print('str(e):\t\t', str(e))

    def srw_moving_command(self, function_code: int, absolute_target: int, velocity_in_counts: int, additional_info: int, 
                            acceleration_in_counts: int, deceleration_incounts: int) -> list:
        """水平车移动

        Args:
            function_code (int): 功能码
            absolute_target (int): 目标位置
            velocity_in_counts (int): 目标速度
            additional_info (int): 附加信息
            acceleration_in_counts (int): 目标加速度
            deceleration_incounts (int): 目标减速度

        Returns:
            str: [当前位置，当前速度，当前扭矩，附加位]
        """
        data_packet = self.packet.srw_moving_package(function_code, absolute_target, velocity_in_counts, 
                                                        additional_info, acceleration_in_counts, deceleration_incounts)
        while True:
            try:
                self.udp_socket.sendto(data_packet, self.udp_address)
                print(data_packet.hex(":"))
                recv_msg, _ = self.udp_socket.recvfrom(1024)
                print(recv_msg.hex(":"))
                if recv_msg[5] == function_code:
                    ret_msg = self.udp_data_parse(recv_msg)
                    return ret_msg
            except Exception as e:
                print('str(e):\t\t', str(e))

    def sort_action_command(self, function_code: int, sort_action_command: int, force_unload_flag: int) -> str:
        """sort装卸货

        Args:
            function_code (int): 功能码
            sort_action_command (int): 装卸货命令
            force_unload_flag (int): 正常，强制卸货

        Returns:
            str: sort机器人装卸货状态
        """
        data_packet = self.packet.sort_action_package(function_code, sort_action_command, force_unload_flag)
        return self.robot_data_processing(function_code, data_packet, rf_protocol.ENUM_SORT_ACTION_STATE)
        
    def moving_cancel_command(self, function_code: int) -> str:
        """主轴停止移动

        Args:
            function_code (int): 功能码

        Returns:
            str: data package
        """
        data_packet = self.packet.robot_moving_cancel_package(function_code)
        end_time = datetime.datetime.now() + datetime.timedelta(seconds = 3)
        while True:
            self.udp_socket.sendto(data_packet, self.udp_address)
            recv_msg, _ = self.udp_socket.recvfrom(1024)
            if recv_msg[5] == function_code:
                return
            if datetime.datetime.now() > end_time:
                print(rf_protocol.ENUM_TIME_OUT.RECV_DATA_TIME_OUT.value["chinese"])
                return rf_protocol.ENUM_TIME_OUT.RECV_DATA_TIME_OUT.value["chinese"]

    def sort_sensor_state_command(self, function_code: int) -> str:
        """sort传感器触发状态

        Args:
            function_code (int): 功能码

        Returns:
            str: 根据传感器触发状态返回
        """
        data_packet = self.packet.sort_sensor_state_package(self, function_code)
        end_time = datetime.datetime.now() + datetime.timedelta(seconds = 15)  # 结束时间延长到15秒
        while True:
            self.udp_socket.sendto(data_packet, self.udp_address)
            recv_msg, _ = self.udp_socket.recvfrom(1024)
            if recv_msg[5] == function_code:
                # end_time = datetime.datetime.now() + datetime.timedelta(seconds = 3)  # 读传感器，取消刷新结束时间
                sort_sensor_data = self.udp_data_parse(recv_msg)
                for state in rf_protocol.ENUM_SORT_SENSOR_STATE:
                    if state.value["value"] & sort_sensor_data:
                        print(state.value["chinese"])
                        return state.value["chinese"]
                        break
            if datetime.datetime.now() > end_time:
                print(rf_protocol.ENUM_TIME_OUT.RECV_DATA_TIME_OUT.value["chinese"])
                return rf_protocol.ENUM_TIME_OUT.RECV_DATA_TIME_OUT.value["chinese"]
                break

    def pick_box_action_command(self, function_code: int, pick_action_command: int, decoupled_absolute_position: int, aligned_absolute_position: int, leave_box_slot_absolute_position: int, additional_info: int) -> str:
        """pick拉还箱命令

        Args:
            function_code (int): 功能码
            pick_action_command (int): 拉还箱命令
            decoupled_absolute_position (int): 脱钩的绝对位置
            aligned_absolute_position (int): 对齐的绝对位置
            leave_box_slot_absolute_position (int): 防撞（不用防撞时就是对齐位置）
            additional_info (int): 附件信息（垂直校正，机器人在上升还是列下降列）

        Returns:
            str: 根据拉箱状态返回
        """
        data_packet = self.packet.pick_box_action_package(function_code, 
                                                        pick_action_command, 
                                                        decoupled_absolute_position, 
                                                        aligned_absolute_position, 
                                                        leave_box_slot_absolute_position, 
                                                        additional_info)

        return self.robot_data_processing(function_code, data_packet, rf_protocol.ENUM_PICK_BOX_STATES)

    def pick_pin_action_command(self, function_code: int, pick_pin_command: int) -> str:
        """pick机器人 pin in  pin out

        Args:
            function_code (int): 功能码
            pick_pin_command (int): pin命令

        Returns:
            str: 根据PIN状态回复
        """
        data_packet = self.packet.pick_pin_action_package(function_code, pick_pin_command)
        return self.robot_data_processing(function_code, data_packet, rf_protocol.ENUM_PICK_PIN_STATES)

    def pick_chain_homing_command(self, function_code: int, pick_chain_homing_command: int) -> str:
        """pick机器人链条回原点命令

        Args:
            function_code (int): 功能码
            pick_chain_homing_command (int): 链条命令

        Returns:
            str: 链条重置后的状态
        """
        data_packet = self.packet.pick_chain_homing_package(function_code, pick_chain_homing_command)
        return self.robot_data_processing(function_code, data_packet, rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES)

    def pick_sensor_state_command(self, function_code: int) -> str:
        """读pick机器人传感器

        Args:
            function_code (int): 功能码

        Returns:
            str: 传感器的触发状态
        """
        sensor_trigger = []
        data_packet = self.packet.pick_sensor_state_package(function_code)
        end_time = datetime.datetime.now() + datetime.timedelta(seconds = 5)
        while True:
            self.udp_socket.sendto(data_packet, self.udp_address)
            recv_msg, _ = self.udp_socket.recvfrom(1024)
            if recv_msg[5] == function_code:
                sensor_data = self.udp_data_parse(recv_msg)

                # 把触发的传感器存进数组
                for state in rf_protocol.ENUM_PICK_SENSOR_BYTE1_STATE:
                    if state.value["value"] & sensor_data[1]:
                        sensor_trigger.append(state.value["chinese"])

                for state in rf_protocol.ENUM_PICK_SENSOR_BYTE2_STATE:
                    if state.value["value"] & sensor_data[2]:
                        sensor_trigger.append(state.value["chinese"])
                        
                print(sensor_trigger)
                return sensor_trigger

            if datetime.datetime.now() > end_time:
                print(rf_protocol.ENUM_TIME_OUT.RECV_DATA_TIME_OUT.value["chinese"])
                return rf_protocol.ENUM_TIME_OUT.RECV_DATA_TIME_OUT.value

    def pick_chain_direct_control_command(self, function_code: int, direct_control_command: int, absolute_target_position: int) -> str:
        """pick机器人链条之间控制

        Args:
            function_code (int): 功能码
            direct_control_command (int): 命令
            absolute_target_position (int): 位置

        Returns:
            str: 链条位置
        """
        data_packet = self.packet.pick_chain_direct_control_package(function_code, direct_control_command, absolute_target_position)
        while True:
            try:
                self.udp_socket.sendto(data_packet, self.udp_address)
                recv_msg, _ = self.udp_socket.recvfrom(1024)
                if recv_msg[5] == function_code:
                    chain_pos = self.udp_data_parse(recv_msg)

                    return chain_pos
            except Exception as e:
                print('str(e):\t\t', str(e))


    def robot_halt_action_command(self, function_code: int, robot_halt_command: int) -> str:
        """机器人急停

        Args:
            function_code (int): 功能码
            robot_halt_command(int): 急停命令

        Returns:
            str: 机器人返回的状态
        """
        data_packet = self.packet.robot_halt_action_package(function_code, robot_halt_command)
        return self.robot_data_processing(function_code, data_packet, rf_protocol.ENUM_HALT_STATE)

    def pick_unload_sensor_command(self, function_code: int) -> str:
        """pick机器人拉箱后传感器检查

        Args:
            function_code (int): 功能码

        Returns:
            str: _description_
        """
        data_packet = self.packet.pick_sensor_check_loaded(function_code)
        return self.robot_data_processing(function_code, data_packet, rf_protocol.ENUM_PICK_SENSOR_CHECK_UNLOADED)
