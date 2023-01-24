from dataclasses import dataclass
from pickle import bytes_types
import crcelk

class robot_rf_packet:

    Batch_Number = b'\x77\xf0'
    crc = crcelk.CrcAlgorithm(32, 0x04C11DB7, 'CRC-32/MPEG-2', 0xFFFFFFFF, False, False, 0)
    
    def __init__(self, rf_addr: str):
        """robot_rf_packet

        Args:
            rf_addr (str): 机器人的rf地址
        """
        self.rf_addr = bytearray.fromhex(rf_addr)
        
    def robot_ping_package(self, function_code: int, state_command: int) -> bytes:
        """机器人注册，上线

        Args:
            function_code (int): 功能码
            state_command (int): 状态命令

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        command = state_command.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function + command
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def robot_homing_package(self, function_code: int, homing_command: int) -> bytes:
        """机器人对主轴原点

        Args:
            function_code (int): 功能码
            homing_command (int): 对原点命令

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        command = homing_command.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function + command
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def cm0_moving_package(self, function_code: int, absolute_target: int, velocity_in_counts: int) -> bytes:
        """机器人移动

        Args:
            function_code (int): 功能码
            absolute_target (int): 目标位置
            velocity_in_counts (int): 目标速度

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        position = absolute_target.to_bytes(4, byteorder='little', signed=True)
        velocity = velocity_in_counts.to_bytes(4, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function + position + velocity
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def srg_moving_package(self, function_code: int, absolute_target: int, velocity_in_counts: int, additional_info: int) -> bytes:
        """机器人移动

        Args:
            function_code (int): 功能码
            absolute_target (int): 目标位置
            velocity_in_counts (int): 目标速度
            additional_info (int): 附加信息

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        position = absolute_target.to_bytes(4, byteorder='little', signed=True)
        velocity = velocity_in_counts.to_bytes(4, byteorder='little', signed=False)
        add_info = additional_info.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function + position + velocity + add_info
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def srw_moving_package(self, function_code: int, absolute_target: int, velocity_in_counts: int, additional_info: int, 
                            acceleration_in_counts: int, deceleration_incounts: int) -> bytes:
        """水平车移动

        Args:
            function_code (int): 功能码
            absolute_target (int): 目标位置
            velocity_in_counts (int): 目标速度
            additional_info (int): 附加信息
            acceleration_in_counts (int): 目标加速度
            deceleration_incounts (int): 目标减速度

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        position = absolute_target.to_bytes(4, byteorder='little', signed=True)
        velocity = velocity_in_counts.to_bytes(4, byteorder='little', signed=False)
        add_info = additional_info.to_bytes(1, byteorder='little', signed=False)
        acceleration = acceleration_in_counts.to_bytes(4, byteorder='little', signed=False)
        deceleration = deceleration_incounts.to_bytes(4, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function + position + velocity + add_info + acceleration + deceleration
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data
    
    def sort_action_package(self, function_code: int, sort_action_command: int, force_unload_flag: int) -> bytes:
        """sort 装卸货

        Args:
            function_code (int): 功能码
            sort_action_command (int): 装卸货命令
            force_unload_flag (int): 正常，强制卸货

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        command = sort_action_command.to_bytes(1, byteorder='little', signed=False)
        force_unload = force_unload_flag.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function + command + force_unload
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def robot_moving_cancel_package(self, function_code: int) -> bytes:
        """主轴停止移动

        Args:
            function_code (int): 功能码

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def sort_sensor_state_package(self, function_code: int) -> bytes:
        """查询传感器触发状态状态

        Args:
            function_code (int): 功能码

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def pick_box_action_package(self, function_code: int, pick_action_command: int, decoupled_absolute_position: int, aligned_absolute_position: int, leave_box_slot_absolute_position: int, additional_info: int) -> bytes:
        """pick 拉还箱

        Args:
            function_code (int): 功能码
            pick_action_command (int): 拉还箱命令
            decoupled_absolute_position (int): 脱钩的绝对位置
            aligned_absolute_position (int): 对齐的绝对位置
            leave_box_slot_absolute_position (int): 防撞（不用防撞时就是对齐位置）
            additional_info (int): 附件信息（垂直校正，机器人在上升还是列下降列）

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        command = pick_action_command.to_bytes(1, byteorder='little', signed=False)
        decoupled = decoupled_absolute_position.to_bytes(4, byteorder='little', signed=True)
        aligned = aligned_absolute_position.to_bytes(4, byteorder='little', signed=True)
        leave = leave_box_slot_absolute_position.to_bytes(4, byteorder='little', signed=True)
        additional = additional_info.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function + command + decoupled + aligned + leave + additional
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def pick_pin_action_package(self, function_code: int, pick_pin_command: int) -> bytes:
        """pick机器人 pin in  pin out

        Args:
            function_code (int): 功能码
            pick_pin_command (int): pin命令

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        command = pick_pin_command.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function + command
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def pick_chain_homing_package(self, function_code: int, pick_chain_homing_command: int) -> bytes:
        """pick机器人链条回原点

        Args:
            function_code (int): 功能码
            pick_chain_homing_command (int): 链条命令

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        command = pick_chain_homing_command.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function + command
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def pick_sensor_state_package(self, function_code: int) -> bytes:
        """读pick机器人传感器

        Args:
            function_code (int): 功能码

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def robot_moving_read_package(self, function_code: int) -> bytes:
        """读机器人主电机位置

        Args:
            function_code (int): 功能码

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def pick_chain_direct_control_package(self, function_code: int, direct_control_command: int, absolute_target_position: int) -> bytes_types:
        """pick链条直接控制

        Args:
            function_code (int): 功能码
            direct_control_command (int): 控制命令
            absolute_target_position (int): 绝对目标位置

        Returns:
            bytes_types: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        command = direct_control_command.to_bytes(1, byteorder='little', signed=False)
        position = absolute_target_position.to_bytes(4, byteorder='little', signed=True)
        data = self.Batch_Number + self.rf_addr + function + command + position
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def robot_halt_action_package(self, function_code: int, halt_action_command: int) -> bytes:
        """机器人暂停

        Args:
            function_code (int): 功能码
            halt_action_command (int): 停止动作命令

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        command = halt_action_command.to_bytes(4, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function + command
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def pick_sensor_check_loaded(self, function_code: int) -> bytes:
        """pick机器人拉箱完成传感器检查

        Args:
            function_code (int): 功能码

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def pick_sensor_check_loadeding(self, function_code: int) -> bytes:
        """pick机器人拉箱后传感器检查

        Args:
            function_code (int): 功能码

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def pick_sensor_check_unloaded_loadeding(self, function_code: int) -> bytes:
        """pick机器人还箱后传感器检查

        Args:
            function_code (int): 功能码

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def robot_debug_loadeding(self, function_code: int, debug_command) -> bytes:
        """机器人调试

        Args:
            function_code (int): 功能码
            debug_command (_type_): 调试命令

        Returns:
            bytes: data package
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        command = debug_command.to_bytes(1, byteorder='little', signed=False)
        data = self.Batch_Number + self.rf_addr + function + command
        crc_ret = self.crc.calc_bytes(data[5:]).to_bytes(4, byteorder='big')
        data = data + crc_ret
        return data

    def configuration_loadeding(self, function_code: int, R_W_command: int, index: int, data, ) -> bytes:
        """读，写机器人配置

        Args:
            function_code (int): 功能码
            R_W_command (int): 读，写命令
            index (int): 索引
            data (_type_): 数据

        Returns:
            bytes: _description_
        """
        function = function_code.to_bytes(1, byteorder='little', signed=False)
        command = R_W_command.to_bytes(1, byteorder='little', signed=False)
        index = index.to_bytes(2, byteorder='little', signed=False)
        reserved = b'\x00'
        data = data.to_bytes(4, byteorder='little', signed=False)
        data_loadeding = self.rf_addr + function + command + index + reserved + data
        crc_ret = self.crc.calc_bytes(data_loadeding[5:]).to_bytes(4, byteorder='big')
        data_loadeding = data_loadeding + crc_ret
        return data_loadeding

