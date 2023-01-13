from rf_protocol import *
from robot_rf_command import *

class robot_rf_action(robot_rf_command):
    
    def __init__(self, robot_addr: str, rf_getway_ip: str):
        """robot_rf_action

        Args:
            robot_addr (str): 机器人的rf地址
            rf_getway_ip (str): 网关的ip地址(字符串)
        """
        self.command = robot_rf_command(robot_addr, rf_getway_ip)

    def robot_online_action(self):
        """机器人上线

        Args:
            
        """
        robot_start = self.command.ping_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PING.value["value"], rf_protocol.ENUM_STATE_CMD.SHUTTLE_STATE_CMD_ONLINE.value)
        
        return robot_start

    def pick_chain_action(self, wait_state: list, action_command: str) -> str:
        """pick链条动作

        Args:
            wait_state (list): 可以退出的状态列表
            action_command (str): 链条动作

        Returns:
            str: 机器人返回的状态
        """
        
        robot_state = self.command.pick_chain_homing_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_CHAIN_HOMING.value["value"], action_command)

        while robot_state not in wait_state:
            robot_state = self.command.pick_chain_homing_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_CHAIN_HOMING.value["value"], action_command)
        
        return robot_state

    def pick_load_function(self, wait_state: list, function_code: int, action_command: int, aligned_absolute_position: int, leave_box_slot_absolute_position: int, vertical_correction: int = 1, 
                            moving_direction: str = "down", load_weigh: int = 0, definition_full_load: int = 0, check_space: int = 1) -> str:
        """拉箱函数

        Args:
            wait_state (list): 可以退出的状态列表
            function_code (int): 功能码
            action_command (int): 要执行的操作
            aligned_absolute_position (int): 对齐位置
            leave_box_slot_absolute_position (int): 防撞位置
            vertical_correction (int): 垂直修正 打开输入 1 不开输入 0
            moving_direction (str): 在上升列 输入 'up' 在下降列 输入'down'
            load_weigh (int): 拉箱称重 开启输入 1 不开启 输入 0
            definition_full_load (int): 满载阈值标定 开启输入 1 不开输入 0
            check_space (int): 关闭抓钩换边检测两面空间 输入 0

        Returns:
            str: 机器人当前状态
        """
        
        pick_box_action = rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"]
        pick_box_action_at = rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"]
        
        function_code = [pick_box_action, pick_box_action_at][(function_code == pick_box_action_at)]

        double_load_a = rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_LOAD_FROM_A.value
        double_unload_a = rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_UNLOAD_TO_A.value
        
        # A ^ up ==> 1 ^ 1 = 0 ==> 9000
        # A ^ down ==> 1 ^ 0 = 1 ==> -9000
        # B ^ down ==> 0 ^ 0 = 0 ==> 9000
        # B ^ up ==> 0 ^ 1 = 1 ==> -9000
        decoupled_absolute_position = [-9000, -9000][(action_command == double_load_a or action_command == double_unload_a) ^ (moving_direction == 'up')]

        additional_info_bit_0 = [1, 0][moving_direction == 'down']
        additional_info_bit_1 = vertical_correction<<1
        if definition_full_load: definition_full_load = (1<<4) + (0<<3) + (1<<2) # 移位后加起来的值，就是开启满载阈值标定的
        additional_info_bit_2_to_4 = (load_weigh<<2) + definition_full_load
        additional_info_bit_5 = check_space<<5
        additional_info = additional_info_bit_5 + additional_info_bit_2_to_4 + additional_info_bit_1 + additional_info_bit_0

        robot_state = self.command.pick_box_action_command(function_code, action_command, decoupled_absolute_position, aligned_absolute_position, 
                                                        leave_box_slot_absolute_position, additional_info)

        while robot_state not in wait_state:
            robot_state = self.command.pick_box_action_command(function_code, action_command, decoupled_absolute_position, aligned_absolute_position, 
                                                            leave_box_slot_absolute_position, additional_info)

        return robot_state

    def pick_unload_sensor_action(self, wait_state: list):
        """pick拉箱完成,传感器检查

        Args:
            wait_state (list): 可以退出的状态列表

        Returns:
            _type_: _description_
        """
        
        robot_state = self.command.pick_unload_sensor_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_SENSOR_CHECK_UNLOADED.value["value"])

        while robot_state not in wait_state:
            robot_state = self.command.pick_unload_sensor_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_SENSOR_CHECK_UNLOADED.value["value"])
        
        return robot_state

    def pick_pin_action(self, action_command: int, wait_state: list):
        """pick PIN 动作

        Args:
            action_command (int): PIN 动作
            wait_state (list): 可以退出的状态列表

        Returns:
            _type_: _description_
        """

        robot_state = self.command.pick_pin_action_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_PIN_ACTION.value["value"], action_command)

        while robot_state not in wait_state:
            robot_state = self.command.pick_pin_action_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_PIN_ACTION.value["value"], action_command)

        return robot_state

    def robot_halt_action(self, action_command: int, wait_state: list):
        """机器人急停

        Args:
            action_command (int): 急停命令
            wait_state (list): 可以退出的状态列表

        Returns:
            _type_: _description_
        """
        robot_state = self.command.robot_halt_action_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_HALT_ACTION.value["value"], action_command)
        
        while robot_state not in wait_state:
            robot_state = self.command.robot_halt_action_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_HALT_ACTION.value["value"], action_command)

        return robot_state

    def sort_load_function(self, wait_state: list, action_command: int, force_unload_flag: int = 0) -> str:
        """sort装卸货函数

        Args:
            wait_state (list): 可以退出的状态列表
            action_command (int): 要执行的操作
            force_unload_flag (int): 强排, 0关闭, 1开启

        Returns:
            str: 机器人返回的状态
        """
        sort_state = self.command.sort_action_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_SORT_ACTION.value["value"], action_command, force_unload_flag)
        
        while sort_state not in wait_state:
            sort_state = self.command.sort_action_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_SORT_ACTION.value["value"], action_command, force_unload_flag)
        
        return sort_state
