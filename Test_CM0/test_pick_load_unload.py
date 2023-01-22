import pytest
import time
from rf_protocol import *
from robot_rf_action import *

device_msg = {
    "robot_rf": "474E50",
    "getway_ip": "10.0.64.225",
    "aligned_absolute_position": 0,  # 对齐位置
    "leave_box_slot_absolute_position": 0,  # 防撞位置
    "vertical_correction": 0,  # 垂直修正 打开输入 1 不开输入 0
    "moving_direction": "down",  # 在上升列 输入 'up' 在下降列 输入'down'
    "load_weigh": 0,  # 拉箱称重 开启输入 1 不开启 输入 0
    "definition_full_load": 0,  # 满载阈值标定 开启输入 1 不开输入 0
    "check_space": 1  # 关闭抓钩换边检测两面空间 输入 0
    }

pick_robot = robot_rf_action(device_msg["robot_rf"], device_msg["getway_ip"])

class Test_online_chain_homing:

    def test_robot_online(self):
        """机器人上线, 链条回原点
        """

        robot_state = pick_robot.robot_online_action()

        assert robot_state == rf_protocol.ENUM_SHUTTLE_STATE.SHUTTLE_STATE_ONLINE.value["chinese"]

        # pick链条回原点
        wait_state = [
                rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_COMPLETED.value["chinese"], 
                rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_ERROR.value["chinese"]
            ]
        
        robot_state = pick_robot.pick_chain_action(wait_state, rf_protocol.ENUM_HOMING_CMD.HOMING_CMD_RESET_STATE.value)
        
        assert robot_state == rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_COMPLETED.value["chinese"]



class Test_load_unload:

    def test_pick_load_A_box_action(self):
        """pick机器人A面单深拉箱
        """

        load_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"]
            ]

        robot_state = pick_robot.pick_load_function(load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]


        # pick机器人pin_out
        pin_out_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]


        # pick机器人pin_in
        pin_in_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


        # pick机器人A面单深还箱
        unload_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_TRAY_CONNECT_FAIL.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_TRAY_CONNECT_FAIL.value["chinese"]
            ]

        robot_state = pick_robot.pick_load_function(unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_A.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]


        # pick机器人还箱后传感器检查
        unload_sensor_wait_state = [
            rf_protocol.ENUM_PICK_SENSOR_CHECK_UNLOADED.PICK_SENSOR_UNLOAD_STATE_OK.value["chinese"],
            rf_protocol.ENUM_PICK_SENSOR_CHECK_UNLOADED.PICK_SENSOR_UNLOAD_STATE_ERROR.value["chinese"]
            ]

        robot_state = pick_robot.pick_unload_sensor_action(unload_sensor_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_SENSOR_CHECK_UNLOADED.PICK_SENSOR_UNLOAD_STATE_OK.value["chinese"]

    
        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]


        # pick机器人pin_in操作
        wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

    def test_pick_load_B_box_action(self):
        """pick机器人B面单深拉箱
        """

        load_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"]
            ]

        robot_state = pick_robot.pick_load_function(load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]


        # pick机器人pin_out
        pin_out_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]


        # pick机器人pin_in
        pin_in_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


        # pick机器人B面单深还箱
        unload_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_TRAY_CONNECT_FAIL.value["chinese"]
            ]

        robot_state = pick_robot.pick_load_function(unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_B.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]


        # pick机器人还箱后传感器检查
        unload_sensor_wait_state = [
            rf_protocol.ENUM_PICK_SENSOR_CHECK_UNLOADED.PICK_SENSOR_UNLOAD_STATE_OK.value["chinese"],
            rf_protocol.ENUM_PICK_SENSOR_CHECK_UNLOADED.PICK_SENSOR_UNLOAD_STATE_ERROR.value["chinese"]
            ]

        robot_state = pick_robot.pick_unload_sensor_action(unload_sensor_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_SENSOR_CHECK_UNLOADED.PICK_SENSOR_UNLOAD_STATE_OK.value["chinese"]

    
        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]


        # pick机器人pin_in操作
        wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


class Test_load_unload_0xDC:
    """0xDC拉还箱测试
    """
    dc_load_wait_state = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"]
    ]

    dc_unload_wait_state = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]
    ]

    load_wait_state = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]
    ]

    unload_wait_state = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]
    ]    

    pin_out_wait_state = [
        rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"],
        rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
    ]

    pin_in_wait_state = [
        rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
        rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
    ]

    chain_reset_wait_state = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHAIN_POSITION_RECOVER_ERROR.value["chinese"]
    ]
    
    
    def test_pick_load_A_box_action(self):
        # 0xDC load
        robot_state = pick_robot.pick_load_function(self.dc_load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]
        
        # PIN_OUT
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, self.pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]

        # PIN_IN
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

        # 0xDC unload
        robot_state = pick_robot.pick_load_function(self.dc_unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]

        # PIN_OUT
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

        # PIN_IN
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

        # 0xDC unload
        robot_state = pick_robot.pick_load_function(self.dc_unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]

        # 0x06 load
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]

        # 0x06 unload
        robot_state = pick_robot.pick_load_function(self.unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]

        # 0xDC load
        robot_state = pick_robot.pick_load_function(self.dc_load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # 0xDC load
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # 0x06 unload
        robot_state = pick_robot.pick_load_function(self.unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]

        # 0x06 load
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # 0xDC unload
        robot_state = pick_robot.pick_load_function(self.dc_unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]

        # 0xDC chain_reset
        robot_state = pick_robot.pick_load_function(self.chain_reset_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHAIN_POSITION_RECOVER.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]

        # PIN_OUT
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, self.pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]

        # PIN_IN
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

    def test_pick_load_B_box_action(self):
        # 0xDC load
        robot_state = pick_robot.pick_load_function(self.dc_load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]
        
        # PIN_OUT
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, self.pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]

        # PIN_IN
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

        # 0xDC unload
        robot_state = pick_robot.pick_load_function(self.dc_unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]

        # PIN_OUT
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

        # PIN_IN
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

        # 0xDC unload
        robot_state = pick_robot.pick_load_function(self.dc_unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]

        # 0x06 load
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]

        # 0x06 unload
        robot_state = pick_robot.pick_load_function(self.unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]

        # 0xDC load
        robot_state = pick_robot.pick_load_function(self.dc_load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # 0xDC load
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # 0x06 unload
        robot_state = pick_robot.pick_load_function(self.unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]

        # 0x06 load
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # 0xDC unload
        robot_state = pick_robot.pick_load_function(self.dc_unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]

        # 0xDC chain_reset
        robot_state = pick_robot.pick_load_function(self.chain_reset_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHAIN_POSITION_RECOVER.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]

        # PIN_OUT
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, self.pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]

        # PIN_IN
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


class Test_double_load_unload:

    def test_pick_double_load_A_box_action(self):
        """pick机器人A面双深拉还箱
        """

        load_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"]
            ]

        robot_state = pick_robot.pick_load_function(load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]


        # pick机器人pin_out
        pin_out_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]


        # pick机器人pin_in
        pin_in_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


        # pick机器人A面双深还箱
        unload_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_TRAY_CONNECT_FAIL.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABEL_CHAIN_TORQUE_EXCESS.value["chinese"]
            ]

        robot_state = pick_robot.pick_load_function(unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_UNLOAD_TO_A.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]


        # pick机器人还箱后传感器检查
        unload_sensor_wait_state = [
            rf_protocol.ENUM_PICK_SENSOR_CHECK_UNLOADED.PICK_SENSOR_UNLOAD_STATE_OK.value["chinese"],
            rf_protocol.ENUM_PICK_SENSOR_CHECK_UNLOADED.PICK_SENSOR_UNLOAD_STATE_ERROR.value["chinese"]
            ]

        robot_state = pick_robot.pick_unload_sensor_action(unload_sensor_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_SENSOR_CHECK_UNLOADED.PICK_SENSOR_UNLOAD_STATE_OK.value["chinese"]

    
        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]


        # pick机器人pin_in操作
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

    def test_pick_double_load_B_box_action(self):
        """pick机器人B面双深拉还箱
        """

        load_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"]
            ]

        robot_state = pick_robot.pick_load_function(load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_LOAD_FROM_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]


        # pick机器人pin_out
        pin_out_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]


        # pick机器人pin_in
        pin_in_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


        # pick机器人A面单深还箱
        unload_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_TRAY_CONNECT_FAIL.value["chinese"]
            ]

        robot_state = pick_robot.pick_load_function(unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_UNLOAD_TO_B.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]

        # pick机器人还箱后传感器检查
        unload_sensor_wait_state = [
            rf_protocol.ENUM_PICK_SENSOR_CHECK_UNLOADED.PICK_SENSOR_UNLOAD_STATE_OK.value["chinese"],
            rf_protocol.ENUM_PICK_SENSOR_CHECK_UNLOADED.PICK_SENSOR_UNLOAD_STATE_ERROR.value["chinese"]
            ]

        # pick机器人还箱后传感器检查
        robot_state = pick_robot.pick_unload_sensor_action(unload_sensor_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_SENSOR_CHECK_UNLOADED.PICK_SENSOR_UNLOAD_STATE_OK.value["chinese"]

    
        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]


        # pick机器人pin_in操作
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]



class Test_load_recoverable_error:
    """拉箱垂直校准失败
    """

    def test_pick_load_A_box_action(self):
        """pick机器人A面单深拉箱
        """

        load_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"]
            ]

        robot_state = pick_robot.pick_load_function(load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"]

        
        pin_recoverable_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]


        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_recoverable_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


        # pick机器人pin_in
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_recoverable_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


        recoverable_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"]
            ]
        
        # 垂直校准失败恢复
        robot_state = pick_robot.pick_load_function(recoverable_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_RESET_RECOVERABLE_ERROR.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]

        
        pin_out_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]


        pin_in_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        # pick机器人pin_in
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


    def test_pick_load_B_box_action(self):
        """pick机器人B面单深拉箱
        """

        load_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"]
            ]

        robot_state = pick_robot.pick_load_function(load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"]

        
        pin_recoverable_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]


        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_recoverable_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


        # pick机器人pin_in
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_recoverable_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


        recoverable_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"]
            ]

        # 垂直校准失败恢复
        robot_state = pick_robot.pick_load_function(recoverable_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_RESET_RECOVERABLE_ERROR.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]

        
        pin_out_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]


        pin_in_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        # pick机器人pin_in
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


    def test_pick_double_load_A_box_action(self):
        """pick机器人A面双深拉箱
        """

        load_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"]
            ]

        robot_state = pick_robot.pick_load_function(load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"]

        
        pin_recoverable_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]


        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_recoverable_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


        # pick机器人pin_in
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_recoverable_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

        
        recoverable_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"]
            ]

        # 垂直校准失败恢复
        robot_state = pick_robot.pick_load_function(recoverable_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_RESET_RECOVERABLE_ERROR.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]


        pin_out_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]


        pin_in_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        # pick机器人pin_in
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


    def test_pick_double_load_B_box_action(self):
        """pick机器人A面双深拉箱
        """

        load_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"]
            ]

        robot_state = pick_robot.pick_load_function(load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_LOAD_FROM_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"]

        
        pin_recoverable_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]


        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_recoverable_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


        # pick机器人pin_in
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_recoverable_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


        recoverable_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"]
            ]

        # 垂直校准失败恢复
        robot_state = pick_robot.pick_load_function(recoverable_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_RESET_RECOVERABLE_ERROR.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]


        pin_out_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]


        pin_in_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"],
        ]

        # pick机器人pin_in
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


class Test_double_unload_recoverable_error:
    """双深还箱垂直校准失败
    """

    load_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"]
            ]

    unload_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_TRAY_CONNECT_FAIL.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABEL_CHAIN_TORQUE_EXCESS.value["chinese"]
            ]

    pin_out_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"]
        ]

    pin_in_wait_state = [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"],
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"]
        ]

    recoverable_wait_state = [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"]
            ]


    def test_double_unload_recoverable_error_A(self):

        # 单深拉箱
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # 双深还箱
        robot_state = pick_robot.pick_load_function(self.unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_UNLOAD_TO_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"]


        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

        # pick机器人pin_in
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

        
        # 垂直校准失败恢复
        robot_state = pick_robot.pick_load_function(self.recoverable_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_RESET_RECOVERABLE_ERROR.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]


        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, self.pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]

        # pick机器人pin_in
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


        # pick机器人A面单深还箱
        robot_state = pick_robot.pick_load_function(self.recoverable_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_A.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]

    def test_double_unload_recoverable_error_B(self):

        # 单深拉箱
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # 双深还箱
        robot_state = pick_robot.pick_load_function(self.unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_UNLOAD_TO_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"]


        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

        # pick机器人pin_in
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]

        
        # 垂直校准失败恢复
        robot_state = pick_robot.pick_load_function(self.recoverable_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_RESET_RECOVERABLE_ERROR.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]


        # pick机器人pin_out
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_OUT.value, self.pin_out_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"]

        # pick机器人pin_in
        robot_state = pick_robot.pick_pin_action(rf_protocol.ENUM_PICK_PIN_ACTION_CMD.PICK_PIN_ACTION_CMD_IN.value, self.pin_in_wait_state)

        assert robot_state == rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"]


        # pick机器人B面单深还箱
        robot_state = pick_robot.pick_load_function(self.recoverable_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_B.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]


class Test_change_court:
    """抓钩换边
    """

    load_wait_state = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"]
        
        ]

    unload_wait_state = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_TRAY_CONNECT_FAIL.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABEL_CHAIN_TORQUE_EXCESS.value["chinese"],
        ]

    claw_change_wait_state = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_FAIL.value["chinese"]
    ]

    claw_ack_wait_state = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]
    ]

    
    def test_change_court(self):

        # 单深拉箱A
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # b to a 检查两边空间
        robot_state = pick_robot.pick_load_function(self.claw_change_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_TO_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED.value["chinese"]

        # ack
        robot_state = pick_robot.pick_load_function(self.claw_ack_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_ACK.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # a to b 检查两边空间
        robot_state = pick_robot.pick_load_function(self.claw_change_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_TO_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED.value["chinese"]

        # ack
        robot_state = pick_robot.pick_load_function(self.claw_ack_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_ACK.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # b to a 不检查两边空间
        robot_state = pick_robot.pick_load_function(self.claw_change_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_TO_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"],
                                                    device_msg["check_space"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED.value["chinese"]

        # ack
        robot_state = pick_robot.pick_load_function(self.claw_ack_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_ACK.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # a to b 不检查两边空间
        robot_state = pick_robot.pick_load_function(self.claw_change_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_TO_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"],
                                                    device_msg["check_space"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED.value["chinese"]

        # ack
        robot_state = pick_robot.pick_load_function(self.claw_ack_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_ACK.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # 单深还箱A
        robot_state = pick_robot.pick_load_function(self.unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]



class Test_pick_load_A_Timeout:
    """拉还箱超时
    """

    sleep_time = 30
    
    load_halt_wait_state = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADING.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"]
    ]

    unload_halt_wait_state = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_RETURNING.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_TRAY_CONNECT_FAIL.value["chinese"]
    ]

    load_wait_state = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"]
    ]

    unload_wait_state = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"], 
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_TRAY_CONNECT_FAIL.value["chinese"]
    ]

    claw_change_side_halt = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADING.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_FAIL.value["chinese"]
    ]

    claw_change_side = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED.value["chinese"],
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_FAIL.value["chinese"]
    ]

    claw_change_side_ack = [
        rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]
    ]

    halt_action_wait_state = [
        rf_protocol.ENUM_HALT_STATE.HALT_STATE_HALT.value["chinese"]
    ]

    halt_resume_wait_state = [
        rf_protocol.ENUM_HALT_STATE.HALT_STATE_RUNNING.value["chinese"]
    ]

    def test_pick_load_A_Timeout(self):
        """单深拉还箱,超时
        """

        # A面单深拉箱
        robot_state = pick_robot.pick_load_function(self.load_halt_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADING.value["chinese"]

        # 急停
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_HALT.value, self.halt_action_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_HALT.value["chinese"]

        time.sleep(self.sleep_time)

        # 急停恢复
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_RESUME.value, self.halt_resume_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_RUNNING.value["chinese"]

        # 机器人A面单深拉箱
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # 机器人A面单深还箱
        robot_state = pick_robot.pick_load_function(self.unload_halt_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_A.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"], 
                                                    device_msg["vertical_correction"], 
                                                    device_msg["moving_direction"], 
                                                    device_msg["load_weigh"], 
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_RETURNING.value["chinese"]

        # 急停
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_HALT.value, self.halt_action_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_HALT.value["chinese"]

        time.sleep(self.sleep_time)

        # 急停恢复
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_RESUME.value, self.halt_resume_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_RUNNING.value["chinese"]

        # 机器人A面单深还箱
        robot_state = pick_robot.pick_load_function(self.unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_A.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"], 
                                                    device_msg["vertical_correction"], 
                                                    device_msg["moving_direction"], 
                                                    device_msg["load_weigh"], 
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]
        
        # 机器人B面单深拉箱
        robot_state = pick_robot.pick_load_function(self.load_halt_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_B.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"], 
                                                    device_msg["vertical_correction"], 
                                                    device_msg["moving_direction"], 
                                                    device_msg["load_weigh"], 
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADING.value["chinese"]

        # 急停
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_HALT.value, self.halt_action_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_HALT.value["chinese"]

        time.sleep(self.sleep_time)

        # 急停恢复
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_RESUME.value, self.halt_resume_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_RUNNING.value["chinese"]

        # 机器人B面单深拉箱
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_B.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"], 
                                                    device_msg["vertical_correction"], 
                                                    device_msg["moving_direction"], 
                                                    device_msg["load_weigh"], 
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # 机器人B面单深还箱
        robot_state = pick_robot.pick_load_function(self.unload_halt_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_B.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"], 
                                                    device_msg["vertical_correction"], 
                                                    device_msg["moving_direction"], 
                                                    device_msg["load_weigh"], 
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_RETURNING.value["chinese"]

        # 急停
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_HALT.value, self.halt_action_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_HALT.value["chinese"]

        time.sleep(self.sleep_time)

        # 急停恢复
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_RESUME.value, self.halt_resume_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_RUNNING.value["chinese"]

        # 机器人B面单深还箱
        robot_state = pick_robot.pick_load_function(self.unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_B.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"], 
                                                    device_msg["vertical_correction"], 
                                                    device_msg["moving_direction"], 
                                                    device_msg["load_weigh"], 
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]

    def test_pick_double_load_A_Timeout(self):
        
        # 机器人A面双深拉箱
        robot_state = pick_robot.pick_load_function(self.load_halt_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADING.value["chinese"]

        # 急停
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_HALT.value, self.halt_action_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_HALT.value["chinese"]

        time.sleep(self.sleep_time)

        # 急停恢复
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_RESUME.value, self.halt_resume_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_RUNNING.value["chinese"]
        
        # 机器人A面双深拉箱
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        
        # 机器人A面双深还箱
        robot_state = pick_robot.pick_load_function(self.unload_halt_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_UNLOAD_TO_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_RETURNING.value["chinese"]

        # 急停
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_HALT.value, self.halt_action_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_HALT.value["chinese"]

        time.sleep(self.sleep_time)

        # 急停恢复
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_RESUME.value, self.halt_resume_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_RUNNING.value["chinese"]
        
        # 机器人A面双深还箱
        robot_state = pick_robot.pick_load_function(self.unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_UNLOAD_TO_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]


        # 机器人B面双深拉箱
        robot_state = pick_robot.pick_load_function(self.load_halt_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_LOAD_FROM_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADING.value["chinese"]

        # 急停
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_HALT.value, self.halt_action_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_HALT.value["chinese"]

        time.sleep(self.sleep_time)

        # 急停恢复
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_RESUME.value, self.halt_resume_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_RUNNING.value["chinese"]

        # 机器人B面双深拉箱
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_LOAD_FROM_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]


        # 机器人B面双深还箱
        robot_state = pick_robot.pick_load_function(self.unload_halt_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_UNLOAD_TO_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_RETURNING.value["chinese"]

        # 急停
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_HALT.value, self.halt_action_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_HALT.value["chinese"]

        time.sleep(self.sleep_time)

        # 急停恢复
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_RESUME.value, self.halt_resume_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_RUNNING.value["chinese"]

        # 机器人B面双深还箱
        robot_state = pick_robot.pick_load_function(self.unload_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_UNLOAD_TO_B.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]

    def test_pick_change_court(self):
        """换边，超时
        """

        # 机器人A面单深拉箱
        robot_state = pick_robot.pick_load_function(self.load_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_LOAD_FROM_A.value,
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"],
                                                    device_msg["vertical_correction"],
                                                    device_msg["moving_direction"],
                                                    device_msg["load_weigh"],
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]


        # 机器人抓钩 b to a
        robot_state = pick_robot.pick_load_function(self.claw_change_side_halt, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_TO_A.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"], 
                                                    device_msg["vertical_correction"], 
                                                    device_msg["moving_direction"], 
                                                    device_msg["load_weigh"], 
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADING.value["chinese"]

        # 急停
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_HALT.value, self.halt_action_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_HALT.value["chinese"]

        time.sleep(self.sleep_time)

        # 急停恢复
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_RESUME.value, self.halt_resume_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_RUNNING.value["chinese"]

        # 机器人抓钩 b to a
        robot_state = pick_robot.pick_load_function(self.claw_change_side, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_TO_A.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"], 
                                                    device_msg["vertical_correction"], 
                                                    device_msg["moving_direction"], 
                                                    device_msg["load_weigh"], 
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED.value["chinese"]
        
        # 机器人抓钩 ack
        robot_state = pick_robot.pick_load_function(self.claw_change_side_ack, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_ACK.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"], 
                                                    device_msg["vertical_correction"], 
                                                    device_msg["moving_direction"], 
                                                    device_msg["load_weigh"], 
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # 机器人抓钩 a to b
        robot_state = pick_robot.pick_load_function(self.claw_change_side_halt, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_TO_B.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"], 
                                                    device_msg["vertical_correction"], 
                                                    device_msg["moving_direction"], 
                                                    device_msg["load_weigh"], 
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADING.value["chinese"]

        # 急停
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_HALT.value, self.halt_action_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_HALT.value["chinese"]

        time.sleep(self.sleep_time)

        # 急停恢复
        robot_state = pick_robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_RESUME.value, self.halt_resume_wait_state)

        assert robot_state == rf_protocol.ENUM_HALT_STATE.HALT_STATE_RUNNING.value["chinese"]

        # 机器人抓钩 a to b
        robot_state = pick_robot.pick_load_function(self.claw_change_side, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_TO_B.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"], 
                                                    device_msg["vertical_correction"], 
                                                    device_msg["moving_direction"], 
                                                    device_msg["load_weigh"], 
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED.value["chinese"]
        
        # 机器人抓钩 ack
        robot_state = pick_robot.pick_load_function(self.claw_change_side_ack, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CHANGE_CLAW_SIDE_ACK.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"], 
                                                    device_msg["vertical_correction"], 
                                                    device_msg["moving_direction"], 
                                                    device_msg["load_weigh"], 
                                                    device_msg["definition_full_load"])
        
        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]

        # 机器人A面单深还箱
        robot_state = pick_robot.pick_load_function(self.unload_halt_wait_state, 
                                                    rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"],
                                                    rf_protocol.ENUM_PICK_BOX_ACTION.PICK_BOX_ACTION_CMD_UNLOAD_TO_A.value, 
                                                    device_msg["aligned_absolute_position"], 
                                                    device_msg["leave_box_slot_absolute_position"], 
                                                    device_msg["vertical_correction"], 
                                                    device_msg["moving_direction"], 
                                                    device_msg["load_weigh"], 
                                                    device_msg["definition_full_load"])

        assert robot_state == rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_RETURNING.value["chinese"]
