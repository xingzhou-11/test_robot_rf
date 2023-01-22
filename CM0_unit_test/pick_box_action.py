from rf_protocol import *
from robot_rf_command import *
import argparse


def pick_load_function(robot_rf_addr: str,getway_ip: str, function_code: int, wait_state: list, action_command: str, 
                        aligned_absolute_position: int, leave_box_slot_absolute_position: int, vertical_correction: int = 0, 
                        moving_direction: str = "down", load_weigh: int = 0, definition_full_load: int = 0, check_space: int = 1) -> str:
    """拉箱函数

    Args:
        robot_rf_addr (str): 机器人RF地址
        getway_ip (str): 网关IP地址
        function_code (int): 功能码
        wait_state (list): 可以退出的状态列表
        action_command (str): 要执行的操作
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

    robot_action = {
                    "read_state": "PICK_BOX_ACTION_CMD_READ_BOX_STATE",
                    "load_a": "PICK_BOX_ACTION_CMD_LOAD_FROM_A",
                    "load_b": "PICK_BOX_ACTION_CMD_LOAD_FROM_B",
                    "unload_a": "PICK_BOX_ACTION_CMD_UNLOAD_TO_A",
                    "unload_b": "PICK_BOX_ACTION_CMD_UNLOAD_TO_B",
                    "double_load_a": "PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_LOAD_FROM_A",
                    "double_load_b": "PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_LOAD_FROM_B",
                    "double_unload_a": "PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_UNLOAD_TO_A",
                    "double_unload_b": "PICK_BOX_ACTION_CMD_DOUBLE_STORAGE_UNLOAD_TO_B",
                    "loaded_to_center": "PICK_BOX_ACTION_CMD_LOADED_TO_CENTER",
                    "center_to_loaded": "PICK_BOX_ACTION_CMD_CENTER_TO_LOADED",
                    "reset_recoverable_error": "PICK_BOX_ACTION_CMD_RESET_RECOVERABLE_ERROR",
                    "reset_error": "PICK_BOX_ACTION_CMD_RESET_LOAD_UNLOAD_ERROR",
                    "claw_to_a": "PICK_BOX_ACTION_CHANGE_CLAW_SIDE_TO_A",
                    "claw_to_b": "PICK_BOX_ACTION_CHANGE_CLAW_SIDE_TO_B",
                    "claw_ack": "PICK_BOX_ACTION_CHANGE_CLAW_SIDE_ACK",
                    "position_recover": "PICK_BOX_ACTION_CHAIN_POSITION_RECOVER"
    }

    pick_robot = robot_rf_command(robot_rf_addr, getway_ip)

    # A ^ up ==> 1 ^ 1 = 0 ==> 9000
    # A ^ down ==> 1 ^ 0 = 1 ==> -9000
    # B ^ down ==> 0 ^ 0 = 0 ==> 9000
    # B ^ up ==> 0 ^ 1 = 1 ==> -9000
    decoupled_absolute_position = [-9000, -9000][(action_command == "double_load_a" or action_command == "double_unload_a") ^ (moving_direction == 'up')]
    additional_info_bit_0 = [1, 0][moving_direction == 'down']
    additional_info_bit_1 = vertical_correction<<1
    if definition_full_load: definition_full_load = (1<<4) + (0<<3) + (1<<2) # 移位后加起来的值，就是开启满载阈值标定的
    additional_info_bit_2_to_4 = (load_weigh<<2) + definition_full_load
    additional_info_bit_5 = check_space<<5

    additional_info = additional_info_bit_5 + additional_info_bit_2_to_4 + additional_info_bit_1 + additional_info_bit_0

    robot_state = pick_robot.pick_box_action_command(function_code, 
                                                    rf_protocol.ENUM_PICK_BOX_ACTION[robot_action[action_command]].value, 
                                                    decoupled_absolute_position, 
                                                    aligned_absolute_position, 
                                                    leave_box_slot_absolute_position, 
                                                    additional_info)

    while robot_state not in wait_state:
        robot_state = pick_robot.pick_box_action_command(function_code, 
                                                    rf_protocol.ENUM_PICK_BOX_ACTION[robot_action[action_command]].value, 
                                                    decoupled_absolute_position, 
                                                    aligned_absolute_position, 
                                                    leave_box_slot_absolute_position, 
                                                    additional_info)

    return robot_state

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description = 'change claw test arguments')
    parser.add_argument("rf_address", help = "rf address", type = str)
    parser.add_argument("gateway_ip", help = "ip address of gateway", type = str)
    parser.add_argument("function_code", help = "function code", type = str)
    parser.add_argument("action_command", help = "action command", type = str)

    args = parser.parse_args()

    wait_state = {
        "read_state": [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADING.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_RETURNING.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CENTERED.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_FAIL.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"]
        ],
        "load_a": [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"]
        ],
        "load_b":[
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"]
        ],
        "unload_a": [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]
        ],
        "unload_b": [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]
        ],
        "double_load_a":[
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_TRAY_CONNECT_FAIL.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"]
        ],
        "double_load_b":[
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"]
        ],
        "double_unload_a":[
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"]
        ],
        "double_unload_b":[
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"]
        ],
        "loaded_to_center": [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CENTERED.value["chinese"]
        ],
        "center_to_loaded": [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]
        ],
        "reset_recoverable_error": [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]
        ],
        "reset_error": [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]
        ],
        "claw_to_a": [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_FAIL.value["chinese"]
        ],
        "claw_to_b": [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_FAIL.value["chinese"]
        ],
        "claw_ack": [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]
        ],
        "position_recover": [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHAIN_POSITION_RECOVER_ERROR.value["chinese"]
        ]
    }

    function_code = [rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION.value["value"], rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_BOX_ACTION_AT_ENTRANCE.value["value"]][(args.function_code == "0xDC")]


    pick_load_function(args.rf_address, args.gateway_ip, function_code, wait_state[args.action_command], args.action_command, 0, 0)
