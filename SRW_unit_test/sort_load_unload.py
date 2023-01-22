from rf_protocol import *
from robot_rf_action import *
import argparse
import signal
import logging
import datetime

def LOG(log_file_name:str, log_msg: str):
    logging.basicConfig(filename=log_file_name, level=logging.INFO)
    logging.info(log_msg)

def sort_action(robot_rf: str, getway_ip: str, wait_state: list, action_command: int, force_unload_flag: int):
    """sort机器人装卸货

    Args:
        rf_address (str): 机器人rf地址
        gateway_ip (str): 网关IP地址
        action_command (int): 要进行的命令
        force_unload_flag (int): 附加信息
    """

    

    sort_robot = robot_rf_command(robot_rf, getway_ip)

    sort_state = sort_robot.sort_action_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_SORT_ACTION.value["value"], rf_protocol.ENUM_SORT_ACTION_CMD[robot_action[action_command]].value, force_unload_flag)
    
    while sort_state not in wait_state:
        sort_state = sort_robot.sort_action_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_SORT_ACTION.value["value"], rf_protocol.ENUM_SORT_ACTION_CMD[robot_action[action_command]].value, force_unload_flag)
    
    return sort_state


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='A test program.')
    parser.add_argument("robot_rf_addr", help="Rf address of the robot.", type = str)
    parser.add_argument("getway_ip", help="IP address of the gateway.", type = str)
    parser.add_argument("sort_action_command", help="The operation to be performed, load_a, load_b, unload_a, unload_b", type = str)
    args = parser.parse_args()
    
    signal.signal(signal.SIGINT, quit)
    signal.signal(signal.SIGTERM, quit)
    
    error_list = [
        rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITHOUT_CARGO.value["chinese"],
        rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITH_CARGO.value["chinese"],
        rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITHOUT_CARGO.value["chinese"],
        rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITH_CARGO.value["chinese"],
        rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_SENSOR_ERROR.value["chinese"]
    ]

    wait_state = {
        "read_state": [
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOADING.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOADED.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOADING.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOADED.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_ERROR.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_ERROR.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITHOUT_CARGO.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITH_CARGO.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_OVERLENGTH.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITHOUT_CARGO.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITH_CARGO.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_UNKNOW_PROFILE.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_UNKNOW_PROFILE.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_SENSOR_ERROR.value["chinese"], 
        ], 
        "load_a": [
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOADED.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_ERROR.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITHOUT_CARGO.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITH_CARGO.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_OVERLENGTH.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_UNKNOW_PROFILE.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_SENSOR_ERROR.value["chinese"], 
        ], 
        "load_b": [
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOADED.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_ERROR.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITHOUT_CARGO.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITH_CARGO.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_OVERLENGTH.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_UNKNOW_PROFILE.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_SENSOR_ERROR.value["chinese"], 
        ], 
        "unload_a": [
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOADED.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_ERROR.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITHOUT_CARGO.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITH_CARGO.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_UNKNOW_PROFILE.value["chinese"], 
        ], 
        "unload_b": [
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOADED.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_ERROR.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITHOUT_CARGO.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITH_CARGO.value["chinese"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_UNKNOW_PROFILE.value["chinese"], 
        ]
    }

    robot_action = {
        "read_state": 0x00,
        "load_a": 0x01,
        "load_b": 0x02,
        "unload_a": 0x05,
        "unload_b": 0x06
    }

    pick_robot = robot_rf_action(args.robot_rf_addr, args.getway_ip)

    robot_state = pick_robot.sort_load_function(wait_state[args.sort_action_command], robot_action[args.sort_action_command])

    if robot_state in error_list:
        current_time = datetime.datetime.now()
        LOG("load_unload_error", f"{current_time} {robot_state}")
