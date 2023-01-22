from rf_protocol import *
from robot_rf_command import *
import argparse

def pick_chain_action(robot_rf_addr: str, getway_ip: str, wait_state: list, action_command: str):
    """pick链条动作

    Args:
        rf_address (str): 机器人rf地址
        gateway_ip (str): 网关IP地址
        wait_state (list): 可以退出的状态列表
        action_command (str): 链条动作, read, start, clear_state, direct
    """
    
    pick_robot = robot_rf_command(robot_rf_addr, getway_ip)

    robot_action = {
        "read": "PICK_CHAIN_HOMING_CMD_READ",
        "start": "PICK_CHAIN_HOMING_CMD_START",
        "clear_state": "PICK_CHAIN_HOMING_CMD_CLEAR_STATE", # 清除原点
        "direct": "PICK_CHAIN_HOMING_CMD_DIRECT"  # 当前位置为原点
    }
    
    robot_state = pick_robot.pick_chain_homing_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_CHAIN_HOMING.value["value"], rf_protocol.ENUM_PICK_CHAIN_HOMING_CMD[robot_action[action_command]].value)

    while robot_state not in wait_state:
        robot_state = pick_robot.pick_chain_homing_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_CHAIN_HOMING.value["value"], rf_protocol.ENUM_PICK_CHAIN_HOMING_CMD[robot_action[action_command]].value)
    
    return robot_state

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description = 'change claw test arguments')
    parser.add_argument("rf_address", help = "rf address", type = str)
    parser.add_argument("getway_ip", help = "ip address of getway", type = str)
    parser.add_argument("action_command", help = "action command", type = str)
    args = parser.parse_args()
    
    wait_state = {
        "read": [
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_IN_PROGRESS.value["chinese"], 
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_COMPLETED.value["chinese"], 
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_ERROR.value["chinese"], 
        ], 
        "start": [
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_COMPLETED.value["chinese"], 
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_ERROR.value["chinese"]
        ], 
        "clear_state": [
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_UNKNOWN.value["chinese"], 
        ],
        "direct": [
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_COMPLETED.value["chinese"], 
        ]
    }

    pick_chain_action(args.rf_address, args.getway_ip, wait_state[args.action_command], args.action_command)