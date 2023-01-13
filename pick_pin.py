from rf_protocol import *
from robot_rf_command import *
import argparse
import pytest




def pick_pin_action(robot_rf: str, getway_ip: str, action_command: int, wait_state: list):
    """pick PIN 动作

    Args:
        robot_rf (str): 机器人rf地址
        getway_ip (str): 网关IP地址
        action_command (int): PIN 动作
        wait_state (list): 可以退出的状态列表

    Returns:
        _type_: _description_
    """
    
    robot_action = {
        "pin_in": "PICK_PIN_ACTION_CMD_IN", 
        "pin_out": "PICK_PIN_ACTION_CMD_OUT"
    }
    
    pick_robot = robot_rf_command(robot_rf, getway_ip)

    robot_state = pick_robot.pick_pin_action_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_PIN_ACTION.value["value"], rf_protocol.ENUM_PICK_PIN_ACTION_CMD[robot_action[action_command]].value)

    while robot_state not in wait_state:
        robot_state = pick_robot.pick_pin_action_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_PIN_ACTION.value["value"], rf_protocol.ENUM_PICK_PIN_ACTION_CMD[robot_action[action_command]].value)

    return robot_state


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description = 'pick pin operate')
    parser.add_argument("rf_address", help = "rf address", type = str)
    parser.add_argument("getway_ip", help = "ip address of getway", type = str)
    parser.add_argument("pin_command", help = "pin in 缩进 pin out 伸出", type = str)

    args = parser.parse_args()
    wait_state = {
        "pin_in": [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_IN.value["chinese"], 
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"]
        ], 
        "pin_out": [
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_OUT.value["chinese"], 
            rf_protocol.ENUM_PICK_PIN_STATES.PICK_PIN_FAIL.value["chinese"]
        ]
    }

    pick_pin_action(args.rf_address, args.getway_ip, args.pin_command, wait_state[args.pin_command])

