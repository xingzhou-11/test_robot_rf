from rf_protocol import *
from robot_rf_command import *
import argparse

parser = argparse.ArgumentParser(description = 'change claw test arguments')
parser.add_argument("rf_address", help = "rf address", type = str)
parser.add_argument("getway_ip", help = "ip address of getway", type = str)
parser.add_argument("action_command", help = "action command", type = str)
parser.add_argument("target_location", help = "target location", type = int)
args = parser.parse_args()

def pick_chain_direct_control(robot_rf_addr: str, getway_ip: str, function_code: int, direct_control_command: int, absolute_target_position: int):
    """_summary_

    Args:
        robot_rf_addr (str): 机器人rf地址
        getway_ip (str): 网关IP地址
        function_code (int): 功能码
        direct_control_command (int): 命令
        absolute_target_position (int): 距离
    """
    command = {
        "read": rf_protocol.ENUM_PICK_CHAIN_DIRECT_CTRL_CMD.CHAIN_DIRECT_CONTROL_READ.value, 
        "set_target": rf_protocol.ENUM_PICK_CHAIN_DIRECT_CTRL_CMD.CHAIN_DIRECT_CONTROL_SET_TARGET.value
    }

    pick_robot = robot_rf_command(robot_rf_addr, getway_ip)

    robot_state = pick_robot.pick_chain_direct_control_command(function_code, command[direct_control_command], absolute_target_position)
    print(robot_state)
    

if __name__ == "__main__":
    pick_chain_direct_control(args.rf_address, args.getway_ip, 0x0C, args.action_command, args.target_location)
