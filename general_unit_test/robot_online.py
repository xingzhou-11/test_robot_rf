from rf_protocol import *
from robot_rf_command import *
import argparse


def robot_online_action(robot_rf_addr, getway_ip):
    """机器人上线

    Args:
        robot_rf_addr(str): 机器人RF地址
        getway_ip(str): 网关IP地址
    """
    robot = robot_rf_command(robot_rf_addr, getway_ip)
    
    robot_start = robot.ping_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PING.value["value"], rf_protocol.ENUM_STATE_CMD.SHUTTLE_STATE_CMD_ONLINE.value)
    
    return robot_start

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='A test program.')
    parser.add_argument("robot_rf_addr", help="Rf address of the robot.", type = str)
    parser.add_argument("getway_ip", help="IP address of the getway.", type = str)
    args = parser.parse_args()
    
    robot_online_action(args.robot_rf_addr, args.getway_ip)