from rf_protocol import *
from robot_rf_command import *
import argparse


def robot_moving(robot_rf_addr: str, getway_ip: str, absolute_target: int, velocity_in_counts: int):
    """机器人移动

    Args:
        rf_address (str): 机器人rf地址
        gateway_ip (str): 网关IP地址
        absolute_target (int): 要到达的位置
        velocity_in_counts (int): 目标速度
    """

    robot = robot_rf_command(robot_rf_addr, getway_ip)

    robot_start = robot.moving_command(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_MOVING.value["value"], absolute_target, velocity_in_counts)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='A test program.')
    parser.add_argument("robot_rf_addr", help="Rf address of the robot.", type = str)
    parser.add_argument("getway_ip", help="IP address of the gateway.", type = str)
    parser.add_argument("absolute_target", help="The position to be achieved.", type = str)
    parser.add_argument("velocity_in_counts", help="target velocity.", type = str)
    args = parser.parse_args()
    robot_moving(args.robot_rf_addr, args.getway_ip, int(args.absolute_target), int(args.velocity_in_counts))
