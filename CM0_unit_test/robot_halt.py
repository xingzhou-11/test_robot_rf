from ..robot_rf_action import *
import argparse

parser = argparse.ArgumentParser(description='A test program.')
parser.add_argument("robot_rf_addr", help="Rf address of the robot.", type = str)
parser.add_argument("getway_ip", help="IP address of the gateway.", type = str)
parser.add_argument("command", help="halt or resume.", type = str)
args = parser.parse_args()

def main(robot_addr: str, rf_getway_ip: str, command:str):
    halt_action_wait_state = [
        rf_protocol.ENUM_HALT_STATE.HALT_STATE_HALT.value["chinese"]
    ]

    halt_resume_wait_state = [
        rf_protocol.ENUM_HALT_STATE.HALT_STATE_RUNNING.value["chinese"]
    ]

    robot = robot_rf_action(robot_addr, rf_getway_ip)

    if command == "halt":
        robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_HALT.value, halt_action_wait_state)
    else:
        robot.robot_halt_action(rf_protocol.ENUM_HALT_ACTION_CMD.HALT_CMD_RESUME.value, halt_resume_wait_state)

if __name__ == "__main__":
    main(args.robot_rf_addr, args.getway_ip, args.command)