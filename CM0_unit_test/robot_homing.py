#!/usr/bin/python3
from argparse import ArgumentParser, RawTextHelpFormatter
import textwrap
import sys 
sys.path.append("..") 
from rf_protocol import rf_protocol
from rf_parameters import parameters
from robot_rf_command import robot_rf_command

parser = ArgumentParser(
    description='change claw test arguments', 
    usage='use "python %(prog)s --help" for more information',
    formatter_class=RawTextHelpFormatter
)
parser.add_argument("rf_address", type=str, help="rf address")
parser.add_argument("getway_ip", type=str, help="ip address of getway")
parser.add_argument("command", type=str, 
    help=textwrap.dedent('''\
        START_AND_STATE 0x00
        RESET_STATE 0x01
        DELIVER_POSITIVE 0x02
        DELIVER_NEGATIVE 0x03
        START_REVERSE_AND_STATE 0x04'''))

args = parser.parse_args()

def wait_state(command: str):
    wait = {
        "0x00": [
            rf_protocol.ENUM_HOMING_STATE.HOMING_STATE_IDLE.value["value"], 
            rf_protocol.ENUM_HOMING_STATE.HOMING_STATE_COMPLETED.value["value"], 
            rf_protocol.ENUM_HOMING_STATE.HOMING_STATE_COMPLETED_LIFTER_C.value["value"], 
            rf_protocol.ENUM_HOMING_STATE.HOMING_STATE_COMPLETED_LIFTER_D.value["value"]
        ], 
        "0x01": [
            rf_protocol.ENUM_HOMING_STATE.HOMING_STATE_IDLE.value["value"], 
            rf_protocol.ENUM_HOMING_STATE.HOMING_STATE_COMPLETED.value["value"]
        ], 
        "0x02": [
            rf_protocol.ENUM_HOMING_STATE.HOMING_STATE_COMPLETED_LIFTER_C.value["value"], 
            rf_protocol.ENUM_HOMING_STATE.HOMING_STATE_COMPLETED_LIFTER_D.value["value"]
        ], 
        "0x03": [
            rf_protocol.ENUM_HOMING_STATE.HOMING_STATE_COMPLETED_LIFTER_C.value["value"], 
            rf_protocol.ENUM_HOMING_STATE.HOMING_STATE_COMPLETED_LIFTER_D.value["value"]
        ], 
        "0x04": [
            rf_protocol.ENUM_HOMING_STATE.HOMING_STATE_IDLE.value["value"], 
            rf_protocol.ENUM_HOMING_STATE.HOMING_STATE_COMPLETED.value["value"]
        ]
    }
    
    return wait[command]

if __name__ == "__main__":
    parameters.homing["cmd"] = int(args.command, 16)
    parameters.homing["wait"] = wait_state(args.command)

    robot = robot_rf_command(args.rf_address, args.getway_ip)
    robot.homing_command()
