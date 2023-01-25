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
    READ 0x00
    START 0x01
    CLEAR_STATE 0x02
    DIRECT 0x03'''))

args = parser.parse_args()

def wite_state(command: str):
    wait = {
        "0x00": [
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_IN_PROGRESS.value["chinese"], 
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_COMPLETED.value["chinese"], 
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_ERROR.value["chinese"]
        ], 
        "0x01": [
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_COMPLETED.value["chinese"], 
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_ERROR.value["chinese"]
        ], 
        "0x02": [
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_UNKNOWN.value["chinese"]
        ], 
        "0x03": [
            rf_protocol.ENUM_PICK_CHAIN_HOMING_STATES.PICK_CHAIN_HOMING_COMPLETED.value["chinese"]
        ]
    }
    
    return wait[command]

if __name__ == "__main__":
    parameters.pick_chain_homing["cmd"] = int(args.command, 16)
    parameters.pick_chain_homing["wait"] = wite_state(args.command)

    robot = robot_rf_command(args.rf_address, args.getway_ip)
    robot.pick_chain_homing_command()
