#!/usr/bin/python3
from argparse import ArgumentParser, RawTextHelpFormatter
import textwrap
import sys 
sys.path.append("..") 
from rf_protocol import rf_protocol
from rf_parameters import parameters
from robot_rf_command import robot_rf_command
import logging

parser = ArgumentParser(
    description='change claw test arguments', 
    usage='use "python %(prog)s --help" for more information',
    formatter_class=RawTextHelpFormatter
)
parser.add_argument("rf_address", type=str, help="rf address")
parser.add_argument("gateway_ip", type=str, help="ip address of gateway")
parser.add_argument(
    "function_code", 
    type = str, 
    help=textwrap.dedent(
        '''
            standard 0x06
            lift_postion 0xDC
            '''
    )
)
parser.add_argument(
    "command", 
    type=str, 
    help=textwrap.dedent(
        '''
            READ_BOX_STATE 0x80
            LOAD_FROM_A 0x81
            LOAD_FROM_B 0x82
            UNLOAD_TO_A 0x83
            UNLOAD_TO_B 0x84
            DOUBLE_STORAGE_LOAD_FROM_A 0x85
            DOUBLE_STORAGE_LOAD_FROM_B 0x86
            DOUBLE_STORAGE_UNLOAD_TO_A 0x87
            DOUBLE_STORAGE_UNLOAD_TO_B 0x88
            LOADED_TO_CENTER 0x89
            CENTER_TO_LOADED 0x8A
            RESET_RECOVERABLE_ERROR 0x8B
            RESET_LOAD_UNLOAD_ERROR 0x8C
            CLAW_SIDE_TO_A 0x8D
            CLAW_SIDE_TO_B 0x8E
            CLAW_SIDE_ACK 0x8F
            POSITION_RECOVER 0x90
            '''
    )
)

args = parser.parse_args()

def wait_state(commid: str):
    # READ_BOX_STATE
    if commid in ["0x80"]:
        return [
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
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABEL_CHAIN_TORQUE_EXCESS.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHAIN_POSITION_RECOVER_ERROR.value["chinese"]
        ]
    # LOAD_FROM_A, LOAD_FROM_B, DOUBLE_STORAGE_LOAD_FROM_A, DOUBLE_STORAGE_LOAD_FROM_B
    elif commid in ["0x81", "0x82", "0x85", "0x86"]:
        return [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_FAIL.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOAD_OVERWEIGHT.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"]
        ]
    # UNLOAD_TO_A, UNLOAD_TO_B, DOUBLE_STORAGE_UNLOAD_TO_A, DOUBLE_STORAGE_UNLOAD_TO_B
    elif commid in ["0x83", "0x84", "0x87", "0x88"]:
        return [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNKNOWN.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOAD_FAIL.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_GRAB_FAIL.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE_TRAY_CONNECT_FAIL.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABEL_CHAIN_TORQUE_EXCESS.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_UNLOADED_AT_ENTRANCE.value["chinese"]
        ]
    # LOADED_TO_CENTER
    elif commid in ["0x89"]:
        return [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CENTERED.value["chinese"]
        ]
    # CENTER_TO_LOADED
    elif commid in ["0x8A"]:
        return [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]
        ]
    # RESET_RECOVERABLE_ERROR
    elif commid in ["0x8B"]:
        return [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]
        ]
    # RESET_LOAD_UNLOAD_ERROR
    elif commid in ["0x8C"]:
        return [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"]
        ]
    # CLAW_SIDE_TO_A, CLAW_SIDE_TO_B
    elif commid in ["0x8D", "0x8D"]:
        return [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_ERROR_RECOVERABLE.value["chinese"], 
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_COMPLETED.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHANGE_CLAW_SIDE_FAIL.value["chinese"]
        ]
    # CLAW_SIDE_ACK
    elif commid in ["0x8F"]:
        return [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_LOADED.value["chinese"]
        ]
    # POSITION_RECOVER
    elif commid in ["0x90"]:
        return [
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_NO_BOX.value["chinese"],
            rf_protocol.ENUM_PICK_BOX_STATES.PICK_BOX_CHAIN_POSITION_RECOVER_ERROR.value["chinese"]
        ]

if __name__ == "__main__":
    parameters.pick_box_action["func"] = int(args.function_code, 16)
    parameters.pick_box_action["cmd"] = int(args.command, 16)
    parameters.pick_box_action["decoupled_absolute_position"] = 9000
    parameters.pick_box_action["aligned_absolute_position"] = 0
    parameters.pick_box_action["leave_box_slot_absolute_position"] = 0
    parameters.pick_box_action["column's_direction"] = 1
    parameters.pick_box_action["vertical_correction"] = 1 # 垂直校正
    parameters.pick_box_action["load_weigh"] = 0
    parameters.pick_box_action["definition_full_load"] = 0
    parameters.pick_box_action["check_space"] = 0
    parameters.pick_box_action["wait"] = wait_state(args.command)

    robot = robot_rf_command(args.rf_address, args.gateway_ip)
    robot.pick_box_action_command()
