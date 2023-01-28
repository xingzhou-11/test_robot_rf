#!/usr/bin/python3
from argparse import ArgumentParser, RawTextHelpFormatter
import textwrap
import sys 
sys.path.append("..") 
from rf_protocol import rf_protocol
from rf_parameters import parameters
from robot_rf_command import robot_rf_command
import logging

logging.basicConfig(filename="sort_action_error.log", format='%(asctime)s %(message)s', filemode='w')
logger = logging.getLogger()
logger.setLevel(logging.INFO)

parser = ArgumentParser(
    description='change claw test arguments', 
    usage='use "python %(prog)s --help" for more information',
    formatter_class=RawTextHelpFormatter
)
parser.add_argument("robot_rf_addr", type = str, help="Rf address of the robot.")
parser.add_argument("getway_ip", type = str, help="IP address of the gateway.")
parser.add_argument(
    "command", 
    type=str, 
    help=textwrap.dedent(
        '''
            READ_STATE 0x00
            LOAD_FROM_A 0x01
            LOAD_FROM_B 0x02
            UNLOAD_TO_A 0x05
            UNLOAD_TO_B 0x06'''
    )
)
parser.add_argument(
    "force_unload", 
    type=str, 
    help=textwrap.dedent(
        '''
            CLOSE 0x00
            OPEN 0x01'''   
    )
)

args = parser.parse_args()

def wait_state(command: str):
    if command == "0x00":
        return [
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOADING.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOADED.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOADING.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOADED.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_ERROR.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_ERROR.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITHOUT_CARGO.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITH_CARGO.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_OVERLENGTH.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITHOUT_CARGO.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITH_CARGO.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_UNKNOW_PROFILE.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_UNKNOW_PROFILE.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_SENSOR_ERROR.value["value"]
        ]
    elif command in ["0x01", "0x02"]:
        return [
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOADED.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_ERROR.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITHOUT_CARGO.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITH_CARGO.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_OVERLENGTH.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_UNKNOW_PROFILE.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_SENSOR_ERROR.value["value"]
        ]
    elif command in ["0x05, 0x06"]:
        return [
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOADED.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_ERROR.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITHOUT_CARGO.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITH_CARGO.value["value"], 
            rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_UNKNOW_PROFILE.value["value"], 
        ]


if __name__ == "__main__":
    
    error_list = [
        rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITHOUT_CARGO.value["value"],
        rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_TIMEOUT_WITH_CARGO.value["value"],
        rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITHOUT_CARGO.value["value"],
        rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_UNLOAD_TIMEOUT_WITH_CARGO.value["value"],
        rf_protocol.ENUM_SORT_ACTION_STATE.SORT_ACTION_STATE_LOAD_SENSOR_ERROR.value["value"]
    ]

    parameters.sort_action["cmd"] = int(args.command, 16)
    parameters.sort_action["force_unload"] = int(args.force_unload, 16)
    parameters.sort_action["wait"] = wait_state(args.command)

    robot = robot_rf_command(args.robot_rf_addr, args.getway_ip)
    robot.sort_action_command()

    if parameters.robot_end_state.value["value"] in error_list:
        logger.info(parameters.robot_end_state)
