from rf_protocol import *
from robot_rf_action import *
import argparse

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description = 'change claw test arguments')
    parser.add_argument("rf_address", help = "rf address", type = str)
    parser.add_argument("gateway_ip", help = "ip address of gateway", type = str)

    args = parser.parse_args()

    pick_robot = robot_rf_action(args.rf_address, args.gateway_ip)

    wait_state = [
        rf_protocol.ENUM_PICK_SENSOR_CHECK_LOAD_STATES.PICK_SENSOR_LOAD_STATE_OK.value["chinese"],
        rf_protocol.ENUM_PICK_SENSOR_CHECK_LOAD_STATES.PICK_SENSOR_LOAD_STATE_POSITION_ERROR.value["chinese"],
        rf_protocol.ENUM_PICK_SENSOR_CHECK_LOAD_STATES.PICK_SENSOR_LOAD_STATE_ERROR.value["chinese"]
    ]

    pick_robot.pick_unload_sensor_action(wait_state)
