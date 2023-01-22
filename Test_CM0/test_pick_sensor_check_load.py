from turtle import st, up
from rf_protocol import *
from robot_rf_command import *
import pytest

def pick_sensor_check_function(pick_sensor_check_function: int, wait: list, robot_rf: str = '474E50', getway_ip: str = '10.0.64.225'):
    
    pick_robot = robot_rf_command(robot_rf, getway_ip)

    robot_state = pick_robot.pick_sensor_check_loaded_command(pick_sensor_check_function)

    while robot_state["chinese"] not in wait:
        robot_state = pick_robot.pick_sensor_check_loaded_command(pick_sensor_check_function)

    return robot_state


def test_pick_sensor_check_load():
    
    sensor_state = [rf_protocol.ENUM_PICK_SENSOR_CHECK_LOAD_STATES.PICK_SENSOR_LOAD_STATE_OK.value["chinese"], rf_protocol.ENUM_PICK_SENSOR_CHECK_LOAD_STATES.PICK_SENSOR_LOAD_STATE_POSITION_ERROR.value["chinese"]]

    robot_state = pick_sensor_check_function(rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_PICK_SENSOR_CHECK_LOADED.value["value"], sensor_state)
    
    assert robot_state["chinese"] == rf_protocol.ENUM_PICK_SENSOR_CHECK_LOAD_STATES.PICK_SENSOR_LOAD_STATE_OK.value["chinese"]