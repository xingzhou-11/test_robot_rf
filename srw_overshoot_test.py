from rf_protocol import *
from robot_rf_command import *
import argparse
import datetime
import time
import logging

parameter = {
    "function_code": rf_protocol.ENUM_FUNCTION_CODE.FUNCTION_CODE_MOVING.value["value"],
    "absolute_target": 4.4,
    "velocity_in_counts": 4,
    "acceleration_in_counts": 2.5,
    "deceleration_in_counts": 2.5
}

def DLOG(log_file_name:str, log_msg: str):
    """DLOG

    Args:
        log_file_name (str): log file name
        level (int): level
        log_msg (str): log msg
    """
    logging.basicConfig(filename=log_file_name, level=logging.INFO)
    logging.info(log_msg)

def main(robot_rf_addr: str,getway_ip: str, function_code: int, self_correct: int=0, Reset_position_check: int=0, enter_lifter: int=0):
    """过冲测试

    Args:
        robot_rf_addr (str): 机器人rf地址
        getway_ip (str): 网关rf地址
        function_code (int): 功能码
        self_correct (int): 自我修正, 打开输入1
        Reset_position_check (int): 复位位置差, 打开输入1
        enter_lifter (int): 进入升降机, 打开输入1
    """
    absolute_target = int(parameter["absolute_target"] * 16000)
    velocity_in_counts = int(parameter["velocity_in_counts"] * 16000)
    acceleration_in_counts = int(parameter["acceleration_in_counts"] * 132629)
    deceleration_in_counts = int(parameter["deceleration_in_counts"] * 132629)
    additional_info = (self_correct<<3) + (Reset_position_check<<2) + enter_lifter
    
    robot = robot_rf_command(robot_rf_addr, getway_ip)
    
    for i in range(1):
        least_value = max_value = 0
        
        # DLOG("overshoot_data.log", f"start_time: {datetime.datetime.now()}")
        while True:
            robot_data = robot.srw_moving_command(function_code, absolute_target, velocity_in_counts, additional_info, acceleration_in_counts, deceleration_in_counts)
            # if robot_data[0] > absolute_target:
            #     max_value = max(max_value, robot_data[0])
            # print(robot_data[0], robot_data[1], robot_data[2])
            DLOG("overshoot_data.log", f",{datetime.datetime.now()},speed,{robot_data[1]},")
            if abs(robot_data[0] - absolute_target) < 160 and robot_data[1] == 0:
                # DLOG("overshoot_data.log", f"{datetime.datetime.now()} max_value: {max_value - absolute_target}")
                # DLOG("overshoot_data.log", f"end_time: {datetime.datetime.now()}")
                break

        # time.sleep(1)
        
        # DLOG("overshoot_data.log", f"start_time: {datetime.datetime.now()}")
        while True:
            robot_data = robot.srw_moving_command(function_code, 0, velocity_in_counts, additional_info, acceleration_in_counts, deceleration_in_counts)
            # if robot_data[0] < 0:
            #     least_value = min(least_value, robot_data[0])
            # print(robot_data[1])
            DLOG("overshoot_data.log", f",{datetime.datetime.now()},speed,{robot_data[1]},")
            if abs(robot_data[0] - 0) < 160 and robot_data[1] == 0:
                # DLOG("overshoot_data.log", f"{datetime.datetime.now()} least_value: {least_value}")
                # DLOG("overshoot_data.log", f"end_time: {datetime.datetime.now()}")
                break

        time.sleep(1)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='A test program.')
    parser.add_argument("robot_rf_addr", help="Rf address of the robot.", type = str)
    parser.add_argument("getway_ip", help="IP address of the gateway.", type = str)
    args = parser.parse_args()

    main(args.robot_rf_addr, args.getway_ip, parameter["function_code"], self_correct=0)

