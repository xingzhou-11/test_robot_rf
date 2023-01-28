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
parser.add_argument("robot_rf_addr", type = str, help="Rf address of the robot.")
parser.add_argument("getway_ip", type = str, help="IP address of the gateway.")

args = parser.parse_args()

if __name__ == "__main__":
    robot = robot_rf_command(args.robot_rf_addr, args.getway_ip)
    robot.sort_sensor_state_command()
    
