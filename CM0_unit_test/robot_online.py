#!/usr/bin/python3
from argparse import ArgumentParser, RawTextHelpFormatter
import textwrap

import sys 
sys.path.append("..") 

from robot_rf_command import robot_rf_command
from rf_parameters import parameters

parser = ArgumentParser(description='A test program.',
    usage='use "python %(prog)s --help" for more information',
    formatter_class=RawTextHelpFormatter)

parser.add_argument("robot_rf_addr", type= str, help= "Rf address of the robot.")
parser.add_argument("getway_ip", type= str, help= "IP address of the getway.")
parser.add_argument("command", type= str, 
    help= textwrap.dedent('''\
    READ 0x00
    ONLINE 0x01'''))
args = parser.parse_args()

if __name__ == "__main__":
    parameters.state["cmd"] = int(args.command, 16)

    robot = robot_rf_command(args.robot_rf_addr, args.getway_ip)
    robot.ping_command()
