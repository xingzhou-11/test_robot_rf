#!/usr/bin/python3
from argparse import ArgumentParser, RawTextHelpFormatter
import sys 
sys.path.append("..") 
from robot_rf_command import robot_rf_command

parser = ArgumentParser(
    description='change claw test arguments', 
    usage='use "python %(prog)s --help" for more information',
    formatter_class=RawTextHelpFormatter
)
parser.add_argument("rf_address", type=str, help="rf address")
parser.add_argument("getway_ip", type=str, help="ip address of getway")
args = parser.parse_args()

if __name__ == "__main__":
    robot = robot_rf_command(args.rf_address, args.getway_ip)
    robot.moving_cancel_command()
