#!/usr/bin/python3
from argparse import ArgumentParser, RawTextHelpFormatter
import textwrap
import sys 
sys.path.append("..") 
from rf_parameters import parameters
from robot_rf_command import robot_rf_command

parser = ArgumentParser(
    description='change claw test arguments', 
    usage='use "python %(prog)s --help" for more information',
    formatter_class=RawTextHelpFormatter
)

parser.add_argument("rf_address", type=str, help="rf address")
parser.add_argument("getway_ip", help="IP address of the gateway.", type = str)
parser.add_argument("absolute_target", help="The position to be achieved.", type = str)
parser.add_argument("velocity_in_counts", help="target velocity.", type = str)

args = parser.parse_args()

if __name__ == "__main__":
    parameters.moving["absolute_target"] = int(args.absolute_target)
    parameters.moving["velocity_in_counts"] = int(args.velocity_in_counts)

    robot = robot_rf_command(args.robot_rf_addr, args.getway_ip)
    robot.cm0_moving_command()
