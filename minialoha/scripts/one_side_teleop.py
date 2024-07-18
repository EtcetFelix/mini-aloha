import sys
import time

import IPython

from minialoha.utils.dynamixel import Dynamixel
from minialoha.utils.robot import Robot

e = IPython.embed


# fixed constants
DELTA_TIME_STEP = 0.02


def prep_robots(master_bot, puppet_bot):
    # TODO: move arms to starting position
    # TODO: move grippers to starting position
    # TODO: Turn torque on for everything
    pass


def press_to_start():
    # TODO: press gripper to start data collection
    # TODO: disable torque for only gripper joint of master robot to allow user movement
    leader.set_trigger_torque()


def teleop():
    """A standalone function for experimenting with teleoperation. No data recording."""
    while True:
        # Sync joint and gripper positions
        follower.set_goal_pos(leader.read_position())
        # sleep DT
        time.sleep(DELTA_TIME_STEP)


if __name__ == "__main__":
    leader_dynamixel = Dynamixel.Config(
        baudrate=1_000_000, device_name="/dev/tty.usbmodem57380045631"
    ).instantiate()
    follower_dynamixel = Dynamixel.Config(
        baudrate=1_000_000, device_name="/dev/tty.usbserial-FT8ISNO8"
    ).instantiate()
    follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 6, 7])
    leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 6, 7])
    side = sys.argv[1]
    press_to_start()
    teleop()
