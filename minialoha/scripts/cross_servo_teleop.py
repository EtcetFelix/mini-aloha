import os
import sys
import time

from minialoha.utils.constants import (
    DELTA_TIME_STEP,
    LEFT_LEADER_BOT_NAME,
    LEFT_PUPPET_BOT_NAME,
    prep_robots,
)
from minialoha.utils.dynamixel import Dynamixel
from minialoha.utils.dynamixel_robot import DynamixelRobot
from minialoha.utils.hiwonder_robot import HiwonderRobot
from minialoha.utils.robot import Robot
from minialoha.utils.robot_manager import RobotManager


def press_to_start(leader: DynamixelRobot):
    trigger_goal_pos = 2500
    leader.set_trigger_waiting_torque(trigger_goal_pos)
    time.sleep(DELTA_TIME_STEP * 10)
    leader.turn_off_trigger_torque()
    close_thresh = 0.1
    close_thresh_in_ticks = (1 - close_thresh) * trigger_goal_pos
    pressed = False
    while not pressed:
        gripper_pos = leader.read_position()[-1]
        if gripper_pos < close_thresh_in_ticks:
            pressed = True
            print("Gripper closed!")
        time.sleep(DELTA_TIME_STEP / 10)
    leader.set_trigger_torque()


def teleop(leader: Robot, follower: Robot):
    """A standalone function for experimenting with teleoperation. No data recording."""
    while True:
        try:
            # Sync joint and gripper positions
            # follower.set_goal_position(leader.read_position())
            leader_pos = leader.read_position()
            follower.set_goal_position(leader_pos)
        except Exception as e:
            print(e)
            break
        # sleep DT
        time.sleep(DELTA_TIME_STEP)


def instantiate_robots(robo_manager: RobotManager):
    leader_dynamixel = Dynamixel.Config(
        baudrate=1_000_000, device_name="COM6"
    ).instantiate()
    robo_manager.add_robot(
        LEFT_LEADER_BOT_NAME,
        DynamixelRobot(leader_dynamixel, servo_ids=[11, 12, 13, 14, 15]),
    )

    follower = HiwonderRobot(port="COM1", leader_id=1, pre_existing_servo_ids=[2, 3, 4])
    robo_manager.add_robot(
        LEFT_PUPPET_BOT_NAME,
        follower,
    )
    return robo_manager


def main(robo_manager: RobotManager):
    instantiate_robots(robo_manager)
    leader = robo_manager.get_robot(LEFT_LEADER_BOT_NAME)
    follower = robo_manager.get_robot(LEFT_PUPPET_BOT_NAME)
    prep_robots(
        leader,
        follower,
    )
    if isinstance(leader, DynamixelRobot):
        press_to_start(leader)
    time.sleep(DELTA_TIME_STEP)
    teleop(leader, follower)


if __name__ == "__main__":
    robo_manager = RobotManager()
    try:
        main(robo_manager)
    except KeyboardInterrupt:
        print("Shutdown requested...exiting")
        robo_manager.shutdown_gracefully()
        try:
            sys.exit(130)
        except SystemExit:
            os._exit(130)
