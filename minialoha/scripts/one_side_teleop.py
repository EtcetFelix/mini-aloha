import os
import sys
import time

from minialoha.utils.dynamixel import Dynamixel
from minialoha.utils.robot import Robot
from minialoha.utils.robot_manager import RobotManager

# fixed constants
DELTA_TIME_STEP = 0.02


def prep_robots(leader: Robot, follower: Robot):
    # TODO: move arms to starting position
    # Make sure to manually adjust your robot to these positions
    # Otherwise they'll move crazy fast and can potentially damage wires
    leader.set_goal_pos(
        action=[
            1000,
            1796,
            1629,
            1003,
            2198,
        ]
    )
    print(f"Leader position: {leader.read_position()}")
    follower.set_goal_pos(action=[2048, 2092, 1967, 1070, 2001])
    print(f"Follower position: {follower.read_position()}")
    # TODO: move grippers to starting position
    # TODO: Turn torque on for everything
    pass


def press_to_start(leader: Robot):
    # TODO: disable torque for only gripper joint of master robot to allow user movement
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
            follower.set_goal_pos(leader.read_position())
        except Exception as e:
            print(e)
            break
        # sleep DT
        time.sleep(DELTA_TIME_STEP)


def main(robo_manager: RobotManager):
    leader_dynamixel = Dynamixel.Config(
        baudrate=1_000_000, device_name="COM6"
    ).instantiate()
    follower_dynamixel = Dynamixel.Config(
        baudrate=1_000_000, device_name="COM3"
    ).instantiate()
    robo_manager.add_robot(
        "follower",
        Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 5]),
    )
    robo_manager.add_robot(
        "leader",
        Robot(leader_dynamixel, servo_ids=[11, 12, 13, 14, 15]),
    )
    leader = robo_manager.get_robot("leader")
    follower = robo_manager.get_robot("follower")
    prep_robots(
        leader,
        follower,
    )
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
