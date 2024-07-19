import time

from minialoha.utils.dynamixel import Dynamixel
from minialoha.utils.robot import Robot

# fixed constants
DELTA_TIME_STEP = 0.02


def prep_robots(leader: Robot, puppet_bot=None):
    # TODO: move arms to starting position
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
    # TODO: move grippers to starting position
    # TODO: Turn torque on for everything
    pass


def press_to_start(leader: Robot):
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
        baudrate=1_000_000, device_name="COM6"
    ).instantiate()
    # follower_dynamixel = Dynamixel.Config(
    #     baudrate=1_000_000, device_name="COM3"
    # ).instantiate()
    # follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 5])
    leader = Robot(leader_dynamixel, servo_ids=[11, 12, 13, 14, 15])
    # side = sys.argv[1]
    # while True:

    prep_robots(leader)
    # press_to_start(leader)
    time.sleep(DELTA_TIME_STEP / 5)
    print("done")
    # time.sleep(DELTA_TIME_STEP)
    # e = IPython.embed()
    # teleop()
