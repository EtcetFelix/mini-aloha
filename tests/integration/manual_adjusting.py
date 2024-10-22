from minialoha.utils.dynamixel import Dynamixel
from minialoha.utils.dynamixel_robot import DynamixelRobot


def read_position_loop(arm: DynamixelRobot):
    """Initiate a loop that constantly returns the position of the robot."""
    while True:
        pos = arm.read_position()
        print(f"Position: {pos}")


if __name__ == "__main__":
    leader_dynamixel = Dynamixel.Config(
        baudrate=1_000_000, device_name="COM6"
    ).instantiate()
    leader = DynamixelRobot(leader_dynamixel, servo_ids=[11, 12, 13, 14, 15])

    read_position_loop(leader)
