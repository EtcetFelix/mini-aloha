import logging

import pytest

from minialoha.utils.dynamixel import Dynamixel
from minialoha.utils.dynamixel_robot import DynamixelRobot
from minialoha.utils.hiwonder_robot import HiwonderRobot
from minialoha.utils.robot_to_robot import dynamixel_to_hiwonder_position

logger = logging.getLogger(__name__)

DNX_PORT = "COM6"
DNX_SERVO_IDS = [11, 12, 13, 14, 15]

HW_PORT = "COM5"
HW_SERVO_IDS = [1, 2, 3, 4, 5]


@pytest.fixture
def dynamixel_robot():
    leader_dynamixel = Dynamixel.Config(
        baudrate=1_000_000, device_name=DNX_PORT
    ).instantiate()
    leader = DynamixelRobot(leader_dynamixel, servo_ids=DNX_SERVO_IDS)
    return leader


@pytest.fixture
def hiwonder_robot() -> HiwonderRobot:
    leader = HiwonderRobot(HW_PORT, HW_SERVO_IDS[0], HW_SERVO_IDS)
    return leader


def test_cross_servo_teleop(
    dynamixel_robot: DynamixelRobot, hiwonder_robot: HiwonderRobot
):
    # Create a function that translates the position read from leader to follower
    dnx_pos = dynamixel_robot.read_position()
    hw_pos = dynamixel_to_hiwonder_position(dnx_pos)
    # hiwonder_robot.set_goal_position(hw_pos)
