import pytest

from minialoha.utils.dynamixel import Dynamixel
from minialoha.utils.robot import Robot


@pytest.fixture
def leader_dynamixel():
    Dynamixel.Config(baudrate=1_000_000, device_name="COM6").instantiate()
    leader = Robot(leader_dynamixel, servo_ids=[11, 12, 13, 14, 15])
    return leader


def test_read_position(leader: Robot):
    pos = leader.read_position()
    assert pos is not None, "Failed to read position"


def test_read_velocity(leader: Robot):
    velocity = leader.read_velocity()
    assert velocity is not None, "Failed to read velocity"


def test_set_goal_pos(leader: Robot):
    leader.set_goal_pos([1000, 1500, 2000, 3000, 3000])
    assert leader.read_position() == [
        1000,
        1500,
        2000,
        3000,
        3000,
    ], "Failed to set goal position"
