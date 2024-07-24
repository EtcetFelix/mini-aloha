import logging

import pytest

from minialoha.utils.dynamixel import Dynamixel
from minialoha.utils.robot import Robot

logger = logging.getLogger(__name__)


@pytest.fixture
def leader():
    leader_dynamixel = Dynamixel.Config(
        baudrate=1_000_000, device_name="COM6"
    ).instantiate()
    leader = Robot(leader_dynamixel, servo_ids=[11, 12, 13, 14, 15])
    return leader


def test_read_position(leader: Robot):
    pos = leader.read_position()
    logger.info(f"Position: {pos}")
    assert pos is not None, "Failed to read position"


def test_read_velocity(leader: Robot):
    velocity = leader.read_velocity()
    logger.info(f"Velocity: {velocity}")
    assert velocity is not None, "Failed to read velocity"


def test_set_goal_pos(leader: Robot):
    """Move the servos to target positions and check if they reached the goal."""
    target_positions = [  # Target positions for the servos, in ticks
        1000,
        2017,
        1629,
        1003,
        2198,
    ]
    pos_error_threshold = 200
    leader.set_goal_pos(target_positions)

    result_positions = leader.read_position()
    logger.info(f"Position: {result_positions}")
    # Verify the servos reached the goal positions
    for indx, target_pos in enumerate(target_positions):
        result_pos = result_positions[indx]
        assert (
            # check within tick tolerance
            target_pos - pos_error_threshold
            <= result_pos
            <= target_pos + pos_error_threshold
        ), f"Position mismatch at index {indx}. Expected {target_pos}, got {result_positions[indx]}"
