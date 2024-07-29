import logging
import time
import timeit

import pytest

from minialoha.utils.hiwonder_robot import HiwonderRobot

logger = logging.getLogger(__name__)

COM_PORT = "COM5"
SERVO_IDS = [1, 2, 3, 4, 5]


@pytest.fixture
def robot() -> HiwonderRobot:
    leader = HiwonderRobot(COM_PORT, SERVO_IDS[0], SERVO_IDS)
    return leader


def test_timing_read_position(robot: HiwonderRobot):
    repetitions = 10
    timer = timeit.Timer(stmt=lambda: robot.read_position(), globals=globals())
    total_time = timer.timeit(number=repetitions)
    logger.info(f"Position: {total_time}")


def test_read_position(robot: HiwonderRobot):
    pos = robot.read_position()
    logger.info(f"Position: {pos}")
    assert pos is not None, "Failed to read position"


def test_set_goal_pos(robot: HiwonderRobot):
    """Move the servos to target positions and check if they reached the goal."""
    target_positions = [  # Target positions for the servos, in ticks
        400,
        350,
        200,
        400,
        750,
    ]
    pos_error_threshold = 100
    start = time.time()
    robot.set_goal_position(target_positions)
    end = time.time()
    logger.info(f"Time taken to set goal positions: {end - start}")

    result_positions = robot.read_position()
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


def test_set_id(robot: HiwonderRobot):
    """Changes the id of a servo in the robot."""
    old_servo_id = 1
    new_servo_id = 2
    robot.change_servo_id(old_servo_id, new_servo_id)
