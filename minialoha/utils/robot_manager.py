"""Module to manage multiple robots at once."""

from typing import Dict, List

from minialoha.utils.robot import Robot


class RobotManager:
    """
    Manager for multiple robots.

    Consolidates robot management related logic irrespective of the type of servos or the communication logic.

    :param robots: A dictionary where the keys are robot names and the values are Robot instances.
    """

    def __init__(
        self,
        robots: Dict[str, Robot] = {},
        leader_robot_names: List[str] = [],
        puppet_robot_names: List[str] = [],
    ):
        self.robots = robots
        self.leader_robot_names = leader_robot_names
        self.puppet_robot_names = puppet_robot_names

    def add_robot(self, name: str, robot: Robot):
        """
        Adds a robot to the manager.

        :param name: The name of the robot.
        :param robot: The Robot instance to add.
        """
        self.robots[name] = robot

    def get_robot(self, name: str):
        return self.robots[name]

    def get_robot_pos(self, name: str):
        return self.robots[name].read_position()

    def set_robot_goal_pos(self, name: str, action):
        self.robots[name].set_goal_position(action)

    def shutdown_gracefully(self):
        """
        Shuts down all robots gracefully.
        """
        print("Shutting down robots gracefully...")
        for _, robot in self.robots.items():
            robot.disable_robot()
