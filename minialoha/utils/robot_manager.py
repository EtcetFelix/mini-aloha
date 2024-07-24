"""Module to manage multiple robots at once."""

from typing import Dict

from minialoha.utils.robot import Robot


class RobotManager:
    """
    Manager for multiple robots.

    :param robots: A dictionary where the keys are robot names and the values are Robot instances.
    """

    def __init__(self, robots: Dict[str, Robot] = {}):
        self.robots = robots

    def add_robot(self, name: str, robot: Robot):
        """
        Adds a robot to the manager.

        :param name: The name of the robot.
        :param robot: The Robot instance to add.
        """
        self.robots[name] = robot

    def get_robot(self, name: str):
        return self.robots[name]

    def shutdown_gracefully(self):
        """
        Shuts down all robots gracefully.
        """
        print("Shutting down robots gracefully...")
        for _, robot in self.robots.items():
            robot.disable_robot()
