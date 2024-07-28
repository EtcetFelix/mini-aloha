from abc import ABC, abstractmethod
from typing import List


class Robot(ABC):
    """Interface for controlling a robot."""

    @abstractmethod
    def read_position(self):
        pass

    @abstractmethod
    def set_goal_position(self, position: List[int]):
        pass
