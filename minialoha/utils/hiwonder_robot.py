from typing import List

from hiwonderbuslinker.bus_control import ServoBus
from hiwonderbuslinker.lewansoul_servo_bus import ServoBusCommunication

from minialoha.utils.robot import Robot


class HiwonderRobot(Robot):
    """Class for controlling a hiwonder robot."""

    def __init__(
        self, port="COM1", leader_id=1, pre_existing_servo_ids=[2, 3, 4]
    ) -> None:
        super().__init__()
        self.servo_bus = ServoBus(port, leader_id, pre_existing_servo_ids)
        self.servo_bus_communication = ServoBusCommunication(port)

    def read_position(self) -> List[int]:
        positions = self.servo_bus.get_bus_position(self.servo_bus_communication)
        return [pos for pos in positions.values()]

    def set_goal_position(self, position: List[int]):
        pass

    def disable_robot(self):
        pass
