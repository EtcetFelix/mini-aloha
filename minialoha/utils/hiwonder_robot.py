from typing import List

from hiwonderbuslinker.bus_control import ServoBus, ServoPosition
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

    def set_goal_position(self, positions: List[int]):
        goal_positions = [
            ServoPosition(servo_id, pos)
            for servo_id, pos in zip(self.servo_bus.servo_ids, positions)
        ]
        self.servo_bus.set_bus_position(
            goal_positions, servo_bus=self.servo_bus_communication, time_s=0.2
        )

    def disable_robot(self):
        pass

    def change_servo_id(self, old_servo_id: int, new_servo_id: int):
        self.servo_bus_communication.id_write(old_servo_id, new_servo_id)
