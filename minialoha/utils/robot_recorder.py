# # from interbotix_xs_msgs.msg import JointSingleCommand
# from minialoha.utils.constants import DELTA_TIME_STEP

# e = IPython.embed


import time
from typing import List

import numpy as np


class Recorder:
    def __init__(self, side, init_node=True, is_debug=False):
        from collections import deque

        # import rospy
        # from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
        # from sensor_msgs.msg import JointState

        self.secs = None
        self.nsecs = None
        self._qpos = None
        self.effort = None
        self.arm_command = None
        self.gripper_command = None
        self.is_debug = is_debug

        if self.is_debug:
            self.joint_timestamps = deque(maxlen=50)
            self.arm_command_timestamps = deque(maxlen=50)
            self.gripper_command_timestamps = deque(maxlen=50)
        time.sleep(0.1)

    @property
    def qpos(self) -> List[int]:
        """Returns the current joint positions."""
        if self._qpos is None:
            raise ValueError("qpos is None")
        return self._qpos

    @qpos.setter
    def qpos(self, qpos):
        self._qpos = qpos

    def update_puppet_state(self):
        # TODO: Go get data from the Robot
        self.qpos = data.position
        self.qvel = data.velocity
        self.effort = data.effort
        self.data = data
        if self.is_debug:
            self.joint_timestamps.append(time.time())

    def puppet_arm_commands_cb(self, data):
        self.arm_command = data.cmd
        if self.is_debug:
            self.arm_command_timestamps.append(time.time())

    def puppet_gripper_commands_cb(self, data):
        self.gripper_command = data.cmd
        if self.is_debug:
            self.gripper_command_timestamps.append(time.time())

    def print_diagnostics(self):
        def dt_helper(l):
            l = np.array(l)
            diff = l[1:] - l[:-1]
            return np.mean(diff)

        joint_freq = 1 / dt_helper(self.joint_timestamps)
        arm_command_freq = 1 / dt_helper(self.arm_command_timestamps)
        gripper_command_freq = 1 / dt_helper(self.gripper_command_timestamps)

        print(
            f"{joint_freq=:.2f}\n{arm_command_freq=:.2f}\n{gripper_command_freq=:.2f}\n"
        )
