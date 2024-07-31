import collections
import time

import dm_env

# import IPython
# import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray

from minialoha.utils.constants import (
    DELTA_TIME_STEP,
    GRIPPER_INDEX,
    NUM_JOINTS_ON_ROBOT,
    PUPPET_GRIPPER_POSITION_NORMALIZE_FN,
)
from minialoha.utils.robot_manager import RobotManager

# from interbotix_xs_modules.arm import InterbotixManipulatorXS
# from interbotix_xs_msgs.msg import JointSingleCommand
from minialoha.utils.robot_recorder import (
    Recorder,
)
from minialoha.utils.robot_utils import (
    # ImageRecorder,
    # move_arms,
    # move_grippers,
    # setup_master_bot,
    setup_puppet_bot,
)

# e = IPython.embed


class RealEnv:
    """
    Environment for real robot bi-manual manipulation
    Action space:      [left_arm_qpos (6),             # absolute joint position
                        left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)
                        right_arm_qpos (6),            # absolute joint position
                        right_gripper_positions (1),]  # normalized gripper position (0: close, 1: open)

    Observation space: {"qpos": Concat[ left_arm_qpos (6),          # absolute joint position
                                        left_gripper_position (1),  # normalized gripper position (0: close, 1: open)
                                        right_arm_qpos (6),         # absolute joint position
                                        right_gripper_qpos (1)]     # normalized gripper position (0: close, 1: open)
                        "qvel": Concat[ left_arm_qvel (6),         # absolute joint velocity (rad)
                                        left_gripper_velocity (1),  # normalized gripper velocity (pos: opening, neg: closing)
                                        right_arm_qvel (6),         # absolute joint velocity (rad)
                                        right_gripper_qvel (1)]     # normalized gripper velocity (pos: opening, neg: closing)
                        "images": {"cam_high": (480x640x3),        # h, w, c, dtype='uint8'
                                   "cam_low": (480x640x3),         # h, w, c, dtype='uint8'
                                   "cam_left_wrist": (480x640x3),  # h, w, c, dtype='uint8'
                                   "cam_right_wrist": (480x640x3)} # h, w, c, dtype='uint8'
    """

    def __init__(
        self,
        robot_manager: RobotManager,
        setup_robots=True,
    ):
        if setup_robots:
            self.setup_robots()

        self.robot_manager = robot_manager
        self.recorders = {}
        for robot_name in robot_manager.puppet_robot_names:
            self.recorders[robot_name] = Recorder(robot_name, self.robot_manager)
        # self.image_recorder = ImageRecorder(init_node=False)
        # self.gripper_command = JointSingleCommand(name="gripper")

    def setup_robots(self):
        setup_puppet_bot(self.puppet_bot_left)
        setup_puppet_bot(self.puppet_bot_right)

    def get_qpos(self) -> NDArray[np.float64]:
        positions = []
        for recorder in self.recorders.values():
            recorder.update_puppet_state()
            qpos_raw = recorder.qpos
            qpos = qpos_raw[:NUM_JOINTS_ON_ROBOT]
            gripper_qpos = [
                PUPPET_GRIPPER_POSITION_NORMALIZE_FN(qpos_raw[GRIPPER_INDEX])
            ]  # this is position not joint
            positions.append(qpos)
            # positions.append(gripper_qpos)
        return np.concatenate(positions)

    # def get_qvel(self):
    #     left_qvel_raw = self.recorder_left.qvel
    #     right_qvel_raw = self.recorder_right.qvel
    #     left_arm_qvel = left_qvel_raw[:6]
    #     right_arm_qvel = right_qvel_raw[:6]
    #     left_gripper_qvel = [PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN(left_qvel_raw[7])]
    #     right_gripper_qvel = [PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN(right_qvel_raw[7])]
    #     return np.concatenate(
    #         [left_arm_qvel, left_gripper_qvel, right_arm_qvel, right_gripper_qvel]
    #     )

    # def get_effort(self):
    #     left_effort_raw = self.recorder_left.effort
    #     right_effort_raw = self.recorder_right.effort
    #     left_robot_effort = left_effort_raw[:7]
    #     right_robot_effort = right_effort_raw[:7]
    #     return np.concatenate([left_robot_effort, right_robot_effort])

    # def get_images(self):
    #     return self.image_recorder.get_images()

    # def set_gripper_pose(
    #     self, left_gripper_desired_pos_normalized, right_gripper_desired_pos_normalized
    # ):
    #     left_gripper_desired_joint = PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN(
    #         left_gripper_desired_pos_normalized
    #     )
    #     self.gripper_command.cmd = left_gripper_desired_joint
    #     self.puppet_bot_left.gripper.core.pub_single.publish(self.gripper_command)

    #     right_gripper_desired_joint = PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN(
    #         right_gripper_desired_pos_normalized
    #     )
    #     self.gripper_command.cmd = right_gripper_desired_joint
    #     self.puppet_bot_right.gripper.core.pub_single.publish(self.gripper_command)

    # def _reset_joints(self):
    #     reset_position = START_ARM_POSE[:6]
    #     move_arms(
    #         [self.puppet_bot_left, self.puppet_bot_right],
    #         [reset_position, reset_position],
    #         move_time=1,
    #     )

    # def _reset_gripper(self):
    #     """Set to position mode and do position resets: first open then close. Then change back to PWM mode"""
    #     move_grippers(
    #         [self.puppet_bot_left, self.puppet_bot_right],
    #         [PUPPET_GRIPPER_JOINT_OPEN] * 2,
    #         move_time=0.5,
    #     )
    #     move_grippers(
    #         [self.puppet_bot_left, self.puppet_bot_right],
    #         [PUPPET_GRIPPER_JOINT_CLOSE] * 2,
    #         move_time=1,
    #     )

    def get_observation(self):
        obs = collections.OrderedDict()
        obs["qpos"] = self.get_qpos()
        # obs["qvel"] = self.get_qvel()
        # obs["effort"] = self.get_effort()
        # obs["images"] = self.get_images()
        return obs

    def get_reward(self):
        return 0

    def reset(self, fake=False):
        if not fake:
            # Reboot puppet robot gripper motors
            self.puppet_bot_left.dxl.robot_reboot_motors("single", "gripper", True)
            self.puppet_bot_right.dxl.robot_reboot_motors("single", "gripper", True)
            self._reset_joints()
            self._reset_gripper()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation(),
        )

    def step(self, action):
        puppet_robots = self.robot_manager.puppet_robot_names
        state_len = int(len(action) / len(puppet_robots))

        # Set the goal pos for all puppet bots
        for puppet_index, puppet_bot_name in enumerate(puppet_robots):
            joint_index = NUM_JOINTS_ON_ROBOT * puppet_index
            action_for_puppet = action[joint_index : joint_index + NUM_JOINTS_ON_ROBOT]
            self.robot_manager.set_robot_goal_pos(
                puppet_bot_name, action_for_puppet[:NUM_JOINTS_ON_ROBOT]
            )

        time.sleep(DELTA_TIME_STEP)
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation(),
        )

    def get_action(self) -> NDArray[np.int32]:
        leader_robots = self.robot_manager.leader_robot_names
        action = np.zeros(
            (NUM_JOINTS_ON_ROBOT) * len(leader_robots), dtype=np.int32
        )  # joints + 1 gripper, for each arm

        # Get all the leader robot actions
        for robot_index, leader_robot in enumerate(leader_robots):
            robot_action = self.robot_manager.get_robot_pos(leader_robot)[
                :NUM_JOINTS_ON_ROBOT
            ]
            # Update the list of joint positions to the action array
            joint_index = NUM_JOINTS_ON_ROBOT * robot_index
            action[joint_index : NUM_JOINTS_ON_ROBOT + joint_index] = robot_action

        return action


def make_real_env(robot_manager: RobotManager, setup_robots=True) -> RealEnv:
    env = RealEnv(robot_manager, setup_robots)
    return env


# def test_real_teleop():
#     """
#     Test bimanual teleoperation and show image observations onscreen.
#     It first reads joint poses from both master arms.
#     Then use it as actions to step the environment.
#     The environment returns full observations including images.

#     An alternative approach is to have separate scripts for teleoperation and observation recording.
#     This script will result in higher fidelity (obs, action) pairs
#     """

#     onscreen_render = True
#     render_cam = "cam_left_wrist"

#     # source of data
#     master_bot_left = InterbotixManipulatorXS(
#         robot_model="wx250s",
#         group_name="arm",
#         gripper_name="gripper",
#         robot_name="master_left",
#         init_node=True,
#     )
#     master_bot_right = InterbotixManipulatorXS(
#         robot_model="wx250s",
#         group_name="arm",
#         gripper_name="gripper",
#         robot_name="master_right",
#         init_node=False,
#     )
#     setup_master_bot(master_bot_left)
#     setup_master_bot(master_bot_right)

#     # setup the environment
#     env = make_real_env(init_node=False)
#     ts = env.reset(fake=True)
#     episode = [ts]
#     # setup visualization
#     if onscreen_render:
#         ax = plt.subplot()
#         plt_img = ax.imshow(ts.observation["images"][render_cam])
#         plt.ion()

#     for t in range(1000):
#         action = get_action(master_bot_left, master_bot_right)
#         ts = env.step(action)
#         episode.append(ts)

#         if onscreen_render:
#             plt_img.set_data(ts.observation["images"][render_cam])
#             plt.pause(DELTA_TIME_STEP)
#         else:
#             time.sleep(DELTA_TIME_STEP)


# if __name__ == "__main__":
#     test_real_teleop()
