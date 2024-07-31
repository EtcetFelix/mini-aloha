import argparse
import os
import sys
import time
from typing import List

import IPython
import numpy as np
from tqdm import tqdm

from minialoha.scripts.real_env import make_real_env
from minialoha.utils.constants import (
    DELTA_TIME_STEP,
    LEFT_LEADER_BOT_NAME,
    LEFT_PUPPET_BOT_NAME,
    TASK_CONFIGS,
)
from minialoha.utils.data_utils import create_dataset_path, save_to_hdf5
from minialoha.utils.dynamixel import Dynamixel
from minialoha.utils.dynamixel_robot import DynamixelRobot
from minialoha.utils.robot_manager import RobotManager

e = IPython.embed

BAUDRATE = 1_000_000


# def opening_ceremony(
#     master_bot_left, master_bot_right, puppet_bot_left, puppet_bot_right
# ):
#     """Move all 4 robots to a pose where it is easy to start demonstration"""
#     # reboot gripper motors, and set operating modes for all motors
#     puppet_bot_left.dxl.robot_reboot_motors("single", "gripper", True)
#     puppet_bot_left.dxl.robot_set_operating_modes("group", "arm", "position")
#     puppet_bot_left.dxl.robot_set_operating_modes(
#         "single", "gripper", "current_based_position"
#     )
#     master_bot_left.dxl.robot_set_operating_modes("group", "arm", "position")
#     master_bot_left.dxl.robot_set_operating_modes("single", "gripper", "position")
#     # puppet_bot_left.dxl.robot_set_motor_registers("single", "gripper", 'current_limit', 1000) # TODO(tonyzhaozh) figure out how to set this limit

#     puppet_bot_right.dxl.robot_reboot_motors("single", "gripper", True)
#     puppet_bot_right.dxl.robot_set_operating_modes("group", "arm", "position")
#     puppet_bot_right.dxl.robot_set_operating_modes(
#         "single", "gripper", "current_based_position"
#     )
#     master_bot_right.dxl.robot_set_operating_modes("group", "arm", "position")
#     master_bot_right.dxl.robot_set_operating_modes("single", "gripper", "position")
#     # puppet_bot_left.dxl.robot_set_motor_registers("single", "gripper", 'current_limit', 1000) # TODO(tonyzhaozh) figure out how to set this limit

#     torque_on(puppet_bot_left)
#     torque_on(master_bot_left)
#     torque_on(puppet_bot_right)
#     torque_on(master_bot_right)

#     # move arms to starting position
#     start_arm_qpos = START_ARM_POSE[:6]
#     move_arms(
#         [master_bot_left, puppet_bot_left, master_bot_right, puppet_bot_right],
#         [start_arm_qpos] * 4,
#         move_time=1.5,
#     )
#     # move grippers to starting position
#     move_grippers(
#         [master_bot_left, puppet_bot_left, master_bot_right, puppet_bot_right],
#         [MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE] * 2,
#         move_time=0.5,
#     )

#     # press gripper to start data collection
#     # disable torque for only gripper joint of master robot to allow user movement
#     master_bot_left.dxl.robot_torque_enable("single", "gripper", False)
#     master_bot_right.dxl.robot_torque_enable("single", "gripper", False)
#     print("Close the gripper to start")
#     close_thresh = -0.3
#     pressed = False
#     while not pressed:
#         gripper_pos_left = get_arm_gripper_positions(master_bot_left)
#         gripper_pos_right = get_arm_gripper_positions(master_bot_right)
#         if (gripper_pos_left < close_thresh) and (gripper_pos_right < close_thresh):
#             pressed = True
#         time.sleep(DELTA_TIME_STEP / 10)
#     torque_off(master_bot_left)
#     torque_off(master_bot_right)
#     print("Started!")


def instantiate_robots() -> RobotManager:
    # source of data
    left_leader_dynamixel = Dynamixel.Config(
        baudrate=BAUDRATE, device_name="COM6"
    ).instantiate()
    puppet_dynamixel_left = Dynamixel.Config(
        baudrate=1_000_000, device_name="COM3"
    ).instantiate()

    puppet_bot_left = DynamixelRobot(puppet_dynamixel_left, servo_ids=[1, 2, 3, 4, 5])

    master_bot_left = DynamixelRobot(
        left_leader_dynamixel, servo_ids=[11, 12, 13, 14, 15]
    )

    robot_manager = RobotManager(
        robots={
            LEFT_LEADER_BOT_NAME: master_bot_left,
            LEFT_PUPPET_BOT_NAME: puppet_bot_left,
        },
        leader_robot_names=[
            LEFT_LEADER_BOT_NAME,
        ],
        puppet_robot_names=[
            LEFT_PUPPET_BOT_NAME,
        ],
    )
    return robot_manager


def prepare_data_for_export(camera_names, actions, timesteps):
    data_dict = {
        "/observations/qpos": [],
        # "/observations/qvel": [],
        # "/observations/effort": [],
        "/action": [],
    }
    for cam_name in camera_names:
        data_dict[f"/observations/images/{cam_name}"] = []

    # len(action): max_timesteps, len(time_steps): max_timesteps + 1
    while actions:
        action = actions.pop(0)
        timestep = timesteps.pop(0)
        data_dict["/observations/qpos"].append(timestep.observation["qpos"])
        # data_dict["/observations/qvel"].append(timestep.observation["qvel"])
        # data_dict["/observations/effort"].append(timestep.observation["effort"])
        data_dict["/action"].append(action)
        for cam_name in camera_names:
            data_dict[f"/observations/images/{cam_name}"].append(
                timestep.observation["images"][cam_name]
            )
    return data_dict


def capture_one_episode(
    dt,
    max_timesteps: int,
    camera_names: List[str],
    dataset_dir,
    dataset_name: str,
    overwrite: bool,
    robot_manager: RobotManager,
):
    print(f"Dataset name: {dataset_name}")

    env = make_real_env(robot_manager, setup_robots=False)

    dataset_path = create_dataset_path(dataset_dir, dataset_name + ".hdf5", overwrite)

    # move all 4 robots to a starting pose where it is easy to start teleoperation, then wait till both gripper closed
    # opening_ceremony(
    #     master_bot_left, master_bot_right, env.puppet_bot_left, env.puppet_bot_right
    # )

    # Data collection
    timestep = env.reset(fake=True)
    timesteps = [timestep]
    actions = []
    actual_dt_history = []
    for _ in tqdm(range(max_timesteps)):
        t0 = time.time()
        user_action = env.get_action()
        t1 = time.time()
        timestep = env.step(user_action)
        t2 = time.time()
        timesteps.append(timestep)
        actions.append(user_action)
        actual_dt_history.append([t0, t1, t2])

    # # Torque on both master bots
    # torque_on(master_bot_left)
    # torque_on(master_bot_right)
    # # Open puppet grippers
    # move_grippers(
    #     [env.puppet_bot_left, env.puppet_bot_right],
    #     [PUPPET_GRIPPER_JOINT_OPEN] * 2,
    #     move_time=0.5,
    # )

    freq_mean = print_dt_diagnosis(actual_dt_history)
    # if freq_mean < 42:
    #     return False

    """
    For each timestep:
    observations
    - images
        - cam_high          (480, 640, 3) 'uint8'
        - cam_low           (480, 640, 3) 'uint8'
        - cam_left_wrist    (480, 640, 3) 'uint8'
        - cam_right_wrist   (480, 640, 3) 'uint8'
    - qpos                  (14,)         'float64'
    - qvel                  (14,)         'float64'

    action                  (14,)         'float64'
    """

    data_dict = prepare_data_for_export(camera_names, actions, timesteps)

    save_to_hdf5(data_dict, dataset_path, camera_names, max_timesteps)

    return True


def main(args, robot_manager: RobotManager):
    task_config = TASK_CONFIGS[args["task_name"]]
    dataset_dir = task_config["dataset_dir"]
    max_timesteps = task_config["episode_len"]
    camera_names = task_config["camera_names"]

    if args["episode_idx"] is not None:
        episode_idx = args["episode_idx"]
    else:
        episode_idx = get_auto_index(dataset_dir)
    overwrite = True

    dataset_name = f"episode_{episode_idx}"
    print(dataset_name + "\n")
    while True:
        is_healthy = capture_one_episode(
            DELTA_TIME_STEP,
            max_timesteps,
            camera_names,
            dataset_dir,
            dataset_name,
            overwrite,
            robot_manager,
        )
        is_healthy = True  # TODO: REMOVE
        if is_healthy:
            break


def get_auto_index(dataset_dir, dataset_name_prefix="", data_suffix="hdf5"):
    max_idx = 1000
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    for i in range(max_idx + 1):
        if not os.path.isfile(
            os.path.join(dataset_dir, f"{dataset_name_prefix}episode_{i}.{data_suffix}")
        ):
            return i
    raise Exception(f"Error getting auto index, or more than {max_idx} episodes")


def print_dt_diagnosis(actual_dt_history):
    actual_dt_history = np.array(actual_dt_history)
    get_action_time = actual_dt_history[:, 1] - actual_dt_history[:, 0]
    step_env_time = actual_dt_history[:, 2] - actual_dt_history[:, 1]
    total_time = actual_dt_history[:, 2] - actual_dt_history[:, 0]

    dt_mean = np.mean(total_time)
    dt_std = np.std(total_time)
    freq_mean = 1 / dt_mean
    print(
        f"Avg freq: {freq_mean:.2f} Get action: {np.mean(get_action_time):.3f} Step env: {np.mean(step_env_time):.3f}"
    )
    return freq_mean


# def debug():
#     print("====== Debug mode ======")
#     recorder = Recorder("right", is_debug=True)
#     image_recorder = ImageRecorder(init_node=False, is_debug=True)
#     while True:
#         time.sleep(1)
#         recorder.print_diagnostics()
#         image_recorder.print_diagnostics()


if __name__ == "__main__":
    print("Starting main...")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--task_name", action="store", type=str, help="Task name.", required=True
    )
    parser.add_argument(
        "--episode_idx",
        action="store",
        type=int,
        help="Episode index.",
        default=None,
        required=False,
    )
    robot_manager = instantiate_robots()
    try:
        main(vars(parser.parse_args()), robot_manager)
    except KeyboardInterrupt:
        print("Shutdown requested...exiting")
        robot_manager.shutdown_gracefully()
        try:
            sys.exit(130)
        except SystemExit:
            os._exit(130)
    # debug()
