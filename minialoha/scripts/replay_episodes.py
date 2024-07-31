import argparse
import os

import h5py
import IPython
from dotenv import load_dotenv

from minialoha.scripts.real_env import make_real_env
from minialoha.utils.robot_manager import RobotManager

e = IPython.embed


def instantiate_robots() -> RobotManager:
    robot_manager = RobotManager()
    return robot_manager


def main(args):
    dataset_dir = args["dataset_dir"]
    episode_idx = args["episode_idx"]
    dataset_name = f"episode_{episode_idx}"

    robot_manager = instantiate_robots()

    dataset_path = os.path.join(dataset_dir, dataset_name + ".hdf5")
    if not os.path.isfile(dataset_path):
        print(f"Dataset does not exist at \n{dataset_path}\n")
        exit()

    with h5py.File(dataset_path, "r") as root:
        actions = root["/action"][()]

    env = make_real_env(robot_manager)
    env.reset()
    for action in actions:
        env.step(action)


if __name__ == "__main__":
    load_dotenv()
    parser = argparse.ArgumentParser()
    # set data_dir default if none is given in command line
    data_dir = os.environ.get("DATA_DIR")
    parser.add_argument(
        "--dataset_dir",
        action="store",
        type=str,
        help="Dataset dir.",
        required=False,
        default=data_dir,
    )
    parser.add_argument(
        "--episode_idx", action="store", type=int, help="Episode index.", required=False
    )
    main(vars(parser.parse_args()))
