import os

from dotenv import load_dotenv

from minialoha.utils.robot import Robot

load_dotenv()

### Task parameters
DATA_DIR = os.getenv("DATA_DIR", default="~/.minialoha/data")
TASK_CONFIGS = {
    "aloha_wear_shoe": {
        "dataset_dir": DATA_DIR + "/aloha_wear_shoe",
        "num_episodes": 50,
        "episode_len": 1000,
        "camera_names": ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"],
    },
}

### ALOHA fixed constants
DELTA_TIME_STEP = 0.02
