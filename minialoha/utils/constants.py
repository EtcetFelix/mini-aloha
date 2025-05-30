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
        "camera_names": [],
        # "camera_names": ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"],
    },
}

### ALOHA fixed constants
DELTA_TIME_STEP = 0.02
LEFT_LEADER_BOT_NAME = "leader_bot_left"
RIGHT_LEADER_BOT_NAME = "right_leader_bot"
LEFT_PUPPET_BOT_NAME = "left_puppet_bot"
RIGHT_PUPPET_BOT_NAME = "right_puppet_bot"

# Left finger position limits (qpos[7]), right_finger = -1 * left_finger
MASTER_GRIPPER_POSITION_OPEN = 0.02417
MASTER_GRIPPER_POSITION_CLOSE = 0.01244
PUPPET_GRIPPER_POSITION_OPEN = 0.05800
PUPPET_GRIPPER_POSITION_CLOSE = 0.0184

# Gripper joint limits (qpos[6])
MASTER_GRIPPER_JOINT_OPEN = 0.3083
MASTER_GRIPPER_JOINT_CLOSE = -0.6842
PUPPET_GRIPPER_JOINT_OPEN = 1.4910
PUPPET_GRIPPER_JOINT_CLOSE = -0.0

############################ Helper functions ############################


def prep_robots(leader: Robot, follower: Robot):
    # Make sure to manually adjust both robots to these positions
    # Otherwise they'll move crazy fast and can potentially damage wires
    goal_pos = [
        1000,
        1796,
        1629,
        1003,
        2198,
    ]
    leader.set_goal_position(goal_pos)
    print(f"Leader position: {leader.read_position()}")
    follower.set_goal_position(goal_pos)
    print(f"Follower position: {follower.read_position()}")


NUM_LEADER_ROBOTS = 1
NUM_PUPPET_ROBOTS = 1
NUM_JOINTS_ON_ROBOT = 5
GRIPPER_INDEX = 4
PUPPET_GRIPPER_POSITION_NORMALIZE_FN = lambda x: (x - PUPPET_GRIPPER_POSITION_CLOSE) / (
    PUPPET_GRIPPER_POSITION_OPEN - PUPPET_GRIPPER_POSITION_CLOSE
)

MASTER_GRIPPER_JOINT_NORMALIZE_FN = lambda x: (x - MASTER_GRIPPER_JOINT_CLOSE) / (
    MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE
)
