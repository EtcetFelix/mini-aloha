A cheaper small scale version of the [non-mobile Aloha robot](https://tonyzhaozh.github.io/aloha/)

Uses the low cost robotic arm designs by Alexander Koch: [low-cost-robot](https://github.com/AlexanderKoch-Koch/low_cost_robot)

## How to make your own:

1. **Create the robots**:
   * Follow the instructions on physically building the [low-cost-robot](https://github.com/AlexanderKoch-Koch/low_cost_robot) by Alexander Koch. Do this twice. After this step you should have 2 "leader" arms and 2 "follower" arms.
  
2. **Download and set up this repo**:
   * Clone this repo to a location of your choice: `git clone https://github.com/EtcetFelix/mini-aloha.git`
   * go into the mini-aloha folder: `cd mini-aloha`
   * Create and activate a venv:
     ```
     python -m venv .venv
     ./.venv/Scripts/activate.ps1  # Windows only
     ./.venv/bin/activate          # Linux & Mac only
     ```
   * install the necessary dependencies: `pip install .`
   * Run any of the scripts in the scripts folder. Ex: `python minialoha/scripts/one_side_teleop.py`
