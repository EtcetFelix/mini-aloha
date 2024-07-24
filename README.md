# Mini-Aloha Robot


Join the discord to see the continued progress and roadmap of the build!

[![Discord](https://img.shields.io/badge/Discord-%235865F2.svg?style=for-the-badge&logo=discord&logoColor=white)](https://discord.gg/Yb4UHCaJqT) 


A cheaper small scale version of the [non-mobile Aloha robot](https://tonyzhaozh.github.io/aloha/)

Currently uses the low cost robotic arm designs by Alexander Koch: [low-cost-robot](https://github.com/AlexanderKoch-Koch/low_cost_robot)

Current Total Cost: $860

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

## Contributing:

If you'd like to contribute, the setup process is a little different. This repository is semi-packaged with poetry, so it's recommended that you install and use that to ensure your python environment is the same.

1. **Install poetry**:
   * Follow the [poetry installation guide](https://python-poetry.org/docs/#installing-with-the-official-installer)
  
2. **Clone this repo:** `git clone https://github.com/EtcetFelix/mini-aloha.git`
3. **go into the mini-aloha folder:** `cd mini-aloha`
4. **Install the dependencies using the poetry lock file:** `poetry install`
5. **Join the discord for roadmap updates, communications, and/or help with setting up:** [Join the Discord](https://discord.gg/Yb4UHCaJqT) 
---

Join the discord to see the continued progress and roadmap of the build!

[![Discord](https://img.shields.io/badge/Discord-%235865F2.svg?style=for-the-badge&logo=discord&logoColor=white)](https://discord.gg/Yb4UHCaJqT) 

