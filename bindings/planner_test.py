import yaml
from ismpc_py import WalkEngine, State
from omegaconf import OmegaConf
from tqdm import tqdm


def load_config(file_path):
    with open(file_path, "r") as file:
        config = OmegaConf.create(yaml.safe_load(file))
    return config

file_path = "config/config.yaml"
config = load_config(file_path)
robot_file_path = f"config/robots/{config.robot}.yaml"
robot_config = load_config(robot_file_path)

state = State()
engine = WalkEngine(state)

elapsed = []
iterations = range(config.N)

desired_state = State()

with tqdm(iterations, desc="Walking...") as pbar:
    for k in pbar:
        print(f"Step: {k}, Time: {engine.get_frame_info().tk:.3f}")
        engine.update(desired_state)
        plan = engine.get_plan()
        


