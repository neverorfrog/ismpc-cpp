import time

import numpy as np
import yaml
from gait import Gait
from ismpc_cpp import WalkEngine
from omegaconf import OmegaConf
from plotting import PlotMode, animate
from tqdm import tqdm


def load_config(file_path):
    with open(file_path, "r") as file:
        config = OmegaConf.create(yaml.safe_load(file))
    return config


file_path = "config/config.yaml"
config = load_config(file_path)
robot_file_path = f"config/robots/{config.robot}.yaml"
robot_config = load_config(robot_file_path)

engine = WalkEngine()

elapsed = []
iterations = range(config.N)
with tqdm(iterations, desc="Walking...") as pbar:
    for k in pbar:
        start = time.time()
        engine.update()
        print(engine.get_robot().state)
        print(engine.get_robot().walk)
        elapsed.append((time.time() - start) * 1000)
        pbar.set_description(f"Step: {k}, Time: {engine.get_frame_info().tk:.3f}")
sim_data = engine.get_frame_info()
print(f"Average elapsed time: {np.mean(elapsed):.3f} ms")
print("Current time: ", sim_data.tk)
animate(Gait(engine), config, robot_config, save=False, plot_mode=PlotMode.TWO_D)
