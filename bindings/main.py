import time

import numpy as np
import yaml
from gait import Gait
from omegaconf import OmegaConf
from plotting import PlotMode, animate, StatePlotter
import matplotlib.pyplot as plt
from tqdm import tqdm

from ismpc_py import FrameInfo, Reference, State, FootstepPlan
from ismpc_py import FootstepPlanProvider, ModelPredictiveController, FootTrajectoryGenerator, MovingConstraintProvider, CasadiMPC

def load_config(file_path):
    with open(file_path, "r") as file:
        config = OmegaConf.create(yaml.safe_load(file))
    return config

file_path = "config/config.yaml"
config = load_config(file_path)
robot_file_path = f"config/robots/{config.robot}.yaml"
robot_config = load_config(robot_file_path)

# Representations
frame_info = FrameInfo()
reference = Reference()
state = State()
plan = FootstepPlan()

planner = FootstepPlanProvider(frame_info, reference, state, plan)
mpc = ModelPredictiveController(frame_info, state, plan)
casadi_mpc = CasadiMPC(frame_info, state, plan)
ft_generator = FootTrajectoryGenerator(frame_info, state, plan)
mc_provider = MovingConstraintProvider(frame_info, state, plan)

elapsed = []
iterations = range(config.N)
planner.update(plan)
# state_plotter = StatePlotter()

with tqdm(iterations, desc="Walking...") as pbar:
    for k in pbar:
        start = time.time()
        mc_provider.update(plan)
        mpc.update(state)

        print("TIME:", frame_info.tk)
        print("LIP", state.lip)
        print("DESIRED LIP", state.desired_lip)
        print("FOOTSTEP:\n", state.footstep)

        ft_generator.update(state)

        print("LEFT FOOT", state.desired_left_foot.pose.translation)
        print("RIGHT FOOT", state.desired_right_foot.pose.translation)

        elapsed.append((time.time() - start) * 1000)

        # Update the time
        frame_info.tk += config.delta
        frame_info.k += 1

        # print("DESIRED LIP", state.desired_lip)

        # Update the state
        state.lip = state.desired_lip
        state.left_foot = state.desired_left_foot
        state.right_foot = state.desired_right_foot

        # pbar.set_description(f"Step: {k}, Time: {frame_info.tk:.3f}")

        # state_plotter.update_plot(state)
        print("\n")

print(f"Average elapsed time: {np.mean(elapsed):.3f} ms")
animate(Gait(state), config, robot_config, save=False, plot_mode=PlotMode.TWO_D)
