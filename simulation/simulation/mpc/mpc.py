import time
import numpy as np
import yaml
from simulation.mpc.gait import Gait
from simulation.mpc.plotting import PlotMode, animate, StatePlotter
import matplotlib.pyplot as plt
from tqdm import tqdm

from ismpc import FrameInfo, Reference, State, FootstepPlan
from ismpc import FootstepPlanProvider, ModelPredictiveController, FootTrajectoryGenerator, MovingConstraintProvider, FootstepSwitcher

from simulation.utils import config

# Representations
frame_info = FrameInfo()
reference = Reference()
state = State()
plan = FootstepPlan()

planner = FootstepPlanProvider(frame_info, reference, state, plan)
mpc = ModelPredictiveController(frame_info, state, plan)
ft_generator = FootTrajectoryGenerator(frame_info, state, plan)
mc_provider = MovingConstraintProvider(frame_info, state, plan)
switcher = FootstepSwitcher(frame_info, state, plan)

elapsed = []
iterations = range(config.N)
planner.update(plan)
# for footstep in plan.footsteps:
    # print(footstep)
# state_plotter = StatePlotter()

with tqdm(iterations, desc="Walking...") as pbar:
    for k in pbar:
        start = time.time()
        planner.update(plan)
        mc_provider.update(plan)
        switcher.update(state)
        mpc.update(state)
        ft_generator.update(state)
        elapsed.append((time.time() - start) * 1000)

        # Update the state
        state.lip = state.desired_lip
        state.left_foot = state.desired_left_foot
        state.right_foot = state.desired_right_foot

        # Update the time
        frame_info.tk += config.delta
        frame_info.k += 1
        pbar.set_description(f"Step: {k}, Time: {frame_info.tk:.3f}")
        # state_plotter.update_plot(state)
        print("\n")

print(f"Average elapsed time: {np.mean(elapsed):.3f} ms")
animate(Gait(state), save=False, plot_mode=PlotMode.TWO_D)
