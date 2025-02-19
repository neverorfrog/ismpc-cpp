import time
import numpy as np
from simulation.mpc.gait import Gait
from simulation.mpc.plotting import PlotMode, animate
from tqdm import tqdm
from ismpc import FrameInfo, Reference, State, FootstepPlan
from ismpc import FootstepPlanProvider, ModelPredictiveController, FootTrajectoryGenerator, MovingConstraintProvider

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

elapsed = []
iterations = range(config.N)

for k in iterations:
    start = time.time()
    planner.update(plan)
    mc_provider.update(plan)
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
    print("==========================================================")
    print(f"Step: {frame_info.k} Time: {frame_info.tk} \n")
    print(f"CURRENT FOOTSTEP: \n{plan.footsteps[0]}")
    print(f"SUPPORT PHASE: {plan.support_phase} \n")
    print(f"DESIRED LEFT FOOT: {state.desired_left_foot.pose.translation}")
    print(f"DESIRED RIGHT FOOT: {state.desired_right_foot.pose.translation}")
    
    print("---------------------------------------------")
    print(f"NEXT FOOTSTEPS: \n")
    for i in range(1, len(plan.footsteps)):
        print(f"Footstep {i}: \n{plan.footsteps[i]}")
    print("---------------------------------------------")
    
    print("==========================================================\n\n")

print(f"Average elapsed time: {np.mean(elapsed):.3f} ms")
animate(Gait(state, plan), save=True, plot_mode=PlotMode.TWO_D)
