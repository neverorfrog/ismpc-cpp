import time
from typing import List
import numpy as np
from simulation.planner.plan import Plan
from simulation.planner.plotting import animate

from ismpc import FrameInfo, Reference, State, FootstepPlan, Footstep, SupportPhase, Foot
from ismpc import FootstepPlanProvider, MovingConstraintProvider, FootstepSwitcher 

from simulation.utils import config

# Representations
frame_info = FrameInfo()
reference = Reference()
state = State()
plan = FootstepPlan()

planner = FootstepPlanProvider(frame_info, reference, state, plan)
moving_constraints = MovingConstraintProvider(frame_info, state, plan)
switcher = FootstepSwitcher(frame_info, state, plan)

elapsed = []
iterations = range(config.N)
phase_switched = False

fs_history: List[Footstep] = []
fs_history.append(state.footstep)

# with tqdm(iterations, desc="Walking...") as pbar:
for k in iterations:
    start = time.time()
    planner.update(plan)
    moving_constraints.update(plan)
    switcher.update(state)
    elapsed.append((time.time() - start) * 1000)
    
    # Update the time
    frame_info.tk += config.delta
    frame_info.k += 1
    # pbar.set_description(f"Step: {k}, Time: {frame_info.tk:.3f}")
    # print("----------------------------------------------------------------")


for fs in state.fs_history:
    print(fs)
    
print(f"Average elapsed time: {np.mean(elapsed):.3f} ms")
animate(Plan(state), save=False)
