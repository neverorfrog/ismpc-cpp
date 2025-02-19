# Intrinsically stable MPC

![](docs/ismpc.png)


## Simulations

To run the simulattions you can use the python bindings.

From the root folder run
```
pixi run preinstall
pixi run build
```

After that, from the folder simulation run
```
pixi run install-ismpc-cpp
pixi run typings
```

## Some Results

<video width="100%" controls>
  <source src="https://github.com/neverorfrog/ismpc-cpp/blob/main/simulation/videos/2d/walking.mp4" type="video/mp4">
</video>

<video width="100%" controls>
  <source src="https://github.com/neverorfrog/ismpc-cpp/blob/main/simulation/videos/2d/walking_curve.mp4" type="video/mp4">
</video>

## Some Concepts

### Footstep Planner

To generate the candidate footstep poses, we use a reference trajectory obtained
by integrating a template model under the action of the high-level reference
velocities. After that, we solve two QP problems to find the poses. The template
model is an omnidirectional motion model which allows the template robot to move
along any Cartesian path with any orientation, so as to perform, e.g., lateral
walks, diagonal walks, and so on. A single step has duration $T$.

#### Breakdown of Time Phases Within $T$:

2. **Single Support Phase:**
   - **Duration:** The single support phase occupies the initial 70% of the duration $T$.
   - **Timing:**
     - It begins right after the first double support phase ends, so it starts at $t = jT$ and ends at $t = jT + 0.7T$ where $j$ is the footstep index.
     - During this time, only one foot is in contact with the ground.

2. **Double Support Phase:**
   - **Duration:** The duration of the double support phase is $0.30T$.
   - **Timing:**
     - The jth double support phase starts at $t = jT + 0.7T$ and ends at $t = jT + T$.

#### Input-Output:

- **Input**
  - Current time $t_k$
  - State, containing information on support foot, walk phase and timestamp about the last footstep

- **Output**
  - F planned footsteps over the planning horizon P with associated timestamps and proposed poses
  - Moving Constraints (dim C) for every timestep inside control horizon with linear interpolation in single support phase


## References

<a id="1">[1]</a>
[N. Scianca, D. De Simone, L. Lanari and G. Oriolo, "MPC for Humanoid Gait Generation: Stability and Feasibility," in IEEE Transactions on Robotics, vol. 36, no. 4, pp. 1171-1188, Aug. 2020](https://ieeexplore.ieee.org/abstract/document/8955951)
