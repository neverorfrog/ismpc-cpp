# Intrinsically stable MPC

![](docs/ismpc.png)

## Installation

### Python Binding

To use also the python bindings, you need to call the setup.py script. The `pixi.toml` file will do that for you. Just run
```
pixi install
pixi run build
pixi run install
```

## Run

To run the code:
```
pixi run main
```

## Results

<video controls src="videos/2d/walking.mp4" title="Title"></video>
<video controls src="videos/3d/walking.mp4" title="Title"></video>


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
