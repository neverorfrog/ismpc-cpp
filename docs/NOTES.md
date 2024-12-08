# Notes

## Nomenclature

- Sampling interval $\delta$
- Preview horizon $T_p = P \cdot \delta$
- Control horizon $T_c = C \cdot \delta$
- Generic time instant $t_k = k \cdot \delta$
- $j$ runs over the footsteps from $0$ to $F$ ($F$ is determined by timings computation)
- $i$ runs over the control steps from $0$ to $C$

## Main Components

- External source
  - Input: external source
  - Output: desired driving and steering velocities $v_x, v_y, \omega$ over $[t_k, t_k+T_p]$
  - How: high-level planning
- Footstep planner
  - Input: $v_x, v_y, \omega$ over $[t_k, t_k+T_p]$
  - Output:
    - Candidate footstep positions ($\hat X_f^k$,$\hat Y_f^k$) which are gonna be changed by ISMPC
    - Footstep orientations $\Theta_f^k$ which remain as such also for ISMPC
    - Footstep timestamps
  - How: quadratic programming
- Intrinsically Stable MPC
  - Input: Candidate footsteps over $T_p$
  - Output: Actual footstep trajectory ($X_f^k$,$Y_f^k$) and CoM trajectory $p^*_c$ over $T_c$, with $T_c \leq T_p$
  - How: [ISMPC](#ismpc)

## ISMPC

### Prediction Model
- Linear inverted pendulum
  - $\ddot x_c = \eta^2(x_c-x_z)$
  - $\eta = \sqrt{\frac{g}{h_c}}$
- The input commands are ZMP velocities (could be also ZMP positions)

### Constraints
#### Kinematic Feasibility
#### ZMP
#### Stability
- Dynamical system can be decomposed into two eigenvalues, generating two fictituous coordinates that describe the evolution of CoM
- By controlling the unstable component, we obtain the stability constraint on the CoM's position, which uses future positions and velocities of the ZMP

### Workflow
- MPC receives the footsteps
- Internally, it generates velocity inputs for the ZMP (treated as decision variables)
- These velocities are then (still inside the MPC) tracked by the CoM
- The final result is the CoM trajectory and the actual footsteps (they are also decision variables)
- The CoM trajectory is then tracked by an inverse kinematics controller
