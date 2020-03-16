# Model Predictive Control Design Project

## Nomenclature
Below are some standard notations used in all functions in the project for easier readability.
Since the dynamics of the quadcopter are decoupled, two state vectors are defined:
- `x_ang`: state vector for rotational dynamics, i.e. `[p q r phi theta psi]'`
- `x_pos`: state vector for translational dynamics, i.e. `[dx/dt dy/dt dz/dt x y z]'`
Likewise, two input vectors
- `u_ang`: input vector for rotational dynamics, i.e. `[u1 u2 u3 u4]`
- `u_pos`: input vector for translational dynamics, i.e. `[u1 phi theta psi]` (the attitude of the drone is considered an input to the translational dynamics)

## Todo

### MPC Design
#### Basic
- Determine sample rates (Elke)
- System linearization for MPC formulation (Emiel)
- MPC Problem formulation (Elke)
  - Prediction matrices
  - Cost function (final cost?)
- Solve MPC routine with YALMIP

#### Extension
- Add disturbances
- Limit state knowledge


