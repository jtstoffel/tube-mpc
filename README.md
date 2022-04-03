# Robust Dynamic Tube MPC

This repo contains uncertain discrete time linear models and examples of applying time-varying tube-based model predictive control (MPC). 
Global RRT/RRT* planner included for tube-to-tube steering with obstacles.

**Dependenices**: 
- [MATLAB controls toolbox](https://www.mathworks.com/products/control.html)
- [YALMIP](https://yalmip.github.io/tutorial/installation/)
- [MOSEK](https://www.mosek.com/products/academic-licenses/)
- [MPT3](https://www.mpt3.org/)

![fig](./figures/rrt_test5.png)
![fig](./figures/rrt_test5_states.png)

## Setup
```
startup
test(true)
```

## Example Use Cases
### Elastic Tube Optimal Control 
1. Edit initial state and elasticity weighting in example file
2. Set `useDataFile` to false to recompute tube approximation parameters if you want to see all the action 

```
run_planar_double_integrator
```

### Tube-to-Tube Sampling-Based Motion Planning 
1. Edit maximum iterations, initial/final nominal states, and ETOC solver tube length(s) in example file
2. Create additional obstacles using polytopes in `build_map.m`

```
run_rrt_example
```

## Data Structures
### Tube
| Fields  | Description | Size | Type |
| ------------- |:-------------:|-------------:|-------------:|
| `z`      |  Nominal state trajectory  | (nx,N) | float
| `v`      | Nominal control input sequence     |  (nu,N)| float
| `a`      | Cross section elasticitiy parameter sequence  |  (qs,N) | float
| `N`      | Number of time steps     | scalar| int
| `cost`      | Total cost for tube solution     | scalar | float
| `success`      | Valid solution success flag     | scalar | bool

### System
| Fields  | Description | Size | Type |
| ------------- |:-------------:|-------------:|-------------:|
| `A`      |  Linear discrete time system state matrix  | (nx,nx) | float
| `B`      |  Linear discrete time system control matrix  | (nx,nu) | float
| `x0`      |  Initial state  | (nx,1) | float
| `x_min`      |  State lower bound  | (nx,1) | float
| `x_max`      |  State upper bound  | (nx,1) | float
| `u_min`      |  Control lower bound  | (nu,1) | float
| `u_max`      |  Control upper bound  | (nu,1) | float
| `w_min`      |  Disturbance lower bound  | (nw,1) | float
| `w_max`      |  Disturbance upper bound  | (nw,1) | float
| `name`      |  System name (for plotting)  | 1 | str
| `nx`      |  Number of states  | 1 | float
| `nu`      |  Number of controls  | 1 | float
| `nw`      |  Number of disturbances  | 1 | float

## TODO
### March 2022
- [x] Speed up preprocessing
- [x] Add support for homothetic and fixed size tubes
- [x] Add different boundary condition options
    - [x] Set-based initial and final conditions
    - [x] Support mixed set and state boundary conditions
- [x] Verify objective function for non-zero goal state
- [x] Update stale models
    - [x] `double_integrator_model.m`
    - [x] `spring_mass_damper_model.m`
- [x] Add full state vs. time plotting to all postprocessing scripts
- [x] Decide on node and edge data for tube-to-tube RRT* tree
- [x] Add RRT planner

### April 2022
- [x] Add tube-to-tube ETOC steering to RRT planner
    - [x] Add tube obstacle collision check
    - [x] Decide on constant or decreasing prediction horizon
- [x] Decide on rewiring strategy for RRT* planner
- [x] Full example using basic map and `planar_double_integrator_model.m`
- [ ] RRT* planner (DOING)
- [ ] Add vehicle model
- [ ] Add additional map options to `build_map.m`
    - [ ] Map saving
    - [ ] Non-convex obstacles
- [ ] Additional example with more complex obstacle environment



## References
### ETMPC
```
@INPROCEEDINGS{7525471,
author={Raković, Sas̆a V. and Levine, William S. and Açikmese, Behçet}, 
booktitle={2016 American Control Conference (ACC)},   
title={Elastic tube model predictive control},   
year={2016},  
volume={},  
number={},  
pages={3594-3599},  
doi={10.1109/ACC.2016.7525471}}
```

### HTMPC
```
@INPROCEEDINGS{6561023,
author={Raković, Saša V. and Cheng, Qifeng},
booktitle={2013 25th Chinese Control and Decision Conference (CCDC)},
title={Homothetic tube MPC for constrained linear difference inclusions},
year={2013},
volume={},
number={},
pages={754-761},
doi={10.1109/CCDC.2013.6561023}}
```