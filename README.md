# Robust Dynamic Tube MPC

This repo contains uncertain linear models and examples of applying time varying tube-based model predictive control (MPC). 

**Dependenices**: 
- [MATLAB controls toolbox](https://www.mathworks.com/products/control.html)
- [YALMIP](https://yalmip.github.io/tutorial/installation/)
- [MOSEK](https://www.mosek.com/products/academic-licenses/)
- [MPT3](https://www.mpt3.org/)


## Setup
```
>> startup
```

## Example 

```
>> run_spring_mass_damper
```

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