# techtory cell description

## Getting started

This package provides a URDF description of a techtory cell + cobotta robot. To visualize the cell in RViz, run:

```bash
ros2 launch techtory_cvrb0609_workcell_description  display.launch.py
```
## Curobo Integration (WIP)

Quick Test:

```bash
 python motion_gen_reacher.py --robot ~/src/curobo_motion_generation/config/cobotta900.yml 
```
Note: 
* Make sure Light is Set to Camera Light in IsaacSim.
* You are inside right directory to execute `motion_gen_reacher`
* Make sure you use the full path for `cobotta900.yml` file.