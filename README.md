# techtory cell description

## Getting started

This package provides a URDF description of a techtory cell + cobotta robot. To visualize the cell in RViz, run:

```bash
ros2 launch techtory_cvrb0609_workcell_description  display.launch.py
```
## Curobo Integration

Prerequisite:-
 - You already have Curobo Installed. If not, please follow link :-
 https://curobo.org/get_started/1_install_instructions.html

Quick Test:

```bash
 python motion_gen_reacher.py --robot ~/src/curobo_motion_generation/config/cobotta900.yml 
```
Note:
* You are inside right directory to execute `motion_gen_reacher`
* Make sure you update the variable `--external_asset_path` in `motion_gen_reacher.py` with correct directory

Sample full path command :-

```bash
 python src/curobo_motion_generation/curobo_motion_generation/motion_gen_reacher.py --robot /home/ipa326/dynamic_planning_demo/src/curobo_motion_generation/config/cobotta900.yml
```