# techtory cell description

## Creating Isaac Sim Environment

To create the Isaac Sim environment, run the following command:

```bash
mkdir -p colcon_tectory_ws/src 
cd colcon_tectory_ws
python3.11 -m venv .venv
source .venv/bin/activate
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
```

## Getting started

This package provides a URDF description of a techtory cell + cobotta robot. To visualize the cell in RViz, run:

```bash
cd colcon_techtory_ws
git clone git@gitlab.cc-asp.fraunhofer.de:ipa326/demonstrator/arena2036/dynamic_planning_demo.git src
cd src 
vcs import . < upstream.repo
cd ..
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src -iry
``` 

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