# Isaac Sim Environment Setup

## Prerequisites

- Python 3.12
- CUDA-capable GPU with compatible drivers

## Installation

1. Create the workspace and virtual environment:

    ```bash
    mkdir -p colcon_techtory_ws/src
    cd colcon_techtory_ws
    python3.12 -m venv .venv
    source .venv/bin/activate
    ```

2. Install PyTorch with CUDA support:

    ```bash
    pip install torch==2.10.0 --index-url https://download.pytorch.org/whl/cu130
    ```

3. Install Isaac Sim:

    ```bash
    pip install isaacsim[all,extscache]==6.0.0 --extra-index-url https://pypi.nvidia.com
    ```

## Launching Isaac Sim

1. Activate the virtual environment:

    ```bash
    cd colcon_techtory_ws
    source .venv/bin/activate
    ```

2. Launch Isaac Sim:

    ```bash
    isaacsim
    ```

3. Open the USD file `techtory_cvrb0609_moveit-final.usd` located in `techtory_cobotta_workcell_description/urdf/`.

> **Note:** Replace paths with your actual Isaac Sim installation and USD file locations.
