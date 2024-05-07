# mujoco_utils
Tools for working with MuJoCo models

## Installation
To install into an existing conda environment, follow these steps:
```bash
conda clone https://github.com/vaxenburg/mujoco_utils.git
cd mujoco_utils
pip install -e .
```
Or create a new environment (e.g. `mj-dev`) and then install:
```bash
conda clone https://github.com/vaxenburg/mujoco_utils.git
cd mujoco_utils
conda create --name mj-dev python=3.10 pip
conda activate mj-dev
pip install -e .
```
