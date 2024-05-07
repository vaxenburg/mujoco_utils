"""Install script for setuptools."""

from setuptools import setup, find_packages


dev_requirements = [
    'yapf',
    'ruff',
    'jupyterlab',
    'tqdm',
]

requirements = [
    'dm_control',
    'pytest',
] + dev_requirements


setup(
    name='mujoco_utils',
    version='0.1',
    packages=find_packages(),
    python_requires='>=3.10',
    install_requires=requirements,
    author='Roman Vaxenburg',
    description='Utilities for MuJoCo MJCF models',
    url='https://github.com/vaxenburg/mujoco_utils',
)
