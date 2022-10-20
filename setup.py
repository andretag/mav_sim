#!/usr/bin/env python

from setuptools import setup, find_packages

setup(name='mav_sim',
      version='1.0',
      description='Minimal Python/Pybullet-based Simulator for Micro Aerial Vehicles',
      author='Andrea Tagliabue',
      author_email='atagliab@mit.edu',
      python_requires=">=3.7.0",
      packages=find_packages(include=['mav_sim', 'mav_sim.*']),
      install_requires=[
        "pybullet>=3.2.5",
        "numpy>=1.23.1",
        "yacs>=0.1.8"
      ],
     )
