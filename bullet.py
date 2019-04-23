#!/usr/bin/env python
import math
import sys
import pybullet as p
from simulation import Simulation

directory = 'robots/demo/'
if len(sys.argv) > 1:
    directory = sys.argv[1]

sim = Simulation(directory)

controls = {}
for name in sim.getJoints():
    controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)

while True:
    targets = {}
    for name in controls.keys():
        targets[name] = p.readUserDebugParameter(controls[name])
    sim.setJoints(targets)

    print('~')
    print(sim.getFrames())

