#!/usr/bin/env python
import math
import sys
import os
import time
import pybullet as p
from .simulation import Simulation

if len(sys.argv) > 1:
    directory = sys.argv[1]
else:
    print("Usage: onshape-to-robot-bullet [robot dir]")
    exit(1)

sim = Simulation(directory+'/robot.urdf', gui=True, panels=True)

controls = {}
for name in sim.getJoints():
    controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)

lastPrint = 0
while True:
    targets = {}
    for name in controls.keys():
        targets[name] = p.readUserDebugParameter(controls[name])
    sim.setJoints(targets)

    if time.time() - lastPrint > 0.05:
        lastPrint = time.time()
        os.system("clear")
        frames = sim.getFrames()
        for frame in frames:
            print(frame)
            print("- x=%f\ty=%f\tz=%f" % frames[frame][0])
            print("- r=%f\tp=%f\ty=%f" % frames[frame][1])
            print("")

    sim.tick()

