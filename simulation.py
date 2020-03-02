#!/usr/bin/env python
import math
import sys
import time
import pybullet as p
from time import sleep

class Simulation:
    def __init__(self, directory):
        self.t = 0
        self.dt = 0.001

        # Instantiating Bullet
        physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -10)

        # Loading floor
        planeId = p.loadURDF('bullet/plane.urdf')

        # Loading robot
        cubeStartPos = [0, 0, 0.75]
        cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        # self.robot = p.loadURDF(directory+"/robot.urdf",
        #                     cubeStartPos, cubeStartOrientation)

        # In case we want to read SDF, but PyBullet can't do that properly so far
        # https://github.com/bulletphysics/bullet3/issues/2651
        # self.robot = p.loadSDF(directory+"/robot.sdf")[0]
        # p.resetBasePositionAndOrientation(self.robot, cubeStartPos, cubeStartOrientation)

        # Engine parameters
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt)
        p.setRealTimeSimulation(1)
        # p.setPhysicsEngineParameter(numSolverIterations=5)
        # p.setPhysicsEngineParameter(numSubSteps=1)

        # Retrieving joints and frames
        self.joints = {}
        self.frames = {}

        # Collecting the available joints
        for k in range(p.getNumJoints(self.robot)):
            jointInfo = p.getJointInfo(self.robot, k)
            name = jointInfo[1].decode('utf-8')
            if '_fixing' not in name:
                if '_frame' in name:
                    self.frames[name] = k
                else:
                    self.joints[name] = k

        print('* Found '+str(len(self.joints))+' DOFs')
        print('* Found '+str(len(self.frames))+' frames')

    def getFrames(self):
        frames = {}

        for name in self.frames.keys():
            jointState = p.getLinkState(self.robot, self.frames[name])
            pos = jointState[0]
            orientation = p.getEulerFromQuaternion(jointState[1])
            frames[name] = [pos, orientation]

        return frames

    def setJoints(self, joints):
        for name in joints.keys():
            if name in self.joints:
                p.setJointMotorControl2(
                    self.robot, self.joints[name], p.POSITION_CONTROL, joints[name])
            else:
                raise Exception("Can't find joint %s" % name)

    def getJoints(self):
        return self.joints.keys()

    def execute(self):
        while True:
            self.tick()

    def tick(self):
        self.t += self.dt
        p.stepSimulation()

