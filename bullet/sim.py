import math
import pybullet as p
from time import sleep
from control import jointsPosition

# Instanciation de Bullet
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -10)

# Chargement du sol
planeId = p.loadURDF('plane.urdf')

# Chargement du robot
cubeStartPos = [0, 0, 0.1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot = p.loadURDF("../urdf/robot.urdf",
                       cubeStartPos, cubeStartOrientation)
cubePos, cubeOrn = p.getBasePositionAndOrientation(robot)

# Map des joints
jointsMap = []
t = 0
dt = 0.01
p.setPhysicsEngineParameter(fixedTimeStep=dt)

k = 0
try:
    while True:
        jointInfo = p.getJointInfo(robot, k)
        if 'fixing' not in jointInfo[1].decode('utf-8'):
            jointsMap.append(k)
        k += 1
except p.error:
    pass

while True:
    t += dt
    joints = jointsPosition(t)
    for k in range(len(joints)):
        jointInfo = p.getJointInfo(robot, jointsMap[k])
        p.setJointMotorControl2(
            robot, jointInfo[0], p.POSITION_CONTROL, joints[k])
    p.stepSimulation()
    sleep(dt)
