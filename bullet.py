#!/usr/bin/env python
import math
import sys
import pybullet as p
from time import sleep

directory = 'robots/demo/'
if len(sys.argv) > 1:
    directory = sys.argv[1]

def drawInertiaBox(parentUid, parentLinkIndex, color):
	dyn = p.getDynamicsInfo(parentUid, parentLinkIndex)
	mass=dyn[0]
	frictionCoeff=dyn[1]
	inertia = dyn[2]
	if (mass>0):
		Ixx = inertia[0]
		Iyy = inertia[1]
		Izz = inertia[2]
		boxScaleX = 0.5*math.sqrt(6*(Izz + Iyy - Ixx) / mass);
		boxScaleY = 0.5*math.sqrt(6*(Izz + Ixx - Iyy) / mass);
		boxScaleZ = 0.5*math.sqrt(6*(Ixx + Iyy - Izz) / mass);
  
		halfExtents = [boxScaleX,boxScaleY,boxScaleZ]
		pts = [[halfExtents[0],halfExtents[1],halfExtents[2]],
				 [-halfExtents[0],halfExtents[1],halfExtents[2]],
				 [halfExtents[0],-halfExtents[1],halfExtents[2]],
				 [-halfExtents[0],-halfExtents[1],halfExtents[2]],
				 [halfExtents[0],halfExtents[1],-halfExtents[2]],
				 [-halfExtents[0],halfExtents[1],-halfExtents[2]],
				 [halfExtents[0],-halfExtents[1],-halfExtents[2]],
				 [-halfExtents[0],-halfExtents[1],-halfExtents[2]]]
	
		
		p.addUserDebugLine(pts[0],pts[1],color,1, parentObjectUniqueId=parentUid, parentLinkIndex = parentLinkIndex)
		p.addUserDebugLine(pts[1],pts[3],color,1, parentObjectUniqueId=parentUid, parentLinkIndex = parentLinkIndex)
		p.addUserDebugLine(pts[3],pts[2],color,1, parentObjectUniqueId=parentUid, parentLinkIndex = parentLinkIndex)
		p.addUserDebugLine(pts[2],pts[0],color,1, parentObjectUniqueId=parentUid, parentLinkIndex = parentLinkIndex)
	
		p.addUserDebugLine(pts[0],pts[4],color,1, parentObjectUniqueId=parentUid, parentLinkIndex = parentLinkIndex)
		p.addUserDebugLine(pts[1],pts[5],color,1, parentObjectUniqueId=parentUid, parentLinkIndex = parentLinkIndex)
		p.addUserDebugLine(pts[2],pts[6],color,1, parentObjectUniqueId=parentUid, parentLinkIndex = parentLinkIndex)
		p.addUserDebugLine(pts[3],pts[7],color,1, parentObjectUniqueId=parentUid, parentLinkIndex = parentLinkIndex)
	
		p.addUserDebugLine(pts[4+0],pts[4+1],color,1, parentObjectUniqueId=parentUid, parentLinkIndex = parentLinkIndex)
		p.addUserDebugLine(pts[4+1],pts[4+3],color,1, parentObjectUniqueId=parentUid, parentLinkIndex = parentLinkIndex)
		p.addUserDebugLine(pts[4+3],pts[4+2],color,1, parentObjectUniqueId=parentUid, parentLinkIndex = parentLinkIndex)
		p.addUserDebugLine(pts[4+2],pts[4+0],color,1, parentObjectUniqueId=parentUid, parentLinkIndex = parentLinkIndex)


# Instanciation de Bullet
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -10)

# Chargement du sol
planeId = p.loadURDF('bullet/plane.urdf')

# Chargement du robot
cubeStartPos = [0, 0, 0.1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot = p.loadURDF(directory+"/robot.urdf",
                       cubeStartPos, cubeStartOrientation)
cubePos, cubeOrn = p.getBasePositionAndOrientation(robot)
nJoints = p.getNumJoints(robot)

# Drawing inertia boxes
if False:
    drawInertiaBox(robot, -1, [0,1,0])
    for i in range (nJoints):
        drawInertiaBox(robot, i, [0,1,0])

# Map des joints
jointsMap = []
framesMap = []
t = 0
dt = 0.01
p.setPhysicsEngineParameter(fixedTimeStep=dt)

# Collecting the available joints
for k in range (nJoints):
    jointInfo = p.getJointInfo(robot, k)
    name = jointInfo[1].decode('utf-8')
    if '_fixing' not in name:
        if '_frame' in name:
            framesMap.append([name, k])
        else:
            jointsMap.append(k)

print('* Found '+str(len(jointsMap))+' DOFs')
print('* Found '+str(len(framesMap))+' frames')

while True:
    t += dt
    
    for joint in jointsMap:
        jointInfo = p.getJointInfo(robot, joint)
        p.setJointMotorControl2(
            robot, jointInfo[0], p.POSITION_CONTROL, math.sin(t))

    print('~')
    for name, joint in framesMap:
        print('Frame '+name)
        jointState = p.getLinkState(robot, joint)
        pos = jointState[0]
        orientation = p.getEulerFromQuaternion(jointState[1])
        print(pos)
        print(orientation)
        
    p.stepSimulation()
    sleep(dt)
