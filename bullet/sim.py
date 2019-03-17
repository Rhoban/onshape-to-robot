import math
import pybullet as p
from time import sleep
from control import jointsPosition

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
planeId = p.loadURDF('plane.urdf')

# Chargement du robot
cubeStartPos = [0, 0, 0.1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot = p.loadURDF("../urdf/robot.urdf",
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
t = 0
dt = 0.01
p.setPhysicsEngineParameter(fixedTimeStep=dt)

# Collecting the available joints
for k in range (nJoints):
    jointInfo = p.getJointInfo(robot, k)
    if 'fixing' not in jointInfo[1].decode('utf-8'):
        jointsMap.append(k)

while True:
    t += dt
    joints = jointsPosition(t)
    for k in range(len(joints)):
        jointInfo = p.getJointInfo(robot, jointsMap[k])
        p.setJointMotorControl2(
            robot, jointInfo[0], p.POSITION_CONTROL, joints[k])
    p.stepSimulation()
    sleep(dt)
