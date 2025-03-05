from transforms3d.quaternions import mat2quat, quat2mat
import math
import sys
import time
import numpy as np
import pybullet as p
from time import sleep
import os
import re


class Simulation:
    """
    A Bullet simulation involving Onshape to robot model
    """

    def __init__(self, robotPath, floor=True, fixed=False, transparent=False, gui=True, ignore_self_collisions=False,
                 realTime=True, panels=False, useUrdfInertia=True, dt=0.002, physicsClient = None):
        """Creates an instance of humanoid simulation

        Keyword Arguments:
            field {bool} -- enable the display of the field (default: {False})
            fixed {bool} -- makes the base of the robot floating/fixed (default: {False})
            transparent {bool} -- makes the robot transparent (default: {False})
            gui {bool} -- enables the gui visualizer, if False it will runs headless (default {True})
            realTime {bool} -- try to have simulation in real time (default {True})
            panels {bool} -- show/hide the user interaction pyBullet panels (default {False})
            useUrdfInertia {bool} -- use URDF from URDF file (default {True})
            dt {float} -- time step (default {0.002})
        """

        self.dir = os.path.dirname(os.path.abspath(__file__))
        self.gui = gui
        self.realTime = realTime
        self.t = 0
        self.start = time.time()
        self.dt = dt
        self.mass = None

        # Debug lines drawing
        self.lines = []
        self.currentLine = 0
        self.lastLinesDraw = 0
        self.lineColors = [[1, 0, 0], [0, 1, 0], [
            0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1]]

        # Instanciating bullet
        if physicsClient is None:
            if gui:
                physicsClient = p.connect(p.GUI)
            else:
                physicsClient = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)

        # Light GUI
        if not panels:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(
                p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)

        # Loading floor and/or plane ground
        if floor:
            self.floor = p.loadURDF(self.dir+'/bullet/plane.urdf')
        else:
            self.floor = None

        # Loading robot
        startPos = [0, 0, 0]
        if not fixed:
            startPos[2] = 1
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        flags = 0 if ignore_self_collisions else p.URDF_USE_SELF_COLLISION
        if useUrdfInertia:
            flags += p.URDF_USE_INERTIA_FROM_FILE
        self.robot = p.loadURDF(robotPath,
                                startPos, startOrientation,
                                flags=flags, useFixedBase=fixed)

        # Setting frictions parameters to default ones
        self.setFloorFrictions()

        # Engine parameters
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt, maxNumCmdPer1ms=0)
        # p.setRealTimeSimulation(0)
        # p.setPhysicsEngineParameter(numSubSteps=1)

        # Retrieving joints and frames
        self.joints = {}
        self.passive_joints = {}
        self.jointsInfos = {}
        self.jointsIndexes = {}
        self.frames = {}
        self.maxTorques = {}

        # Collecting the available joints
        n = 0
        for k in range(p.getNumJoints(self.robot)):
            jointInfo = p.getJointInfo(self.robot, k)
            name = jointInfo[1].decode('utf-8')
            
            if 'passive' in name:
                self.passive_joints[name] = k
            elif not name.endswith('_fixed'):
                if '_frame' in name:
                    self.frames[name] = k
                else:
                    self.jointsIndexes[name] = n
                    n += 1
                    self.joints[name] = k
                    self.jointsInfos[name] = {
                        'type': jointInfo[2]
                    }
                    if jointInfo[8] < jointInfo[9]:
                        self.jointsInfos[name]['lowerLimit'] = jointInfo[8]
                        self.jointsInfos[name]['upperLimit'] = jointInfo[9]

        # Changing robot opacity if transparent set to true
        if transparent:
            for k in range(p.getNumJoints(self.robot)):
                p.changeVisualShape(self.robot, k, rgbaColor=[
                                    0.3, 0.3, 0.3, 0.3])

        print('* Found '+str(len(self.joints))+' DOFs')
        print('* Found '+str(len(self.frames))+' frames')

    def setFloorFrictions(self, lateral=1, spinning=-1, rolling=-1):
        """Sets the frictions with the plane object

        Keyword Arguments:
            lateral {float} -- lateral friction (default: {1.0})
            spinning {float} -- spinning friction (default: {-1.0})
            rolling {float} -- rolling friction (default: {-1.0})
        """
        if self.floor is not None:
            p.changeDynamics(self.floor, -1, lateralFriction=lateral,
                             spinningFriction=spinning, rollingFriction=rolling)

    def lookAt(self, target):
        """Control the look of the visualizer camera

        Arguments:
            target {tuple} -- target as (x,y,z) tuple
        """
        if self.gui:
            params = p.getDebugVisualizerCamera()
            p.resetDebugVisualizerCamera(
                params[10], params[8], params[9], target)

    def getRobotPose(self):
        """Gets the robot (origin) position

        Returns:
            (tuple(3), tuple(3)) -- (x,y,z), (roll, pitch, yaw)
        """
        pose = p.getBasePositionAndOrientation(self.robot)
        return (pose[0], p.getEulerFromQuaternion(pose[1]))

    def frameToWorldMatrix(self, frame):
        """Gets the given frame to world matrix transformation. can be a frame name
        from URDF/SDF or "origin" for the part origin

        Arguments:
            frame {str} -- frame name

        Returns:
            np.matrix -- a 4x4 matrix
        """

        if frame == 'origin':
            frameToWorldPose = p.getBasePositionAndOrientation(self.robot)
        else:
            frameToWorldPose = p.getLinkState(self.robot, self.frames[frame])

        return self.poseToMatrix(frameToWorldPose)

    def transformation(self, frameA, frameB):
        """Transformation matrix AtoB

        Arguments:
            frameA {str} -- frame A name
            frameB {str} -- frame B name

        Returns:
            np.matrix -- A 4x4 matrix
        """
        AtoWorld = self.frameToWorldMatrix(frameA)
        BtoWorld = self.frameToWorldMatrix(frameB)

        return np.linalg.inv(BtoWorld) * AtoWorld

    def poseToMatrix(self, pose):
        """Converts a pyBullet pose to a transformation matrix"""
        translation = pose[0]
        quaternion = pose[1]

        # NOTE: PyBullet quaternions are x, y, z, w
        rotation = quat2mat([quaternion[3], quaternion[0],
                             quaternion[1], quaternion[2]])

        m = np.identity(4)
        m[0:3, 0:3] = rotation
        m.T[3, 0:3] = translation

        return np.matrix(m)

    def matrixToPose(self, matrix):
        """Converts a transformation matrix to a pyBullet pose"""
        arr = np.array(matrix)
        translation = list(arr.T[3, 0:3])
        quaternion = mat2quat(arr[0:3, 0:3])

        # NOTE: PyBullet quaternions are x, y, z, w
        quaternion = [quaternion[1], quaternion[2],
                      quaternion[3], quaternion[0]]

        return translation, quaternion

    def setRobotPose(self, pos, orn):
        """Sets the robot (origin) pose

        Arguments:
            pos {tuple} -- (x,y,z) position
            orn {tuple} -- (x,y,z,w) quaternions
        """
        p.resetBasePositionAndOrientation(self.robot, pos, orn)

    def reset(self, height=0.5, orientation='straight'):
        """Resets the robot for experiment (joints, robot position, simulator time)

        Keyword Arguments:
            height {float} -- height of the reset (m) (default: {0.55})
            orientation {str} -- orientation (straight, front or back) of the robot (default: {'straight'})
        """
        self.lines = []
        self.t = 0
        self.start = time.time()

        # Resets the robot position
        orn = [0, 0, 0]
        if orientation == 'front':
            orn = [0, math.pi/2, 0]
        elif orientation == 'back':
            orn = [0, -math.pi/2, 0]
        self.resetPose([0, 0, height], p.getQuaternionFromEuler(orn))

        # Reset the joints to 0
        for entry in self.joints.values():
            p.resetJointState(self.robot, entry, 0)

    def resetPose(self, pos, orn):
        """Called by reset() with the robot pose

        Arguments:
            pos {tuple} -- (x,y,z) position
            orn {tuple} -- (x,y,z,w) quaternions
        """
        self.setRobotPose(pos, orn)

    def getFrame(self, frame):
        """Gets the given frame

        Arguments:
            frame {str} -- frame name

        Returns:
            tuple -- (pos, orn), where pos is (x, y, z) and orn is quaternions (x, y, z, w)
        """
        jointState = p.getLinkState(self.robot, self.frames[frame])
        return (jointState[0], jointState[1])

    def getFrames(self):
        """Gets the available frames in the current robot model

        Returns:
            dict -- dict of str -> (pos, orientation)
        """
        frames = {}

        for name in self.frames.keys():
            jointState = p.getLinkState(self.robot, self.frames[name])
            pos = jointState[0]
            orientation = p.getEulerFromQuaternion(jointState[1])
            frames[name] = [pos, orientation]

        return frames
    
    def getVelocity(self, frame):
        """Gets the velocity of the given frame

        Arguments:
            frame {str} -- frame name

        Returns:
            tuple -- (linear, angular)
        """
        jointState = p.getLinkState(self.robot, self.frames[frame], computeLinkVelocity=True)
        return (jointState[6], jointState[7])

    def resetJoints(self, joints):
        """Reset all the joints to a given position

        Arguments:
            joints {dict} -- dict of joint name -> angle (float, radian)
        """
        for name in joints:
            p.resetJointState(self.robot, self.joints[name], joints[name])

    def setJoints(self, joints):
        """Set joint targets for motor control in simulation

        Arguments:
            joints {dict} -- dict of joint name -> angle (float, radian)

        Raises:
            Exception: if a joint is not found, exception is raised

        Returns:
            applied {dict} -- dict of joint states (position, velocity, reaction forces, applied torque)
        """
        applied = {}

        for name in self.passive_joints:
            p.setJointMotorControl2(self.robot, self.passive_joints[name], controlMode=p.VELOCITY_CONTROL, force=0)

        for name in joints.keys():
            if name in self.joints:
                if name.endswith('_speed'):
                    p.setJointMotorControl2(
                        self.robot, self.joints[name], p.VELOCITY_CONTROL, targetVelocity=joints[name])
                else:
                    if name in self.maxTorques:
                        maxTorque = self.maxTorques[name]
                        p.setJointMotorControl2(
                            self.robot, self.joints[name], p.POSITION_CONTROL, joints[name], force=maxTorque)
                    else:
                        p.setJointMotorControl2(
                            self.robot, self.joints[name], p.POSITION_CONTROL, joints[name])

                applied[name] = p.getJointState(self.robot, self.joints[name])
            else:
                raise Exception("Can't find joint %s" % name)

        return applied

    def getJoints(self):
        """Get all the joints names

        Returns:
            list -- list of str, with joint names
        """
        return self.joints.keys()

    def getJointsInfos(self, name):
        """Get informations about a joint

        Return:
            list -- a list with key type, lowerLimit & upperLimit (if defined)
        """

        return self.jointsInfos[name]

    def getRobotMass(self):
        """Returns the robot mass

        Returns:
            float -- the robot mass (kg)
        """
        if self.mass is None:
            k = -1
            self.mass = 0
            while True:
                if k == -1 or p.getLinkState(self.robot, k) is not None:
                    d = p.getDynamicsInfo(self.robot, k)
                    self.mass += d[0]
                else:
                    break
                k += 1

        return self.mass

    def getCenterOfMassPosition(self):
        """Returns center of mass of the robot

        Returns:
            pos -- (x, y, z) robot center of mass
        """

        k = -1
        mass = 0
        com = np.array([0., 0., 0.])
        while True:
            if k == -1:
                pos, _ = p.getBasePositionAndOrientation(self.robot)
            else:
                res = p.getLinkState(self.robot, k)
                if res is None:
                    break
                pos = res[0]

            d = p.getDynamicsInfo(self.robot, k)
            m = d[0]
            com += np.array(pos) * m
            mass += m

            k += 1

        return com / mass

    def addDebugPosition(self, position, color=None, duration=30):
        """Adds a debug position to be drawn as a line

        Arguments:
            position {tuple} -- (x,y,z) (m)

        Keyword Arguments:
            color {tuple} -- (r,g,b) (0->1) (default: {None})
            duration {float} -- line duration on screen before disapearing (default: {30})
        """
        if color is None:
            color = self.lineColors[self.currentLine % len(self.lineColors)]

        if self.currentLine >= len(self.lines):
            self.lines.append({})

        self.lines[self.currentLine]['update'] = True
        self.lines[self.currentLine]['to'] = position
        self.lines[self.currentLine]['color'] = color
        self.lines[self.currentLine]['duration'] = duration

        self.currentLine += 1

    def drawDebugLines(self):
        """Updates the drawing of debug lines"""
        self.currentLine = 0
        if time.time() - self.lastLinesDraw > 0.05:
            for line in self.lines:
                if 'from' in line:
                    if line['update'] == True:
                        p.addUserDebugLine(
                            line['from'], line['to'], line['color'], 2, line['duration'])
                        line['update'] = False
                    else:
                        del line['from']
                line['from'] = line['to']

            self.lastLinesDraw = time.time()

    def contactPoints(self):
        """Gets all contact points and forces

        Returns:
            list -- list of entries (link_name, position in m, normal force vector, force in N)
        """
        result = []
        contacts = p.getContactPoints(bodyA=self.floor, bodyB=self.robot)
        for contact in contacts:
            link_index = contact[4]
            if link_index >= 0:
                link_name = (p.getJointInfo(
                    self.robot, link_index)[12]).decode()
            else:
                link_name = 'base'
            result.append((link_name, contact[6], contact[7], contact[9]))

        return result

    def autoCollisions(self):
        """Returns the total amount of N in autocollisions (not with ground)

        Returns:
            float -- Newtons of collisions not with ground
        """
        total = 0
        for k in range(1, p.getNumJoints(self.robot)):
            contacts = p.getContactPoints(bodyA=k)
            for contact in contacts:
                if contact[2] != self.floor:
                    total += contact[9]
        return total

    def addConstraint(self, frameA, frameB, constraint = p.JOINT_POINT2POINT):
        """Adds a constraint between two given frames

        Args:
            frameA (str): frame A name
            frameB (str): frame A name
            constraint (int, optional): pyBullet joint type. Defaults to p.JOINT_POINT2POINT.

        Returns:
            int: returns from pybullet createConstraint
        """        
        infosA = p.getJointInfo(self.robot, self.frames[frameA])
        infosB = p.getJointInfo(self.robot, self.frames[frameB])

        st = p.getLinkState(self.robot, infosA[16])
        T_world_parentA = self.poseToMatrix(st[:2])
        T_world_childA = self.poseToMatrix(self.getFrame(frameA))
        T_parentA_childA = np.linalg.inv(T_world_parentA) * T_world_childA
        childApose = self.matrixToPose(T_parentA_childA)

        st = p.getLinkState(self.robot, infosB[16])
        T_world_parentB = self.poseToMatrix(st[:2])
        T_world_childB = self.poseToMatrix(self.getFrame(frameB))
        T_parentB_childB = np.linalg.inv(T_world_parentB) * T_world_childB
        childBpose = self.matrixToPose(T_parentB_childB)
        
        c =  p.createConstraint(
            self.robot,
            infosA[16],
            self.robot,
            infosB[16],
            constraint,
            [0.0, 0.0, 0.0],
            childApose[0],
            childBpose[0],
            childApose[1],
            childBpose[1],
        )

        p.changeConstraint(c, maxForce=1e3)

        return c

    def execute(self):
        """Executes the simulaiton infinitely (blocks)"""
        while True:
            self.tick()

    def tick(self):
        """Ticks one step of simulation. If realTime is True, sleeps to compensate real time"""
        self.t += self.dt
        self.drawDebugLines()

        p.stepSimulation()
        delay = self.t - (time.time() - self.start)
        if delay > 0 and self.realTime:
            time.sleep(delay)
