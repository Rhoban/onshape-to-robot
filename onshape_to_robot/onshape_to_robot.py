#!/usr/bin/env python
import numpy as np
from copy import copy
from colorama import Fore, Back, Style
import sys
from sys import exit
import os
import hashlib
from . import csg
from .robot_description import RobotURDF, RobotSDF

# Loading configuration, collecting occurrences and building robot tree
from .load_robot import \
    config, client, tree, occurrences, getOccurrence, frames

# Creating robot for output
if config['outputFormat'] == 'urdf':
    robot = RobotURDF(config['robotName'])
elif config['outputFormat'] == 'sdf':
    robot = RobotSDF(config['robotName'])
else:
    print(Fore.RED + 'ERROR: Unknown output format: '+config['outputFormat']+' (supported are urdf and sdf)' + Style.RESET_ALL)
    exit()
robot.drawCollisions = config['drawCollisions']
robot.jointMaxEffort = config['jointMaxEffort']
robot.mergeSTLs = config['mergeSTLs']
robot.maxSTLSize = config['maxSTLSize']
robot.simplifySTLs = config['simplifySTLs']
robot.jointMaxVelocity = config['jointMaxVelocity']
robot.noDynamics = config['noDynamics']
robot.packageName = config['packageName']
robot.addDummyBaseLink = config['addDummyBaseLink']
robot.robotName = config['robotName']
robot.additionalXML = config['additionalXML']


# Adds a part to the current robot link
def addPart(occurrence, matrix):
    global config, occurrenceLinkNames
    part = occurrence['instance']

    # Importing STL file for this part
    prefix = extractPartName(part['name'], part['configuration'])

    if prefix in config['ignore']:
        return

    stlFile = prefix+'.stl'
    # shorten the configuration to a maximum number of chars to prevent errors. Necessary for standard parts like screws
    if len(part['configuration']) > 40:
        shortend_configuration = hashlib.md5(part['configuration'].encode('utf-8')).hexdigest()
    else:
        shortend_configuration = part['configuration']
    stl = client.part_studio_stl_m(part['documentId'], part['documentMicroversion'], part['elementId'], 
                                   part['partId'], shortend_configuration)
    f = open(config['outputDirectory']+'/'+stlFile, 'wb')
    f.write(stl)
    f.close()

    # Import the SCAD files pure shapes
    shapes = None
    if config['useScads']:
        scadFile = prefix+'.scad'
        if os.path.exists(config['outputDirectory']+'/'+scadFile):
            shapes = csg.process(config['outputDirectory']+'/'+scadFile)
        
    # Obtain metadatas about part to retrieve color
    metadata = client.part_get_metadata(part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'], shortend_configuration)
    if 'appearance' in metadata:
        colors = metadata['appearance']['color']
        color = np.array([colors['red'], colors['green'], colors['blue']])/255.0
    else:
        color = [0.5, 0.5, 0.5]

    # Obtain mass properties about that part
    if config['noDynamics']:
        mass = 0
        com = [0]*3
        inertia = [0]*12
    else:
        if prefix in config['dynamicsOverride']:
            entry = config['dynamicsOverride'][prefix]
            mass = entry['mass']
            com = entry['com']
            inertia = entry['inertia']
        else:
            massProperties = client.part_mass_properties(part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'], shortend_configuration)
            massProperties = massProperties['bodies'][part['partId']]
            mass = massProperties['mass'][0]
            com = massProperties['centroid']
            inertia = massProperties['inertia']

            if abs(mass) < 1e-9:
                print(Fore.YELLOW + 'WARNING: part '+part['name']+' has no mass, maybe you should assign a material to it ?' + Style.RESET_ALL)

    pose = occurrence['transform']
    if robot.relative:
        pose = np.linalg.inv(matrix)*pose
    
    robot.addPart(pose, config['outputDirectory']+'/'+stlFile, mass, com, inertia, color, shapes, prefix)

partNames = {}
def extractPartName(name, configuration):
    parts = name.split(' ')
    del parts[-1]

    # only add configuration to name if its not default and not a very long configuration (which happens for library parts like screws)
    if configuration != 'default' and len(configuration) < 40:
        parts += ['_' + configuration.replace('=', '_').replace(' ', '_')]

    return '_'.join(parts).lower()

def processPartName(name, configuration, overrideName=None):
    if overrideName is None:
        global partNames
        name = extractPartName(name, configuration)

        if name in partNames:
            partNames[name] += 1
        else:
            partNames[name] = 1

        return name+'_'+str(partNames[name])
    else:
        return overrideName

def buildRobot(tree, matrix):
    occurrence = getOccurrence([tree['id']])
    instance = occurrence['instance']
    print(Fore.BLUE + Style.BRIGHT + '* Adding top-level instance ['+instance['name']+']' + Style.RESET_ALL)

    # Build a part name that is unique but still informative
    link = processPartName(instance['name'], instance['configuration'], occurrence['linkName'])

    # Create the link, collecting all children in the tree assigned to this top-level part
    robot.startLink(link, matrix)
    for occurrence in occurrences.values():
        if occurrence['assignation'] == tree['id'] and occurrence['instance']['type'] == 'Part':
            name = '_'.join(occurrence['path'])
            extra = ''
            if occurrence['instance']['configuration'] != 'default':
                extra = Style.DIM + ' (configuration: '+occurrence['instance']['configuration']+')'
            print(Fore.GREEN + '+ Adding part '+occurrence['instance']['name']+extra + Style.RESET_ALL)
            addPart(occurrence, matrix)
    robot.endLink()

    # Adding the frames (linkage is relative to parent)
    if tree['id'] in frames:
        for name, part in frames[tree['id']]:
            frame = getOccurrence(part)['transform']
            if robot.relative:
                frame = np.linalg.inv(matrix)*frame
            robot.addFrame(name, frame)

    # Following the children in the tree, calling this function recursively
    k = 0
    for child in tree['children']:
        worldAxisFrame = child['axis_frame']
        zAxis = child['z_axis']
        jointType = child['jointType']
        jointLimits = child['jointLimits']

        if robot.relative:
            axisFrame = np.linalg.inv(matrix)*worldAxisFrame
            childMatrix = worldAxisFrame
        else:
            # In SDF format, everything is expressed in the world frame, in this case
            # childMatrix will be always identity
            axisFrame = worldAxisFrame
            childMatrix = matrix

        subLink = buildRobot(child, childMatrix)
        robot.addJoint(jointType, link, subLink, axisFrame, child['dof_name'], jointLimits, zAxis)

    return link

# Start building the robot
buildRobot(tree, np.matrix(np.identity(4)))
robot.finalize()
# print(tree)

print("\n" + Style.BRIGHT + "* Writing "+robot.ext.upper()+" file" + Style.RESET_ALL)
f = open(config['outputDirectory']+'/robot.'+robot.ext, 'w')
f.write(robot.xml)
f.close()
