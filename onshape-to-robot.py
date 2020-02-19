#!/usr/bin/env python
import numpy as np
from onshape_api.client import Client
from copy import copy
from robot import RobotURDF, RobotSDF
from colorama import Fore, Back, Style
import sys
import os
import commentjson as json
import csg

# Loading configuration
from config import config, configFile

# OnShape API client
client = Client(logging=False, creds=configFile)

if config['versionId'] is None:
    print("\n" + Style.BRIGHT + '* Retrieving workspace ID ...' + Style.RESET_ALL)
    document = client.get_document(config['documentId']).json()
    workspaceId = document['defaultWorkspace']['id']
else:
    print("\n" + Style.BRIGHT + '* Using configuration version ID '+config['versionId']+' ...' + Style.RESET_ALL)

print("\n" + Style.BRIGHT + '* Retrieving elements in the document, searching for the assembly...' + Style.RESET_ALL)
elements = client.list_elements(config['documentId']).json()
assemblyId = None
for element in elements:
    if element['type'] == 'Assembly' and (config['assemblyName'] is False or element['name'] == config['assemblyName']):
        print("- Found assembly, id: "+element['id']+', name: "'+element['name']+'"')
        assemblyId = element['id']

if assemblyId == None:
    print("! Unable to find assembly in this document")
    exit(1)

print("\n" + Style.BRIGHT + '* Retrieving assembly' + Style.RESET_ALL)
if config['versionId'] is None:
    assembly = client.get_assembly(config['documentId'], workspaceId, assemblyId)
else:
    assembly = client.get_assembly(config['documentId'], config['versionId'], assemblyId, 'v')

# Collecting parts instance from assembly and subassemblies
instances = {}
firstInstance = None
def collectParts(instancesToWalk):
    global firstInstance
    for instance in instancesToWalk:
        if firstInstance is None:
            firstInstance = instance['id']
        instances[instance['id']] = instance

root = assembly['rootAssembly']
collectParts(root['instances'])
for asm in assembly['subAssemblies']:
    collectParts(asm['instances'])

# Collecting occurences
occurrences = {}
for occurrence in root['occurrences']:
    occurrence['assignation'] = None
    occurrence['instance'] = instances[occurrence['path'][-1]]
    occurrence['transform'] = np.matrix(np.reshape(occurrence['transform'], (4, 4)))
    occurrences[tuple(occurrence['path'])] = occurrence

# Gets an occurrence given its path
def getOccurrence(path):
    return occurrences[tuple(path)]

assignations = {}
frames = {}
def assignParts(root, parent):
    assignations[root] = parent
    for occurrence in occurrences.values():
        if occurrence['path'][0] == root:
            occurrence['assignation'] = parent

print("\n" + Style.BRIGHT +'* Getting assembly features, scanning for DOFs...' + Style.RESET_ALL)
trunk = None
occurrenceLinkNames = {}
relations = {}
topLevels = set()
features = root['features']
for feature in features:
    if feature['featureType'] == 'mateConnector':
        name = feature['featureData']['name']
        if name[0:5] == 'link_':
            name = name[5:]
            occurrenceLinkNames[tuple(feature['featureData']['occurrence'])] = name
    else:
        if feature['suppressed']:
            continue

        data = feature['featureData']
        
        if len(data['matedEntities']) != 2 or \
            len(data['matedEntities'][0]['matedOccurrence']) == 0 \
            or len(data['matedEntities'][1]['matedOccurrence']) == 0:
            continue

        child = data['matedEntities'][0]['matedOccurrence'][0]
        parent = data['matedEntities'][1]['matedOccurrence'][0]

        if data['name'][0:3] == 'dof':
            parts = data['name'].split('_')
            del parts[0]
            data['inverted'] = False
            if parts[-1] == 'inv' or parts[-1] == 'inverted':
                data['inverted'] = True
                del parts[-1]
            name = '_'.join(parts)
            if name == '':
                print(Fore.RED + 
                    'ERROR: a DOF dones\'t have any name ("'+data['name']+'" should be "dof_...")' + Style.RESET_ALL)
                exit()
            print(Fore.GREEN + '+ Found DOF: '+name + Style.RESET_ALL)
            
            relations[child] = [parent, data, name]
            assignParts(child, child)
            assignParts(parent, parent)
            if child not in frames:
                frames[child] = []
            if parent not in frames:
                frames[parent] = []
      
print(Fore.GREEN + Style.BRIGHT + '* Found total '+str(len(relations))+' DOFs')

# If we have no DOF
if len(relations) == 0:
    trunk = firstInstance
    assignParts(firstInstance, firstInstance)

# Spreading parts assignations
changed = True
while changed:
    changed = False
    for feature in features:
        if feature['featureType'] != 'mate' or feature['suppressed']:
            continue

        data = feature['featureData']

        if len(data['matedEntities']) != 2 \
            or len(data['matedEntities'][0]['matedOccurrence']) == 0 \
            or len(data['matedEntities'][1]['matedOccurrence']) == 0:
            continue

        occurrenceA = data['matedEntities'][0]['matedOccurrence'][0]
        occurrenceB = data['matedEntities'][1]['matedOccurrence'][0]

        if (occurrenceA not in assignations) != (occurrenceB not in assignations):
            if data['name'][0:5] == 'frame':
                name = '_'.join(data['name'].split('_')[1:])
                if occurrenceA in assignations:
                    frames[occurrenceA].append([name, data['matedEntities'][1]['matedOccurrence']])
                    assignParts(occurrenceB, {True: assignations[occurrenceA], False: 'frame'}[config['drawFrames']])
                    changed = True
                else:
                    frames[occurrenceB].append([name, data['matedEntities'][0]['matedOccurrence']])
                    assignParts(occurrenceA, {True: assignations[occurrenceB], False: 'frame'}[config['drawFrames']])
                    changed = True
            else:
                if occurrenceA in assignations:
                    assignParts(occurrenceB, assignations[occurrenceA])
                    changed = True
                else:
                    assignParts(occurrenceA, assignations[occurrenceB])
                    changed = True

print("\n" + Style.BRIGHT + '* Building robot tree' + Style.RESET_ALL)
def collect(id):
    part = {}
    part['id'] = id
    part['children'] = []
    for childId in relations:
        entry = relations[childId]
        if entry[0] == id:
            child = collect(childId)
            child['mate'] = entry[1]
            child['relation'] = entry[2]
            part['children'].append(child)
    return part

# Searching for the trunk, assuming it is an element that has
# not children
for childId in relations:
    entry = relations[childId]
    if entry[0] not in relations:
        trunk = entry[0]
        break
trunkOccurrence = getOccurrence([trunk])
print(Style.BRIGHT + '* Trunk is '+trunkOccurrence['instance']['name'] + Style.RESET_ALL)

for occurrence in occurrences.values():
    if occurrence['assignation'] is None:
        print(Fore.YELLOW + 'WARNING: part ('+occurrence['instance']['name']+') has no assignation, connecting it with trunk' + Style.RESET_ALL)
        occurrence['assignation'] = trunk

tree = collect(trunk)

if config['outputFormat'] == 'urdf':
    robot = RobotURDF()
elif config['jointMaxEffort'] == 'sdf':
    robot = RobotSDF()
else:
    print(Fore.RED + '! ERROR Unknown output format: '+config['outputFormat']+' (supported are urdf and sdf)' + Style.RESET_ALL)
    exit()
robot.drawCollisions = config['drawCollisions']
robot.jointMaxEffort = config['jointMaxEffort']
robot.jointMaxVelocity = config['jointMaxVelocity']
robot.noDynamics = config['noDynamics']

# Adds a part to the current robot link
def addPart(occurrence, matrix):
    global config, occurrenceLinkNames
    part = occurrence['instance']

    # Importing STL file for this part
    prefix = extractPartName(part['name'], part['configuration'])

    if prefix in config['ignore']:
        return

    stlFile = prefix+'.stl'
    stl = client.part_studio_stl_m(part['documentId'], part['documentMicroversion'], part['elementId'], 
                                   part['partId'], part['configuration'])
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
    metadata = client.part_get_metadata(part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'], part['configuration'])
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
            massProperties = client.part_mass_properties(part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'], part['configuration'])
            massProperties = massProperties['bodies'][part['partId']]
            mass = massProperties['mass'][0]
            com = massProperties['centroid']
            inertia = massProperties['inertia']

        if abs(mass) < 1e-9:
            print(Fore.YELLOW + 'WARNING: part '+part['name']+' has no mass, maybe you should assign a material to it ?' + Style.RESET_ALL)

    pose = occurrence['transform']
    if robot.relative:
        pose = np.linalg.inv(matrix)*pose
    
    linkName = None
    path = tuple(occurrence['path'])
    if path in occurrenceLinkNames:
        linkName = occurrenceLinkNames[path]

    robot.addPart(pose, stlFile, mass, com, inertia, color, shapes, prefix, linkName)

partNames = {}
def extractPartName(name, configuration):
    parts = name.split(' ')
    del parts[-1]

    if configuration != 'default':
        parts += ['_' + configuration.replace('=', '_').replace(' ', '_')]

    return '_'.join(parts).lower()

def processPartName(name, configuration):
    global partNames
    name = extractPartName(name, configuration)

    if name in partNames:
        partNames[name] += 1
    else:
        partNames[name] = 1

    return name+'_'+str(partNames[name])

def buildRobot(tree, matrix, linkPart=None):
    occurrence = getOccurrence([tree['id']])
    instance = occurrence['instance']
    print(Fore.BLUE + Style.BRIGHT + '* Adding top-level instance ['+instance['name']+']' + Style.RESET_ALL)

    link = processPartName(instance['name'], instance['configuration'])

    # Collecting all children in the tree assigned to this top-level part
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

    # Calling the function with recursion for children
    k = 0
    for child in tree['children']:
        mate = child['mate']
        if mate['matedEntities'][0]['matedOccurrence'][0] == tree['id']:
            matedOccurrence = mate['matedEntities'][1]
        else:
            matedOccurrence = mate['matedEntities'][0]
        childLinkPart = getOccurrence(matedOccurrence['matedOccurrence'])

        worldAxisFrame = childLinkPart['transform']
        origin = matedOccurrence['matedCS']['origin']
        zAxis = np.array(matedOccurrence['matedCS']['zAxis'])

        if mate['inverted']:
            zAxis = -zAxis

        translation = np.matrix(np.identity(4))
        translation[0, 3] += origin[0]
        translation[1, 3] += origin[1]
        translation[2, 3] += origin[2]
        worldAxisFrame = worldAxisFrame * translation

        childMatrix = matrix
        axisFrame = worldAxisFrame
        if robot.relative:
            axisFrame = np.linalg.inv(matrix)*axisFrame
            childMatrix = worldAxisFrame

        subLink = buildRobot(child, childMatrix, '_'.join(childLinkPart['path']))
        robot.addJoint(link, subLink, axisFrame, child['relation'], zAxis)

    return link

# Start building the robot
buildRobot(tree, np.matrix(np.identity(4)))
robot.finalize()
# print(tree)

print("\n" + Style.BRIGHT + "* Writing "+robot.ext.upper()+" file" + Style.RESET_ALL)
f = open(config['outputDirectory']+'/robot.'+robot.ext, 'w')
f.write(robot.xml)
f.close()
