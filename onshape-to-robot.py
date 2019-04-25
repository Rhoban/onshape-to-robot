#!/usr/bin/env python
import numpy as np
from onshape_api.client import Client
from copy import copy
from robot import RobotURDF, RobotSDF
import sys
import os
import json
import csg

# Loading configuration
robot = 'robots/demo/'
if len(sys.argv) > 1:
    robot = sys.argv[1]

configFile = robot+'/config.json'
client = Client(logging=False, creds=configFile)
config = json.load(open(configFile))

def configGet(name, default=None):
    global config
    if name in config:
        return config[name]
    else:
        if default is None:
            print('! ERROR missing key "'+name+'" in config')
            exit()
        else:
            return default

documentId = configGet('documentId')
drawFrames = configGet('drawFrames')
drawCollisions = configGet('drawCollisions', False)
useScads = configGet('useScads', True)
assemblyName = configGet('assemblyName', False)
outputFormat = configGet('outputFormat', 'urdf')
jointMaxEffort = configGet('jointMaxEffort', 1)
jointMaxVelocity = configGet('jointMaxVelocity', 20)
noDynamics = configGet('noDynamics', False)
ignore = configGet('ignore', [])
outputDirectory = robot
tmp = configGet('dynamics', {})
dynamicsOverride = {}
for key in tmp:
    dynamicsOverride[key.lower()] = tmp[key]

try:
    os.makedirs(outputDirectory)
except OSError:
    pass

print('* Retrieving workspace ID ...')
document = client.get_document(documentId).json()
workspaceId = document['defaultWorkspace']['id']
print('- Workspace id: '+workspaceId)

print('* Retrieving elements in the document, searching for the assembly...')
elements = client.list_elements(documentId).json()
assemblyId = None
for element in elements:
    if element['type'] == 'Assembly' and (assemblyName is False or element['name'].lower() == assemblyName):
        print("- Found assembly, id: "+element['id']+', name: "'+element['name']+'"')
        assemblyId = element['id']

if assemblyId == None:
    print("! Unable to find assembly in this document")
    exit(1)

print('* Retrieving assembly')
assembly = client.get_assembly(documentId, workspaceId, assemblyId)

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

print('* Getting assembly features, scanning for DOFs...')
trunk = None
relations = {}
topLevels = set()
features = root['features']
for feature in features:
    data = feature['featureData']

    if len(data['matedEntities'][0]['matedOccurrence']) == 0 \
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
            print('! Error: a DOF dones\'t have any name ("'+data['name']+'" should be "dof_...")')
            exit()
        print('Found dof: '+name)
        
        relations[child] = [parent, data, name]
        assignParts(child, child)
        assignParts(parent, parent)
        if child not in frames:
            frames[child] = []
        if parent not in frames:
            frames[parent] = []
                
print('- Found '+str(len(relations))+' DOFs')

# If we have no DOF
if len(relations) == 0:
    trunk = firstInstance
    assignParts(firstInstance, firstInstance)

# Spreading parts assignations
changed = True
while changed:
    changed = False
    for feature in features:
        data = feature['featureData']

        if len(data['matedEntities'][0]['matedOccurrence']) == 0 \
            or len(data['matedEntities'][1]['matedOccurrence']) == 0:
            continue

        occurrenceA = data['matedEntities'][0]['matedOccurrence'][0]
        occurrenceB = data['matedEntities'][1]['matedOccurrence'][0]

        if (occurrenceA not in assignations) or (occurrenceB not in assignations):
            if data['name'][0:5] == 'frame':
                name = '_'.join(data['name'].split('_')[1:])
                if occurrenceA in assignations:
                    frames[occurrenceA].append([name, data['matedEntities'][1]['matedOccurrence']])
                    assignParts(occurrenceB, {True: assignations[occurrenceA], False: 'frame'}[drawFrames])
                    changed = True
                if occurrenceB in assignations:
                    frames[occurrenceB].append([name, data['matedEntities'][0]['matedOccurrence']])
                    assignParts(occurrenceA, {True: assignations[occurrenceB], False: 'frame'}[drawFrames])
                    changed = True
            else:
                if occurrenceA in assignations:
                    assignParts(occurrenceB, assignations[occurrenceA])
                    changed = True
                if occurrenceB in assignations:
                    assignParts(occurrenceA, assignations[occurrenceB])
                    changed = True
    
print('* Building robot tree')
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
print('Trunk is '+trunkOccurrence['instance']['name'])

for occurrence in occurrences.values():
    if occurrence['assignation'] is None:
        print('! WARNING, part ('+occurrence['instance']['name']+') has no assignation, connecting it with trunk')
        occurrence['assignation'] = trunk

tree = collect(trunk)

if outputFormat == 'urdf':
    robot = RobotURDF()
elif outputFormat == 'sdf':
    robot = RobotSDF()
else:
    print('! ERROR Unknown output format: '+outputFormat+' (supported are urdf and sdf')
    exit()
robot.drawCollisions = drawCollisions
robot.jointMaxEffort = jointMaxEffort
robot.jointMaxVelocity = jointMaxVelocity
robot.noDynamics = noDynamics

# Adds a part to the current robot link
def addPart(occurrence, matrix):
    global noDynamics, dynamicsOverride, ignore
    part = occurrence['instance']

    # Importing STL file for this part
    prefix = extractPartName(part['name'])

    if prefix in ignore:
        return

    stlFile = prefix+'.stl'
    stl = client.part_studio_stl_m(part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'])
    f = open(outputDirectory+'/'+stlFile, 'wb')
    f.write(stl)
    f.close()

    # Import the SCAD files pure shapes
    shapes = None
    if useScads:
        scadFile = prefix+'.scad'
        if os.path.exists(outputDirectory+'/'+scadFile):
            shapes = csg.process(outputDirectory+'/'+scadFile)
        
    # Obtain metadatas about part to retrieve color
    metadata = client.part_get_metadata(part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'])
    if 'appearance' in metadata:
        colors = metadata['appearance']['color']
        color = np.array([colors['red'], colors['green'], colors['blue']])/255.0
    else:
        color = [0.5, 0.5, 0.5]

    # Obtain mass properties about that part
    if noDynamics:
        mass = 0
        com = [0]*3
        inertia = [0]*12
    else:
        if prefix in dynamicsOverride:
            entry = dynamicsOverride[prefix]
            mass = entry['mass']
            com = entry['com']
            inertia = entry['inertia']
        else:
            massProperties = client.part_mass_properties(part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'])
            massProperties = massProperties['bodies'][part['partId']]
            mass = massProperties['mass'][0]
            com = massProperties['centroid']
            inertia = massProperties['inertia']

        if abs(mass) < 1e-9:
            print('! WARNING Part '+part['name']+' has no mass, maybe you should assign a material to it ?')

    pose = occurrence['transform']
    if robot.relative:
        pose = np.linalg.inv(matrix)*pose
    robot.addPart(pose, stlFile, mass, com, inertia, color, shapes, prefix)

partNames = {}
def extractPartName(name):
    parts = name.split(' ')
    del parts[-1]
    return '_'.join(parts).lower()

def processPartName(name):
    global partNames
    name = extractPartName(name)

    if name in partNames:
        partNames[name] += 1
    else:
        partNames[name] = 1

    return name+'_'+str(partNames[name])

def buildRobot(tree, matrix, linkPart=None):
    occurrence = getOccurrence([tree['id']])
    instance = occurrence['instance']
    print('~ Adding top-level instance ['+instance['name']+']')

    link = processPartName(instance['name'])

    # Collecting all children in the tree assigned to this top-level part
    robot.startLink(link, matrix)
    for occurrence in occurrences.values():
        if occurrence['assignation'] == tree['id'] and occurrence['instance']['type'] == 'Part':
            name = '_'.join(occurrence['path'])
            print('- Adding part '+occurrence['instance']['name'])
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

print("* Writing "+robot.ext+" file")
f = open(outputDirectory+'/robot.'+robot.ext, 'w')
f.write(robot.xml)
f.close()
