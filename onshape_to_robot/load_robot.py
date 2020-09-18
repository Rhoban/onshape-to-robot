import math
from sys import exit
import numpy as np
import uuid
from .onshape_api.client import Client
from .config import config, configFile
from colorama import Fore, Back, Style

# OnShape API client
client = Client(logging=False, creds=configFile)
client.useCollisionsConfigurations = config['useCollisionsConfigurations']

# If a versionId is provided, it will be used, else the main workspace is retrieved
if config['versionId'] == '':
    print("\n" + Style.BRIGHT + '* Retrieving workspace ID ...' + Style.RESET_ALL)
    document = client.get_document(config['documentId']).json()
    workspaceId = document['defaultWorkspace']['id']
    print(Fore.GREEN + "+ Using workspace id: " + workspaceId + Style.RESET_ALL)
else:
    print("\n" + Style.BRIGHT + '* Using configuration version ID '+config['versionId']+' ...' + Style.RESET_ALL)

# Now, finding the assembly, according to given name in configuration, or else the first possible one
print("\n" + Style.BRIGHT + '* Retrieving elements in the document, searching for the assembly...' + Style.RESET_ALL)
if config['versionId'] == '':
    elements = client.list_elements(config['documentId'], workspaceId).json()
else:
    elements = client.list_elements(config['documentId'], config['versionId'], 'v').json()
assemblyId = None
assemblyName = ''
for element in elements:
    if element['type'] == 'Assembly' and \
        (config['assemblyName'] is False or element['name'] == config['assemblyName']):
        print(Fore.GREEN + "+ Found assembly, id: "+element['id']+', name: "'+element['name']+'"' + Style.RESET_ALL)
        assemblyName = element['name']
        assemblyId = element['id']

if assemblyId == None:
    print(Fore.RED + "ERROR: Unable to find assembly in this document" + Style.RESET_ALL)
    exit(1)

# Retrieving the assembly
print("\n" + Style.BRIGHT + '* Retrieving assembly "'+assemblyName+'" with id '+assemblyId+ Style.RESET_ALL)
if config['versionId'] == '':
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

        if instance['type'] == 'Part' and instance['id'] in instances:
            i1 = instance
            i2 = instances[instance['id']]
            entries = ['documentId', 'documentVersion', 'documentMicroversion', 'elementId', 'partId']
            for entry in entries:
                if i1[entry] != i2[entry]:
                    print(Fore.YELLOW + "BE CAREFUL: Same id is used for multiple instances:" + Style.RESET_ALL)
                    print(Fore.YELLOW + "- " + instance['name'] + Style.RESET_ALL)
                    print(Fore.YELLOW + "- " + instances[instance['id']]['name'] + Style.RESET_ALL)
                    break

        instances[instance['id']] = instance

root = assembly['rootAssembly']
collectParts(root['instances'])
for asm in assembly['subAssemblies']:
    collectParts(asm['instances'])

# Collecting occurrences, the path is the assembly / sub assembly chain
occurrences = {}
for occurrence in root['occurrences']:
    occurrence['assignation'] = None
    occurrence['instance'] = instances[occurrence['path'][-1]]
    occurrence['transform'] = np.matrix(np.reshape(occurrence['transform'], (4, 4)))
    occurrence['linkName'] = None
    occurrences[tuple(occurrence['path'])] = occurrence

# Gets an occurrence given its full path
def getOccurrence(path):
    return occurrences[tuple(path)]

# Assignations are pieces that will be in the same link. Note that this is only for top-level
# item of the path (all sub assemblies and parts in assemblies are naturally in the same link as
# the parent), but other parts that can be connected with mates in top assemblies are then assigned to
# the link
assignations = {}

# Frames (mated with frame_ name) will be special links in the output file allowing to track some specific
# manually identified frames
frames = {}

def assignParts(root, parent):
    assignations[root] = parent
    for occurrence in occurrences.values():
        if occurrence['path'][0] == root:
            occurrence['assignation'] = parent

# Load joint features to get limits later
if config['versionId'] == '':
    joint_features = client.get_features(config['documentId'], workspaceId, assemblyId)
else:
    joint_features = client.get_features(config['documentId'], config['versionId'], assemblyId, type='v')

# Gets the limits of a given joint
def getLimits(jointType, name):
    enabled = False
    minimum, maximum = 0, 0
    for feature in joint_features['features']:
        # find coresponding joint
        if name == feature['message']['name']:
            # find min and max values
            for parameter in feature['message']['parameters']:
                if parameter['message']['parameterId'] == "limitsEnabled":
                    enabled = parameter['message']['value']

                if jointType == 'revolute':
                    # Note: we here assume it's in deg
                    if parameter['message']['parameterId'] == 'limitAxialZMin':
                        minimum = math.radians(float(parameter['message']['expression'][:-4]))
                    if parameter['message']['parameterId'] == 'limitAxialZMax':
                        maximum = math.radians(float(parameter['message']['expression'][:-4]))
                elif jointType == 'prismatic':
                    # Note: we here assume it's in mm
                    if parameter['message']['parameterId'] == 'limitZMin':
                        minimum = float(parameter['message']['expression'][:-3])/1000.0
                    if parameter['message']['parameterId'] == 'limitZMax':
                        maximum = float(parameter['message']['expression'][:-3])/1000.0
    if enabled:
        return (minimum, maximum)
    else:
        print(Fore.YELLOW + 'WARNING: joint ' + name + ' of type ' + jointType + ' has no limits ' + Style.RESET_ALL)
        return None

# First, features are scanned to find the DOFs. Links that they connects are then tagged 
print("\n" + Style.BRIGHT +'* Getting assembly features, scanning for DOFs...' + Style.RESET_ALL)
trunk = None
relations = {}
features = root['features']
for feature in features:
    if feature['featureType'] == 'mateConnector':
        name = feature['featureData']['name']
        if name[0:5] == 'link_':
            name = name[5:]
            occurrences[(feature['featureData']['occurrence'][0],)]['linkName'] = name
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
            
            if data['mateType'] == 'REVOLUTE' or data['mateType'] == 'CYLINDRICAL':
                jointType = 'revolute'
                limits = getLimits(jointType, data['name'])
            elif data['mateType'] == 'SLIDER':
                jointType = 'prismatic'
                limits = getLimits(jointType, data['name'])
            elif data['mateType'] == 'FASTENED':
                jointType = 'floating'
                limits = None
            else:
                print(Fore.RED +'ERROR: "'+ name+'" is declared as a DOF but the mate type is '+data['mateType']+'')
                print('       Only REVOLUTE, CYLINDRICAL, SLIDER and FASTENED are supported'  +Style.RESET_ALL)
                exit(1)

            limitsStr = ''
            if limits is not None:
                limitsStr = '[' +str(round(limits[0], 3)) + ': ' + str(round(limits[1], 3)) + ']'
            print(Fore.GREEN + '+ Found DOF: '+name + ' ' + Style.DIM + '('+jointType+')'+limitsStr+ Style.RESET_ALL)

            # We compute the axis in the world frame
            matedEntity = data['matedEntities'][0]
            matedTransform = getOccurrence(matedEntity['matedOccurrence'])['transform']
            zAxis = np.array(matedEntity['matedCS']['zAxis'])
            if data['inverted']:
                zAxis = -zAxis
            origin = matedEntity['matedCS']['origin']

            translation = np.matrix(np.identity(4))
            translation[0, 3] += origin[0]
            translation[1, 3] += origin[1]
            translation[2, 3] += origin[2]
            worldAxisFrame = matedTransform * translation

            relations[child] = {
                'parent': parent, 
                'worldAxisFrame': worldAxisFrame,
                'zAxis': zAxis,
                'name': name,
                'type': jointType,
                'limits': limits
            }

            assignParts(child, child)
            assignParts(parent, parent)
            if child not in frames:
                frames[child] = []
            if parent not in frames:
                frames[parent] = []
      
print(Fore.GREEN + Style.BRIGHT + '* Found total '+str(len(relations))+' DOFs' + Style.RESET_ALL)

# If we have no DOF
if len(relations) == 0:
    trunk = firstInstance
    assignParts(firstInstance, firstInstance)

def connectParts(child, parent):
    assignParts(child, parent)

# Spreading parts assignations, this parts mainly does two things:
# 1. Finds the parts of the top level assembly that are not directly in a sub assembly and try to assign them
#    to an existing link that was identified before
# 2. Among those parts, finds the ones that are frames (connected with a frame_* connector)
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
                    connectParts(occurrenceB, assignations[occurrenceA])
                    changed = True
                else:
                    connectParts(occurrenceA, assignations[occurrenceB])
                    changed = True

# Building and checking robot tree, here we:
# 1. Search for robot trunk (which will be the top-level link)
# 2. Scan for orphaned parts (if you add something floating with no mate to anything)
#    that are then assigned to trunk by default
# 3. Collect all the pieces of the robot tree
print("\n" + Style.BRIGHT + '* Building robot tree' + Style.RESET_ALL)

for childId in relations:
    entry = relations[childId]
    if entry['parent'] not in relations:
        trunk = entry['parent']
        break
trunkOccurrence = getOccurrence([trunk])
print(Style.BRIGHT + '* Trunk is '+trunkOccurrence['instance']['name'] + Style.RESET_ALL)

for occurrence in occurrences.values():
    if occurrence['assignation'] is None:
        print(Fore.YELLOW + 'WARNING: part ('+occurrence['instance']['name']+') has no assignation, connecting it with trunk' + Style.RESET_ALL)
        child = occurrence['path'][0]
        connectParts(child, trunk)

def collect(id):
    part = {}
    part['id'] = id
    part['children'] = []
    for childId in relations:
        entry = relations[childId]
        if entry['parent'] == id:
            child = collect(childId)
            child['axis_frame'] = entry['worldAxisFrame']
            child['z_axis'] = entry['zAxis']
            child['dof_name'] = entry['name']
            child['jointType'] = entry['type']
            child['jointLimits'] = entry['limits']
            part['children'].append(child)
    return part

tree = collect(trunk)
