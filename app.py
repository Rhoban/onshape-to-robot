import numpy as np
from apikey.client import Client
from copy import copy
from robot import Robot
import os

# You should fil creds.json with API key for your acount obtained from
# https://dev-portal.onshape.com/keys

# The first part in assembly should be the trunk
# The sub-parts should revolute around Z-axis
# DOFs should be named "dof..."
# Assembly should be named "robot"

client = Client(logging=False)

# Document that should be parsed
# XXX: Argv-ize
documentId = '483c803918afc4d52e2647f0'

print('* Retrieving workspace ID ...')
document = client.get_document(documentId).json()
workspaceId = document['defaultWorkspace']['id']
print('- Workspace id: '+workspaceId)

print('* Retrieving elements in the document, searching for the assembly...')
elements = client.list_elements(documentId).json()
assemblyId = None
for element in elements:
    if element['type'] == 'Assembly' and element['name'].lower() == 'robot':
        print("- Found assembly, id: "+element['id']+', name: "'+element['name']+'"')
        assemblyId = element['id']

if assemblyId == None:
    print("! Unable to find assembly in this document")
    exit(1)

print('* Retrieving assembly')
assembly = client.get_assembly(documentId, workspaceId, assemblyId).json()

# Collecting parts instance from assembly and subassemblies
instances = {}
def collectParts(instancesToWalk):
    for instance in instancesToWalk:
        instances[instance['id']] = instance

root = assembly['rootAssembly']
collectParts(root['instances'])
for asm in assembly['subAssemblies']:
    collectParts(asm['instances'])

# Collecting occurences
occurrences = []
for occurrence in root['occurrences']:
    occurrence['instance'] = instances[occurrence['path'][-1]]
    occurrence['transform'] = np.matrix(np.reshape(occurrence['transform'], (4, 4)))
    occurrences.append(occurrence)

# XXX: Instead of doing that, we can get the mate features in assembly using
# ?includeMateFeatures=true when calling the get_assembly function, it's clearer...
print('* Getting assembly features, scanning for DOFs...')
relations = []
features = root['features']
for feature in features:
    data = feature['featureData']
    if data['name'][0:3] == 'dof':
        a = data['matedEntities'][0]['matedOccurrence'][0]
        b = data['matedEntities'][1]['matedOccurrence'][0]
        relations.append([a,b])
print('- Found '+str(len(relations))+' DOFs')
    
print('* Building robot tree')
def collect(id, parent = None):
    part = {}
    part['id'] = id
    part['children'] = []
    for entry in relations:
        if entry[0] == id and entry[1] != parent:
            part['children'].append(collect(entry[1], id))
        if entry[1] == id and entry[0] != parent:
            part['children'].append(collect(entry[0], id))
    return part

trunk = root['instances'][0]['id']
tree = collect(trunk)

robot = Robot()

def addPart(link, occurrence, matrix):
    part = occurrence['instance']
    # Importing STL file for this part
    stlFile = '%s_%s_%s_%s.stl' % (part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'])
    if not os.path.exists('urdf/'+stlFile):
        stl = client.part_studio_stl_m(part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'])
        f = open('urdf/'+stlFile, 'wb')
        f.write(stl.content)
        f.close()
        
    massProperties = client.part_mas_properties(part['documentId'], part['documentMicroversion'], part['elementId'], part['partId']).json()
    massProperties = massProperties['bodies'][part['partId']]
    mass = massProperties['mass'][0]
    com = massProperties['centroid']
    inertia = massProperties['inertia']
    robot.addPart(link, '_'.join(occurrence['path']), np.linalg.inv(matrix)*occurrence['transform'], stlFile, mass, com, inertia)

def getOccurrence(path):
    for occurrence in occurrences:
        if occurrence['path'] == path:
            return occurrence

def buildRobot(tree, matrix):
    print('~~~ Adding instance')
    occurrence = getOccurrence([tree['id']])
    instance = occurrence['instance']
    print('> '+instance['name'])
    link = robot.addLink(instance['name'])

    # Matrix pass from the world frame to the current instance
    matrix = matrix*occurrence['transform']

    if instance['type'] == 'part':
        print('Error: instance '+instance['name']+' is not an assembly')
        exit()
    else:
        # The instance is probably an assembly, gathering everything that
        # begins with the same path
        for occurrence in occurrences:
            if occurrence['path'][0] == tree['id'] and occurrence['instance']['type'] == 'Part':
                addPart(link, occurrence, matrix)

    for child in tree['children']:
        childOccurrence = getOccurrence([child['id']])
        print(child['id'])
        print(childOccurrence['transform'])
        subLink = buildRobot(child, matrix)
        #  XXX/ Not correct
        robot.addJoint(link, subLink, childOccurrence['transform'])

    return link

# Start building the robot
buildRobot(tree, np.matrix(np.identity(4)))
robot.finalize()
print(tree)

print("* Writing URDF file")
urdf = file('urdf/robot.urdf', 'w')
urdf.write(robot.urdf)
urdf.close()
