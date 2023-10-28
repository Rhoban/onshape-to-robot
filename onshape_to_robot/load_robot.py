from collections import defaultdict
import math
from sys import exit
import numpy as np
import uuid
from .onshape_api.client import Client
# from .config import config, configFile
from colorama import Fore, Back, Style



class OnShapeClient:
    def __init__(self, config):
        self.config = config
        
        # OnShape API client
        self.client = Client(logging=False, creds=None)
        self.client.useCollisionsConfigurations = config["useCollisionsConfigurations"]

        workspace_id = self.getWorkspaceId()
        
        # Now, finding the assembly, according to given name in configuration, or else the first possible one
        print("\n" + Style.BRIGHT +
            '* Retrieving elements in the document, searching for the assembly...' + Style.RESET_ALL)
        if config['versionId'] != '':
            elements = self.client.list_elements(
                config['documentId'], config['versionId'], 'v').json()
        else:
            elements = self.client.list_elements(config['documentId'], workspace_id).json()
            
        assembly_id = None
        assemblyName = ""
        for element in elements:
            if element['type'] == 'Assembly' and \
                    (config['assemblyName'] is False or element['name'] == config['assemblyName']):
                print(Fore.GREEN + "+ Found assembly, id: " +
                    element['id']+', name: "'+element['name']+'"' + Style.RESET_ALL)
                assemblyName = element['name']
                assembly_id = element['id']

        if assembly_id == None:
            print(Fore.RED + "ERROR: Unable to find assembly in this document" + Style.RESET_ALL)
            exit(1)

        # Retrieving the assembly
        print("\n" + Style.BRIGHT + '* Retrieving assembly "' +
            assemblyName+'" with id '+assembly_id + Style.RESET_ALL)
        if config['versionId'] != '':
            assembly = self.client.get_assembly(
                config['documentId'], config['versionId'], assembly_id, 'v', configuration=config['configuration'])
        else:
            assembly = self.client.get_assembly(
                config['documentId'], workspace_id, assembly_id, configuration=config['configuration'])

        root = assembly['rootAssembly']

        # Finds a (leaf) instance given the full path, typically A B C where A and B would be subassemblies and C
        # the final part

        # Collecting occurrences, the path is the assembly / sub assembly chain
        self.occurrences = {}
        for occurrence in root['occurrences']:
            occurrence['assignation'] = None
            occurrence['instance'] = self.findInstance(assembly, occurrence['path'])
            occurrence['transform'] = np.matrix(
                np.reshape(occurrence['transform'], (4, 4)))
            occurrence['linkName'] = None
            self.occurrences[tuple(occurrence['path'])] = occurrence

        # Gets an occurrence given its full path



        # Assignations are pieces that will be in the same link. Note that this is only for top-level
        # item of the path (all sub assemblies and parts in assemblies are naturally in the same link as
        # the parent), but other parts that can be connected with mates in top assemblies are then assigned to
        # the link
        assignations = {}

        # Frames (mated with frame_ name) will be special links in the output file allowing to track some specific
        # manually identified frames
        self.frames = defaultdict(list)


        def assignParts(root, parent):
            assignations[root] = parent
            for occurrence in self.occurrences.values():
                if occurrence['path'][0] == root:
                    occurrence['assignation'] = parent

        from .features import init as features_init, getLimits
        features_init(self.client, config, root, workspace_id, assembly_id)


        # First, features are scanned to find the DOFs. Links that they connects are then tagged
        print("\n" + Style.BRIGHT +
            '* Getting assembly features, scanning for DOFs...' + Style.RESET_ALL)
        trunk = None
        self.relations = {}
        features = root['features']
        for feature in features:
            if feature['featureType'] == 'mateConnector':
                name = feature['featureData']['name']
                if name[0:5] == 'link_':
                    name = name[5:]
                    self.occurrences[(feature['featureData']['occurrence'][0],)
                                ]['linkName'] = name
            else:
                if feature['suppressed']:
                    continue

                data = feature['featureData']

                if 'matedEntities' not in data or len(data['matedEntities']) != 2 or \
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

                    limits = None
                    if data['mateType'] == 'REVOLUTE' or data['mateType'] == 'CYLINDRICAL':
                        if 'wheel' in parts or 'continuous' in parts:
                            jointType = 'continuous'
                        else:
                            jointType = 'revolute'

                        if not config['ignoreLimits']:
                            limits = getLimits(jointType, data['name'])
                    elif data['mateType'] == 'SLIDER':
                        jointType = 'prismatic'
                        if not config['ignoreLimits']:
                            limits = getLimits(jointType, data['name'])
                    elif data['mateType'] == 'FASTENED':
                        jointType = 'fixed'
                    else:
                        print(Fore.RED + 'ERROR: "' + name +
                            '" is declared as a DOF but the mate type is '+data['mateType']+'')
                        print(
                            '       Only REVOLUTE, CYLINDRICAL, SLIDER and FASTENED are supported' + Style.RESET_ALL)
                        exit(1)

                    # We compute the axis in the world frame
                    matedEntity = data['matedEntities'][0]
                    matedTransform = self.getOccurrence(
                        matedEntity['matedOccurrence'])['transform']
                    
                    # jointToPart is the (rotation only) matrix from joint to the part
                    # it is attached to
                    jointToPart = np.eye(4)
                    jointToPart[:3, :3] = np.stack((
                        np.array(matedEntity['matedCS']['xAxis']),
                        np.array(matedEntity['matedCS']['yAxis']),
                        np.array(matedEntity['matedCS']['zAxis'])
                    )).T
                    
                    if data['inverted']:
                        if limits is not None:
                            limits = (-limits[1], -limits[0])


                        # Flipping the joint around X axis
                        flip = np.array([[1, 0, 0, 0],
                                        [0, -1, 0, 0],
                                        [0, 0, -1, 0],
                                        [0, 0,  0,  1]])
                        jointToPart = jointToPart.dot(flip)

                    zAxis = np.array([0, 0, 1])

                    origin = matedEntity['matedCS']['origin']
                    translation = np.matrix(np.identity(4))
                    translation[0, 3] += origin[0]
                    translation[1, 3] += origin[1]
                    translation[2, 3] += origin[2]
                    worldAxisFrame = matedTransform * translation

                    # Resulting frame of axis, always revolving around z
                    worldAxisFrame = worldAxisFrame.dot(jointToPart)

                    limitsStr = ''
                    if limits is not None:
                        limitsStr = '[' + str(round(limits[0], 3)) + \
                            ': ' + str(round(limits[1], 3)) + ']'
                    print(Fore.GREEN + '+ Found DOF: '+name + ' ' + Style.DIM +
                        '('+jointType+')'+limitsStr + Style.RESET_ALL)

                    if child in self.relations:
                        print(Fore.RED)
                        print('Error, the relation '+name +
                            ' is connected a child that is already connected')
                        print('Be sure you ordered properly your relations, see:')
                        print(
                            'https://onshape-to-robot.readthedocs.io/en/latest/design.html#specifying-degrees-of-freedom')
                        print(Style.RESET_ALL)
                        exit()

                    self.relations[child] = {
                        'parent': parent,
                        'worldAxisFrame': worldAxisFrame,
                        'zAxis': zAxis,
                        'name': name,
                        'type': jointType,
                        'limits': limits
                    }

                    assignParts(child, child)
                    assignParts(parent, parent)

        print(Fore.GREEN + Style.BRIGHT + '* Found total ' +
            str(len(self.relations))+' DOFs' + Style.RESET_ALL)

        # If we have no DOF
        if len(self.relations) == 0:
            trunk = root['instances'][0]['id']
            assignParts(trunk, trunk)


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
                            self.frames[occurrenceA].append(
                                [name, data['matedEntities'][1]['matedOccurrence']])
                            assignParts(occurrenceB, {True: assignations[occurrenceA], False: 'frame'}[
                                        config['drawFrames']])
                            changed = True
                        else:
                            self.frames[occurrenceB].append(
                                [name, data['matedEntities'][0]['matedOccurrence']])
                            assignParts(occurrenceA, {True: assignations[occurrenceB], False: 'frame'}[
                                        config['drawFrames']])
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

        for childId in self.relations:
            entry = self.relations[childId]
            if entry['parent'] not in self.relations:
                trunk = entry['parent']
                break
        trunkOccurrence = self.getOccurrence([trunk])
        print(Style.BRIGHT + '* Trunk is ' +
            trunkOccurrence['instance']['name'] + Style.RESET_ALL)

        for occurrence in self.occurrences.values():
            if occurrence['assignation'] is None:
                print(Fore.YELLOW + 'WARNING: part (' +
                    occurrence['instance']['name']+') has no assignation, connecting it with trunk' + Style.RESET_ALL)
                child = occurrence['path'][0]
                connectParts(child, trunk)

        # If a sub-assembly is suppressed, we also mark as suppressed the parts in this sub-assembly
        for occurrence in self.occurrences.values():
            if not occurrence['instance']['suppressed']:
                for k in range(len(occurrence['path'])-1):
                    upper_path = tuple(occurrence['path'][0:k+1])
                    if upper_path in self.occurrences and self.occurrences[upper_path]['instance']['suppressed']:
                        occurrence['instance']['suppressed'] = True

        def collect(id):
            part = {}
            part['id'] = id
            part['children'] = []
            for childId in self.relations:
                entry = self.relations[childId]
                if entry['parent'] == id:
                    child = collect(childId)
                    child['axis_frame'] = entry['worldAxisFrame']
                    child['z_axis'] = entry['zAxis']
                    child['dof_name'] = entry['name']
                    child['jointType'] = entry['type']
                    child['jointLimits'] = entry['limits']
                    part['children'].append(child)
            return part

        self.tree = collect(trunk)
    
    def getParent(self, part_id):
        part = self.relations.get(part_id)
        if not part:
            return None
        return part["parent"]

        #return self.client, tree, occurrences, getOccurrence, frames

    def findInstance(self, assembly, path, instances=None):
        if instances is None:
            instances = assembly['rootAssembly']['instances']

        for instance in instances:
            if instance['id'] == path[0]:
                if len(path) == 1:
                    # If the length of remaining path is 1, the part is in the current assembly/subassembly
                    return instance
                else:
                    # Else, we need to find the matching sub assembly to find the proper part (recursively)
                    d = instance['documentId']
                    m = instance['documentMicroversion']
                    e = instance['elementId']
                    for asm in assembly['subAssemblies']:
                        if asm['documentId'] == d and asm['documentMicroversion'] == m and asm['elementId'] == e:
                            return self.findInstance(assembly, path[1:], asm['instances'])

        print(Fore.RED + 'Could not find instance for ' + str(path) + Style.RESET_ALL)


    def getWorkspaceId(self):
        # If a versionId is provided, it will be used, else the main workspace is retrieved
        if self.config["versionId"]:
            print("\n" + Style.BRIGHT + "* Using configuration version ID " +
                self.config["versionId"]+' ...' + Style.RESET_ALL)
            return None
        
        if self.config["workspaceId"]:
            print("\n" + Style.BRIGHT + "* Using configuration workspace ID " +
                self.config["workspaceId"]+' ...' + Style.RESET_ALL)
            return self.config["workspaceId"]
        
        print("\n" + Style.BRIGHT + "* Retrieving workspace ID ..." + Style.RESET_ALL)
        document = self.client.get_document(self.config["documentId"]).json()
        workspace_id = document["defaultWorkspace"]["id"]
        print(Fore.GREEN + "+ Using workspace id: " + workspace_id + Style.RESET_ALL)
        return workspace_id


    def getOccurrence(self, path):
        return self.occurrences[tuple(path)]
    
    def fetchSTL(self, part, shortend_configuration):
        return self.client.part_studio_stl_m(
            part["documentId"], part["documentMicroversion"], part["elementId"], part["partId"], 
            shortend_configuration)