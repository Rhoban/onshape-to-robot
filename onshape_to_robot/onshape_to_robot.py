import os
import numpy as np
from copy import copy
import commentjson as json
import colorama
from colorama import Fore, Back, Style
import sys
from sys import exit
import os
import hashlib
from omegaconf import OmegaConf

from . import csg
from .robot_description import RobotURDF#, RobotSDF


class OnshapeRobotExporter:
    def __init__(self, config_path="config.yaml"):
        self.config_path = config_path
        self.config = OmegaConf.load(self.config_path)

        # convert workspace root path as absolute path
        self.root_directory = os.path.dirname(os.path.abspath(config_path))
        print("workspace path:", self.root_directory)

        self.robot = None

        # Creating robot for output
        if self.config.outputFormat == 'urdf':
            self.robot = RobotURDF(self.config.robotName, self.config, self)
        # elif self.config.outputFormat == 'sdf':
        #     self.robot = RobotSDF(self.config.robotName, self.config)
        else:
            print(Fore.RED + 'ERROR: Unknown output format: ' +
                self.config.outputFormat +' (supported are urdf and sdf)' + Style.RESET_ALL)
            exit()
        
        self.createDirectories()

        

    def createDirectories(self):        
        # Output directory, making it if it doesn't exists
        if not os.path.exists(self.root_directory):
            os.makedirs(self.root_directory)

        # these are all relative paths
        if self.config.flavor == "ros":
            self.part_directory = "parts"
            self.urdf_directory = "urdf"
            self.mesh_directory = "meshes"
            self.scad_directory = "scad"
        else:
            self.part_directory = "parts"
            self.urdf_directory = "urdf"
            self.mesh_directory = "meshes"
            self.scad_directory = "scad"

        part_dir_abs = os.path.join(self.root_directory, self.part_directory)
        urdf_dir_abs = os.path.join(self.root_directory, self.urdf_directory)
        mesh_dir_abs = os.path.join(self.root_directory, self.mesh_directory)
        scad_dir_abs = os.path.join(self.root_directory, self.scad_directory)

        if not os.path.exists(part_dir_abs):
            os.makedirs(part_dir_abs)
        if not os.path.exists(urdf_dir_abs):
            os.makedirs(urdf_dir_abs)
        if not os.path.exists(mesh_dir_abs):
            os.makedirs(mesh_dir_abs)
        if not os.path.exists(scad_dir_abs):
            os.makedirs(scad_dir_abs)



partNames = {}

def main():
    colorama.just_fix_windows_console()

    config_path = sys.argv[1]

    exporter = OnshapeRobotExporter(config_path) 
    
    
    # Loading configuration, collecting occurrences and building robot tree
    from .load_robot import \
        config, client, tree, occurrences, getOccurrence, frames


    
    exporter.robot.drawCollisions = config['drawCollisions']
    exporter.robot.jointMaxEffort = config['jointMaxEffort']
    exporter.robot.mergeSTLs = config['mergeSTLs']
    exporter.robot.maxSTLSize = config['maxSTLSize']
    exporter.robot.simplifySTLs = config['simplifySTLs']
    exporter.robot.jointMaxVelocity = config['jointMaxVelocity']
    exporter.robot.noDynamics = config['noDynamics']
    exporter.robot.packageName = config['packageName']
    exporter.robot.addDummyBaseLink = config['addDummyBaseLink']
    exporter.robot.robotName = config['robotName']
    exporter.robot.additionalXML = config['additionalXML']
    exporter.robot.useFixedLinks = config['useFixedLinks']
    exporter.robot.meshDir


    def partIsIgnore(name):
        if config['whitelist'] is None:
            return name in config['ignore']
        else:
            return name not in config['whitelist']

    # Adds a part to the current robot link


    def addPart(occurrence, matrix, instance_name):
        part = occurrence['instance']

        if part['suppressed']:
            return

        if part['partId'] == '':
            print(Fore.YELLOW + 'WARNING: Part '+part['name']+' has no partId'+Style.RESET_ALL)
            return

        # Importing STL file for this part
        justPart, prefix = extractPartName(part['name'], part['configuration'])

        extra = ''
        if occurrence['instance']['configuration'] != 'default':
            extra = Style.DIM + ' (configuration: ' + \
                occurrence['instance']['configuration']+')'
        symbol = '+'
        if partIsIgnore(justPart):
            symbol = '-'
            extra += Style.DIM + ' / ignoring visual and collision'

        print(Fore.GREEN + symbol+' Adding part ' +
            occurrence['instance']['name']+extra + Style.RESET_ALL)

        if partIsIgnore(justPart):
            stl_path = None
        else:
            stl_path = os.path.join(exporter.mesh_directory, instance_name.split("<")[0].replace(" ", "_") + prefix.replace("/", "_")+".stl")
            
            # shorten the configuration to a maximum number of chars to prevent errors. Necessary for standard parts like screws
            if len(part['configuration']) > 40:
                shortend_configuration = hashlib.md5(
                    part['configuration'].encode('utf-8')).hexdigest()
            else:
                shortend_configuration = part['configuration']
            stl = client.part_studio_stl_m(part["documentId"], part["documentMicroversion"], part["elementId"],
                                        part["partId"], shortend_configuration)
            with open(os.path.join(exporter.root_directory, stl_path), "wb") as stream:
                stream.write(stl)

            stlMetadata = os.path.join(exporter.part_directory, prefix.replace('/', '_')+'.part')
            with open(os.path.join(exporter.root_directory, stlMetadata), 'w', encoding="utf-8") as stream:
                json.dump(part, stream, indent=4, sort_keys=True)


        # Import the SCAD files pure shapes
        shapes = None
        
        config['useScads'] = True
        if config['useScads']:
            scadFile = prefix+'.scad'
            scad_path = os.path.join(exporter.root_directory, exporter.scad_directory, scadFile)
            if os.path.exists(scad_path):
                shapes = csg.process(
                    scad_path, config['pureShapeDilatation'])
            else:
                print("generating SCAD!")
                with open(scad_path, 'w', encoding="utf-8") as stream:
                    stream.write("")

        # Obtain metadatas about part to retrieve color
        if config['color'] is not None:
            color = config['color']
        else:
            metadata = client.part_get_metadata(
                part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'], part['configuration'])

            color = [0.5, 0.5, 0.5]

            # XXX: There must be a better way to retrieve the part color
            for entry in metadata['properties']:
                if 'value' in entry and type(entry['value']) is dict and 'color' in entry['value']:
                    rgb = entry['value']['color']
                    color = np.array(
                        [rgb['red'], rgb['green'], rgb['blue']])/255.0

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
                massProperties = client.part_mass_properties(
                    part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'], part['configuration'])

                if part['partId'] not in massProperties['bodies']:
                    print(Fore.YELLOW + 'WARNING: part ' +
                        part['name']+' has no dynamics (maybe it is a surface)' + Style.RESET_ALL)
                    return
                massProperties = massProperties['bodies'][part['partId']]
                mass = massProperties['mass'][0]
                com = massProperties['centroid']
                inertia = massProperties['inertia']

                if abs(mass) < 1e-9:
                    print(Fore.YELLOW + 'WARNING: part ' +
                        part['name']+' has no mass, maybe you should assign a material to it ?' + Style.RESET_ALL)

        pose = occurrence['transform']
        if exporter.robot.relative:
            pose = np.linalg.inv(matrix)*pose

        exporter.robot.addPart(pose, stl_path, mass, com, inertia, color, shapes, prefix)


    partNames = {}


    def extractPartName(name, configuration):
        parts = name.split(' ')
        del parts[-1]
        basePartName = '_'.join(parts).lower()

        # only add configuration to name if its not default and not a very long configuration (which happens for library parts like screws)
        if configuration != 'default' and len(configuration) < 40:
            parts += ['_' + configuration.replace('=', '_').replace(' ', '_')]

        return basePartName, '_'.join(parts).lower()


    def processPartName(name, configuration, overrideName=None):
        if overrideName is None:
            global partNames
            _, name = extractPartName(name, configuration)

            if name in partNames:
                partNames[name] += 1
            else:
                partNames[name] = 1

            if partNames[name] == 1:
                return name
            else:
                return name+'_'+str(partNames[name])
        else:
            return overrideName


    def buildRobot(tree, matrix):
        occurrence = getOccurrence([tree['id']])
        instance = occurrence['instance']
        print(Fore.BLUE + Style.BRIGHT +
            '* Adding top-level instance ['+instance['name']+']' + Style.RESET_ALL)

        # Build a part name that is unique but still informative
        link = processPartName(
            instance['name'], instance['configuration'], occurrence['linkName'])

        # Create the link, collecting all children in the tree assigned to this top-level part
        exporter.robot.startLink(link, matrix)
        for occurrence in occurrences.values():
            if occurrence['assignation'] == tree['id'] and occurrence['instance']['type'] == 'Part':
                addPart(occurrence, matrix, instance['name'])
        exporter.robot.endLink()

        # Adding the frames (linkage is relative to parent)
        if tree['id'] in frames:
            for name, part in frames[tree['id']]:
                frame = getOccurrence(part)['transform']
                if exporter.robot.relative:
                    frame = np.linalg.inv(matrix)*frame
                exporter.robot.addFrame(name, frame)

        # Following the children in the tree, calling this function recursively
        k = 0
        for child in tree['children']:
            worldAxisFrame = child['axis_frame']
            zAxis = child['z_axis']
            jointType = child['jointType']
            jointLimits = child['jointLimits']

            if exporter.robot.relative:
                axisFrame = np.linalg.inv(matrix)*worldAxisFrame
                childMatrix = worldAxisFrame
            else:
                # In SDF format, everything is expressed in the world frame, in this case
                # childMatrix will be always identity
                axisFrame = worldAxisFrame
                childMatrix = matrix

            subLink = buildRobot(child, childMatrix)
            exporter.robot.addJoint(jointType, link, subLink, axisFrame,
                        child['dof_name'], jointLimits, zAxis)

        return link


    # Start building the robot
    buildRobot(tree, np.matrix(np.identity(4)))
    exporter.robot.finalize()
    # print(tree)

    print("\n" + Style.BRIGHT + "* Writing " +
        exporter.robot.ext.upper()+" file" + Style.RESET_ALL)
    with open(os.path.join(exporter.root_directory, exporter.urdf_directory, "robot."+exporter.robot.ext), 'w', encoding="utf-8") as stream:
        stream.write(exporter.robot.xml)

    if len(config['postImportCommands']):
        print("\n" + Style.BRIGHT + "* Executing post-import commands" + Style.RESET_ALL)
        for command in config['postImportCommands']:
            print("* "+command)
            os.system(command)


if __name__ == "__main__":

    if len(sys.argv) <= 1:
        print(Fore.RED +
            'ERROR: usage: onshape-to-robot {robot_directory}' + Style.RESET_ALL)
        print("Read documentation at https://onshape-to-robot.readthedocs.io/")
        exit("")
    
    main()