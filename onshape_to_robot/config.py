from sys import exit
import sys
import os
import commentjson as json
from colorama import Fore, Back, Style

config = {}

# Loading configuration & parameters
if len(sys.argv) <= 1:
    exit(Fore.RED + 'ERROR: usage: onshape-to-robot.py [robot_directory]' + Style.RESET_ALL)
    exit("Read documentation at https://github.com/rhoban/onshape-to-robot/#onshape-to-robot-sdfurdf")
robot = sys.argv[1]


def configGet(name, default=None):
    global config
    if name in config:
        return config[name]
    else:
        if default is None:
            print(Fore.RED + 'ERROR: missing key "'+name+'" in config' + Style.RESET_ALL)
            exit()
        else:
            return default


configFile = robot+'/config.json'
config = json.load(open(configFile))

config['documentId'] = configGet('documentId')
config['versionId'] = configGet('versionId', '')
config['drawFrames'] = configGet('drawFrames', False)
config['drawCollisions'] = configGet('drawCollisions', False)
config['assemblyName'] = configGet('assemblyName', False)
config['outputFormat'] = configGet('outputFormat', 'urdf')
config['connectWithFixedLinks'] = configGet('connectWithFixedLinks', True)

# Using OpenSCAD for simplified geometry
config['useScads'] = configGet('useScads', True)

# Dynamics
config['jointMaxEffort'] = configGet('jointMaxEffort', 1)
config['jointMaxVelocity'] = configGet('jointMaxVelocity', 20)
config['noDynamics'] = configGet('noDynamics', False)

# Ignore list
config['ignore'] = configGet('ignore', [])

# STLs merge and simplification
config['mergeSTLs'] = configGet('mergeSTLs', False)
config['maxSTLSize'] = configGet('maxSTLSize', 3)
config['simplifySTLs'] = configGet('simplifySTLs', False)

config['outputDirectory'] = robot
config['dynamicsOverride'] = {}

# ROS support
config['packageName'] = configGet('packageName', '')
config['addDummyBaseLink'] = configGet('addDummyBaseLink', False)
config['robotName'] = configGet('robotName', 'onshape')

# additional XML code to insert
if config['outputFormat'] == 'urdf':
    additionalFileName = configGet('additionalUrdfFile', '')
else:
    additionalFileName = configGet('additionalSdfFile', '')

if additionalFileName == '':
    config['additionalXML'] = ''
else:
    with open(robot + additionalFileName, 'r') as additionalXMLFile:
        config['additionalXML'] = additionalXMLFile.read()


# Creating dynamics override array
tmp = configGet('dynamics', {})
for key in tmp:
    if tmp[key] == 'fixed':
        config['dynamicsOverride'][key.lower()] = {"com": [0,0,0], "mass": 0, "inertia": [0,0,0,0,0,0,0,0,0]}
    else:
        config['dynamicsOverride'][key.lower()] = tmp[key]

# Output directory, making it if it doesn't exists
try:
    os.makedirs(config['outputDirectory'])
except OSError:
    pass

# Checking that OpenSCAD is present
if config['useScads']:
    print(Style.BRIGHT + '* Checking OpenSCAD presence...' + Style.RESET_ALL)
    if os.system('openscad -v 2> /dev/null') != 0:
        print(Fore.RED + "Can't run openscad -v, disabling OpenSCAD support" + Style.RESET_ALL)
        print(Fore.BLUE + "TIP: consider installing openscad:" + Style.RESET_ALL)
        print(Fore.BLUE + "sudo add-apt-repository ppa:openscad/releases" + Style.RESET_ALL)
        print(Fore.BLUE + "sudo apt-get update" + Style.RESET_ALL)
        print(Fore.BLUE + "sudo apt-get install openscad" + Style.RESET_ALL)
        config['useScads'] = False

# Checking that MeshLab is present
if config['simplifySTLs']:
    print(Style.BRIGHT + '* Checking MeshLab presence...' + Style.RESET_ALL)
    if not os.path.exists('/usr/bin/meshlabserver') != 0:
        print(Fore.RED + "No /usr/bin/meshlabserver, disabling STL simplification support" + Style.RESET_ALL)
        print(Fore.BLUE + "TIP: consider installing meshlab:" + Style.RESET_ALL)
        print(Fore.BLUE + "sudo apt-get install meshlab" + Style.RESET_ALL)
        config['simplifySTLs'] = False
