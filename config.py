import commentjson as json
import sys
import os
from colorama import Fore, Back, Style

config = {}

# Loading configuration & parameters
if len(sys.argv) <= 1:
    exit(Fore.RED + 'ERROR: usage: onshape-to-robot.py [robot_directory]' + Style.RESET_ALL)
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
config['drawFrames'] = configGet('drawFrames')
config['drawCollisions'] = configGet('drawCollisions', False)
config['useScads'] = configGet('useScads', True)
config['assemblyName'] = configGet('assemblyName', False)
config['outputFormat'] = configGet('outputFormat', 'urdf')
config['jointMaxEffort'] = configGet('jointMaxEffort', 1)
config['jointMaxVelocity'] = configGet('jointMaxVelocity', 20)
config['noDynamics'] = configGet('noDynamics', False)
config['ignore'] = configGet('ignore', [])
config['outputDirectory'] = robot
config['dynamicsOverride'] = {}

# Creating dynamics override array
tmp = configGet('dynamics', {})
for key in tmp:
    config['dynamicsOverride'][key.lower()] = tmp[key]

# Output directory, making it if it doesn't exists
try:
    os.makedirs(config['outputDirectory'])
except OSError:
    pass

# Checking that OpenSCAD is present
if config['useScads']:
    print(Style.BRIGHT + '* Checking OpenSCAD version...' + Style.RESET_ALL)
    if os.system('openscad -v') != 0:
        print(Fore.RED + "Can't run openscad -v, disabling OpenSCAD support" + Style.RESET_ALL)
        config['useScads'] = False
