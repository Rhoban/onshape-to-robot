from sys import exit
import sys
import os
import commentjson as json
import subprocess
from .message import error, bright, info

config = {}

# Loading configuration & parameters
if len(sys.argv) <= 1:
    print(error("ERROR: usage: onshape-to-robot {robot_directory}"))
    print(error("Read documentation at https://onshape-to-robot.readthedocs.io/"))
    exit("")
robot = sys.argv[1]


def configGet(name, default=None, hasDefault=False, valuesList=None):
    global config
    hasDefault = hasDefault or (default is not None)

    if name in config:
        value = config[name]
        if valuesList is not None and value not in valuesList:
            print(
                error(
                    f"ERROR: Value for {name} should be onf of: {','.join(valuesList)}"
                )
            )
            exit()
        return value
    else:
        if hasDefault:
            return default
        else:
            print(error(f"ERROR: missing key {name} in config"))
            exit()


configFile = robot + "/config.json"
if not os.path.exists(configFile):
    print(error(f"ERROR: The file {configFile} can't be found"))
    exit()
with open(configFile, "r", encoding="utf8") as stream:
    config = json.load(stream)

config["documentId"] = configGet("documentId")
config["versionId"] = configGet("versionId", "")
config["workspaceId"] = configGet("workspaceId", "")
config["drawFrames"] = configGet("drawFrames", False)
config["drawCollisions"] = configGet("drawCollisions", False)
config["assemblyName"] = configGet("assemblyName", False)
config["outputFormat"] = configGet("outputFormat", "urdf")
config["useFixedLinks"] = configGet("useFixedLinks", False)
config["configuration"] = configGet("configuration", "default")
config["ignoreLimits"] = configGet("ignoreLimits", False)

# Using OpenSCAD for simplified geometry
config["useScads"] = configGet("useScads", True)
config["pureShapeDilatation"] = configGet("pureShapeDilatation", 0.0)

# Dynamics
config["jointMaxEffort"] = configGet("jointMaxEffort", 1)
config["jointMaxVelocity"] = configGet("jointMaxVelocity", 20)
config["noDynamics"] = configGet("noDynamics", False)

# Ignore list
config["ignore"] = configGet("ignore", [])
config["whitelist"] = configGet("whitelist", None, hasDefault=True)

# Color override
config["color"] = configGet("color", None, hasDefault=True)

# STLs merge and simplification
config["mergeSTLs"] = configGet(
    "mergeSTLs", "no", valuesList=["no", "visual", "collision", "all"]
)
config["maxSTLSize"] = configGet("maxSTLSize", 3)
config["simplifySTLs"] = configGet(
    "simplifySTLs", "no", valuesList=["no", "visual", "collision", "all"]
)

# Post-import commands to execute
config["postImportCommands"] = configGet("postImportCommands", [])

config["outputDirectory"] = robot
config["dynamicsOverride"] = {}

# Add collisions=true configuration on parts
config["useCollisionsConfigurations"] = configGet("useCollisionsConfigurations", True)

# ROS support
config["packageName"] = configGet("packageName", "")
config["addDummyBaseLink"] = configGet("addDummyBaseLink", False)
config["robotName"] = configGet("robotName", "onshape")

# additional XML code to insert
if config["outputFormat"] == "urdf":
    additionalFileName = configGet("additionalUrdfFile", "")
else:
    additionalFileName = configGet("additionalSdfFile", "")

if additionalFileName == "":
    config["additionalXML"] = ""
else:
    with open(robot + additionalFileName, "r", encoding="utf-8") as stream:
        config["additionalXML"] = stream.read()


# Creating dynamics override array
tmp = configGet("dynamics", {})
for key in tmp:
    if tmp[key] == "fixed":
        config["dynamicsOverride"][key.lower()] = {
            "com": [0, 0, 0],
            "mass": 0,
            "inertia": [0, 0, 0, 0, 0, 0, 0, 0, 0],
        }
    else:
        config["dynamicsOverride"][key.lower()] = tmp[key]

# Output directory, making it if it doesn't exists
try:
    os.makedirs(config["outputDirectory"])
except OSError:
    pass

# Checking that OpenSCAD is present
if config["useScads"]:
    print(bright("* Checking OpenSCAD presence..."))
    try:
        subprocess.run(["openscad", "-v"])
    except FileNotFoundError:
        print(bright("Can't run openscad -v, disabling OpenSCAD support"))
        print(info("TIP: consider installing openscad:"))
        print(info("Linux:"))
        print(info("sudo add-apt-repository ppa:openscad/releases"))
        print(info("sudo apt-get update"))
        print(info("sudo apt-get install openscad"))
        print(info("Windows:"))
        print(info("go to: https://openscad.org/downloads.html "))
        config["useScads"] = False

# Checking that MeshLab is present
if config["simplifySTLs"]:
    print(bright("* Checking MeshLab presence..."))
    if not os.path.exists("/usr/bin/meshlabserver") != 0:
        print(error("No /usr/bin/meshlabserver, disabling STL simplification support"))
        print(info("TIP: consider installing meshlab:"))
        print(info("sudo apt-get install meshlab"))
        config["simplifySTLs"] = False

# Checking that versionId and workspaceId are not set on same time
if config["versionId"] != "" and config["workspaceId"] != "":
    print(error("You can't specify workspaceId AND versionId"))
