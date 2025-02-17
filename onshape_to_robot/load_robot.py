from collections import defaultdict
import math
from sys import exit
import numpy as np
import uuid
from .onshape_api.client import Client
from .config import config, configFile
from .message import error, info, bright, success, warning

# OnShape API client
workspaceId = None
client = Client(logging=False, creds=configFile)
client.useCollisionsConfigurations = config["useCollisionsConfigurations"]

# If a versionId is provided, it will be used, else the main workspace is retrieved
print("")
if config["versionId"] != "":
    print(bright(f"* Using configuration version ID {config['versionId']} ..."))
elif config["workspaceId"] != "":
    print(bright(f"* Using configuration workspace ID {config['workspaceId']} ..."))
    workspaceId = config["workspaceId"]
else:
    print(bright("* Retrieving workspace ID ..."))
    document = client.get_document(config["documentId"])
    workspaceId = document["defaultWorkspace"]["id"]
    print(success(f"+ Using workspace id: {workspaceId}"))

# Now, finding the assembly, according to given name in configuration, or else the first possible one
print("")
print(bright("* Retrieving elements in the document, searching for the assembly..."))
if config["versionId"] != "":
    elements = client.list_elements(
        config["documentId"], config["versionId"], "v"
    )
else:
    elements = client.list_elements(config["documentId"], workspaceId)
assemblyId = None
assemblyName = ""
for element in elements:
    if element["type"] == "Assembly" and (
        config["assemblyName"] is False or element["name"] == config["assemblyName"]
    ):
        print(success(f"+ Found assembly, id: {element['id']}, name: {element['name']}"))
        assemblyName = element["name"]
        assemblyId = element["id"]

if assemblyId == None:
    print(error(f"ERROR: Unable to find assembly in this document"))
    exit(1)

# Retrieving the assembly
print(bright(f"* Retrieving assembly \"{assemblyName}\" with id {assemblyId}"))
if config["versionId"] != "":
    assembly = client.get_assembly(
        config["documentId"],
        config["versionId"],
        assemblyId,
        "v",
        configuration=config["configuration"],
    )
else:
    assembly = client.get_assembly(
        config["documentId"],
        workspaceId,
        assemblyId,
        configuration=config["configuration"],
    )

root = assembly["rootAssembly"]

# Finds a (leaf) instance given the full path, typically A B C where A and B would be subassemblies and C
# the final part


def findInstance(path, instances=None):
    global assembly

    if instances is None:
        instances = assembly["rootAssembly"]["instances"]

    for instance in instances:
        if instance["id"] == path[0]:
            if len(path) == 1:
                # If the length of remaining path is 1, the part is in the current assembly/subassembly
                return instance
            else:
                # Else, we need to find the matching sub assembly to find the proper part (recursively)
                d = instance["documentId"]
                m = instance["documentMicroversion"]
                e = instance["elementId"]
                for asm in assembly["subAssemblies"]:
                    if (
                        asm["documentId"] == d
                        and asm["documentMicroversion"] == m
                        and asm["elementId"] == e
                    ):
                        return findInstance(path[1:], asm["instances"])

    print(error(f"Could not find instance for {str(path)}"))


# Collecting occurrences, the path is the assembly / sub assembly chain
occurrences = {}
for occurrence in root["occurrences"]:
    occurrence["assignation"] = None
    occurrence["instance"] = findInstance(occurrence["path"])
    occurrence["transform"] = np.matrix(np.reshape(occurrence["transform"], (4, 4)))
    occurrence["linkName"] = None
    occurrences[tuple(occurrence["path"])] = occurrence

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
frames = defaultdict(list)


def assignParts(root, parent):
    assignations[root] = parent
    for occurrence in occurrences.values():
        if occurrence["path"][0] == root:
            occurrence["assignation"] = parent


from .features import init as features_init, getLimits

features_init(client, config, root, workspaceId, assemblyId)


def get_T_part_mate(matedEntity: dict):
    T_part_mate = np.eye(4)
    T_part_mate[:3, :3] = np.stack(
        (
            np.array(matedEntity["matedCS"]["xAxis"]),
            np.array(matedEntity["matedCS"]["yAxis"]),
            np.array(matedEntity["matedCS"]["zAxis"]),
        )
    ).T
    T_part_mate[:3, 3] = matedEntity["matedCS"]["origin"]

    return T_part_mate


# First, features are scanned to find the DOFs. Links that they connects are then tagged
print("")
print(bright("* Getting assembly features, scanning for DOFs..."))
trunk = None
relations = {}
features = root["features"]
for feature in features:
    if feature["featureType"] == "mateConnector":
        name = feature["featureData"]["name"]
        if name[0:5] == "link_":
            name = name[5:]
            occurrences[(feature["featureData"]["occurrence"][0],)]["linkName"] = name
    else:
        if feature["suppressed"]:
            continue

        data = feature["featureData"]

        if (
            "matedEntities" not in data
            or len(data["matedEntities"]) != 2
            or len(data["matedEntities"][0]["matedOccurrence"]) == 0
            or len(data["matedEntities"][1]["matedOccurrence"]) == 0
        ):
            continue

        child = data["matedEntities"][0]["matedOccurrence"][0]
        parent = data["matedEntities"][1]["matedOccurrence"][0]

        if data["name"].startswith("closing_"):
            for k in 0, 1:
                matedEntity = data["matedEntities"][k]
                occurrence = matedEntity["matedOccurrence"][0]

                T_world_part = getOccurrence(matedEntity["matedOccurrence"])[
                    "transform"
                ]
                T_part_mate = get_T_part_mate(matedEntity)
                T_world_mate = T_world_part * T_part_mate

                frames[occurrence].append([f"{data['name']}_{k+1}", T_world_mate])
        elif data["name"].startswith("dof_"):
            parts = data["name"].split("_")
            del parts[0]
            data["inverted"] = False
            if parts[-1] == "inv" or parts[-1] == "inverted":
                data["inverted"] = True
                del parts[-1]
            name = "_".join(parts)
            if name == "":
                print(error(f"ERROR: a dof doesn't have any name ({data['name']} should be \"dof_...\")"))
                exit()

            limits = None
            if data["mateType"] == "REVOLUTE" or data["mateType"] == "CYLINDRICAL":
                if "wheel" in parts or "continuous" in parts:
                    jointType = "continuous"
                else:
                    jointType = "revolute"

                if not config["ignoreLimits"]:
                    limits = getLimits(jointType, data["name"])
            elif data["mateType"] == "SLIDER":
                jointType = "prismatic"
                if not config["ignoreLimits"]:
                    limits = getLimits(jointType, data["name"])
            elif data["mateType"] == "FASTENED":
                jointType = "fixed"
            else:
                print(error(f"ERROR: {name} is declared as a DOF but the mate type is {data['mateType']}"))
                print(error("       Only REVOLUTE, CYLINDRICAL, SLIDER and FASTENED are supported"))
                exit(1)

            # We compute the axis in the world frame
            matedEntity = data["matedEntities"][0]
            T_world_part = getOccurrence(matedEntity["matedOccurrence"])["transform"]

            # jointToPart is the (rotation only) matrix from joint to the part
            # it is attached to
            T_part_mate = get_T_part_mate(matedEntity)

            if data["inverted"]:
                if limits is not None:
                    limits = (-limits[1], -limits[0])

                # Flipping the joint around X axis
                flip = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
                T_part_mate[:3, :3] = T_part_mate[:3, :3] @ flip

            T_world_mate = T_world_part * T_part_mate

            limitsStr = ""
            if limits is not None:
                limitsStr = (
                    "["
                    + str(round(limits[0], 3))
                    + ": "
                    + str(round(limits[1], 3))
                    + "]"
                )
            print(success(f"+ Found DOF: {name} ({jointType}) {limitsStr}"))

            if child in relations:
                print(error(f"Error, the relation {name} is connecting a child that is already connected"))
                print("Be sure you ordered properly your relations, see:")
                print(
                    "https://onshape-to-robot.readthedocs.io/en/latest/design.html#specifying-degrees-of-freedom"
                )
                exit()

            relations[child] = {
                "parent": parent,
                "worldAxisFrame": T_world_mate,
                "zAxis": np.array([0, 0, 1]),
                "name": name,
                "type": jointType,
                "limits": limits,
            }

            assignParts(child, child)
            assignParts(parent, parent)

print(success(f"* Found total {len(relations)} DOFs"))

# If we have no DOF
if len(relations) == 0:
    trunk = root["instances"][0]["id"]
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
        if feature["featureType"] != "mate" or feature["suppressed"]:
            continue

        data = feature["featureData"]

        if (
            len(data["matedEntities"]) != 2
            or len(data["matedEntities"][0]["matedOccurrence"]) == 0
            or len(data["matedEntities"][1]["matedOccurrence"]) == 0
        ):
            continue

        occurrenceA = data["matedEntities"][0]["matedOccurrence"][0]
        occurrenceB = data["matedEntities"][1]["matedOccurrence"][0]

        if (occurrenceA not in assignations) != (occurrenceB not in assignations):
            if data["name"].startswith("frame_"):
                # In case of a constraint naemd "frame_", we add it as a frame, we draw it if drawFrames is True
                name = "_".join(data["name"].split("_")[1:])
                if occurrenceA in assignations:
                    frames[occurrenceA].append(
                        [name, data["matedEntities"][1]["matedOccurrence"]]
                    )
                    assignParts(
                        occurrenceB,
                        {True: assignations[occurrenceA], False: "frame"}[
                            config["drawFrames"]
                        ],
                    )
                else:
                    frames[occurrenceB].append(
                        [name, data["matedEntities"][0]["matedOccurrence"]]
                    )
                    assignParts(
                        occurrenceA,
                        {True: assignations[occurrenceB], False: "frame"}[
                            config["drawFrames"]
                        ],
                    )
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
print("")
print(bright("* Building robot tree"))

for childId in relations:
    entry = relations[childId]
    if entry["parent"] not in relations:
        trunk = entry["parent"]
        break
trunkOccurrence = getOccurrence([trunk])
print(bright(f"* Trunk is {trunkOccurrence['instance']['name']}"))

for occurrence in occurrences.values():
    if occurrence["assignation"] is None:
        print(warning(f"WARNING: part ({occurrence['instance']['name']}) has no assignation, connecting it with trunk"))
        child = occurrence["path"][0]
        connectParts(child, trunk)

# If a sub-assembly is suppressed, we also mark as suppressed the parts in this sub-assembly
for occurrence in occurrences.values():
    if not occurrence["instance"]["suppressed"]:
        for k in range(len(occurrence["path"]) - 1):
            upper_path = tuple(occurrence["path"][0 : k + 1])
            if (
                upper_path in occurrences
                and occurrences[upper_path]["instance"]["suppressed"]
            ):
                occurrence["instance"]["suppressed"] = True


def collect(id):
    part = {}
    part["id"] = id
    part["children"] = []
    for childId in relations:
        entry = relations[childId]
        if entry["parent"] == id:
            child = collect(childId)
            child["axis_frame"] = entry["worldAxisFrame"]
            child["z_axis"] = entry["zAxis"]
            child["dof_name"] = entry["name"]
            child["jointType"] = entry["type"]
            child["jointLimits"] = entry["limits"]
            part["children"].append(child)
    return part


tree = collect(trunk)
