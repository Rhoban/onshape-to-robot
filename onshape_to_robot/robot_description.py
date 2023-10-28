import os
import math
import uuid

import numpy as np

from . import stl_combine

def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def origin(matrix):
    urdf = '<origin xyz="%.20g %.20g %.20g" rpy="%.20g %.20g %.20g" />'
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    rpy = rotationMatrixToEulerAngles(matrix)

    return urdf % (x, y, z, rpy[0], rpy[1], rpy[2])


def pose(matrix, frame=''):
    sdf = '<pose>%.20g %.20g %.20g %.20g %.20g %.20g</pose>'
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    rpy = rotationMatrixToEulerAngles(matrix)

    if frame != '':
        sdf = '<frame name="'+frame+'_frame">'+sdf+'</frame>'

    return sdf % (x, y, z, rpy[0], rpy[1], rpy[2])


class RobotDescription(object):
    def __init__(self, name, config=None, exporter=None):
        self.config = config
        self.exporter = exporter
        # self.drawCollisions = False
        self.relative = True
        # self.mergeSTLs = 'no'
        # self.mergeSTLsCollisions = False
        # self.useFixedLinks = False
        # self.simplifySTLs = 'no'
        # self.maxSTLSize = 3
        # self.jointMaxEffort = 1
        # self.jointMaxVelocity = 10
        # self.noDynamics = False
        # self.packageName = ""
        self.robotName = name
        self.meshDir = None

        self.json = {}
        self.floating_precision = 6

    def getOrigin(self, transform):
        rpy = rotationMatrixToEulerAngles(transform)
        return {
            "xyz": "{:.{precision}f} {:.{precision}f} {:.{precision}f}".format(transform[0, 3], transform[1, 3], transform[2, 3], precision=self.floating_precision),
            "rpy": "{:.{precision}f} {:.{precision}f} {:.{precision}f}".format(rpy[0], rpy[1], rpy[2], precision=self.floating_precision),
        }
        
    def jointMaxEffortFor(self, jointName):
        if isinstance(self.config.jointMaxEffort, int) or isinstance(self.config.jointMaxEffort, float):
            return self.config.jointMaxEffort
        if jointName in self.config.jointMaxEffort:
            return self.config.jointMaxEffort[jointName]
        return self.config.jointMaxEffort["default"]

    def jointMaxVelocityFor(self, jointName):
        if isinstance(self.config.jointMaxVelocity, int) or isinstance(self.config.jointMaxVelocity, float):
            return self.config.jointMaxVelocity
        if jointName in self.config.jointMaxVelocity:
            return self.config.jointMaxVelocity[jointName]
        return self.config.jointMaxVelocity["default"]



    def mergeSTL(self, stl_path, matrix, color, mass, node='visual'):
        if node == 'visual':
            self.exporter._color += np.array(color) * mass
            self.exporter._color_mass += mass

        m = stl_combine.load_mesh(os.path.join(self.exporter.root_directory, stl_path))
        stl_combine.apply_matrix(m, matrix)

        if self.exporter._mesh[node] is None:
            self.exporter._mesh[node] = m
        else:
            self.exporter._mesh[node] = stl_combine.combine_meshes(self.exporter._mesh[node], m)


class RobotURDF(RobotDescription):
    def __init__(self, name, config=None, exporter=None):
        super().__init__(name)
        self.config = config
        self.exporter = exporter
        self.ext = 'urdf'
        self.json = {
            "robot": {
                "name": self.robotName,
                "link": [],
                "joint": [],
            }
        }
        pass

    def addDummyLink(self, link_name, visualMatrix=None, visualSTL=None, visualColor=None):
        if self.noDynamics:
            mass_value = "0"
        else:
            mass_value = "1e-9"
        dummy_link = {
            "name": link_name,
            "inertial": {
                "origin": {
                    "xyz": "{:.{precision}f} {:.{precision}f} {:.{precision}f}".format(0, 0, 0, precision=self.floating_precision),
                    "rpy": "{:.{precision}f} {:.{precision}f} {:.{precision}f}".format(0, 0, 0, precision=self.floating_precision),
                },
                "mass": {
                    "value": mass_value,
                },
                "inertia": {
                    "ixx": "0", "ixy": "0", "ixz": "0",
                                "iyy": "0", "iyz": "0",
                                            "izz": "0",
                },
            },
        }
        self.json["robot"]["link"].append(dummy_link)
        if visualSTL is not None:
            self.addSTL(link_name, visualMatrix, visualSTL, visualColor,
                        link_name+"_visual", 'visual')

    def addDummyBaseLink(self, name):
        # adds a dummy base_link for ROS users
        dummy_base_link = {
            "name": "base_link",
        }
        dummy_base_joint = {
            "name": "base_link_to_base",
            "type": "fixed",
            "parent": {
                "link": "base_link",
            },
            "child": {
                "link": name
            },
            "origin": {
                "xyz": "{:.{precision}f} {:.{precision}f} {:.{precision}f}".format(0, 0, 0, precision=self.floating_precision),
                "rpy": "{:.{precision}f} {:.{precision}f} {:.{precision}f}".format(0, 0, 0, precision=self.floating_precision),
            }
        }
        self.json["robot"]["link"].append(dummy_base_link)
        self.json["robot"]["joint"].append(dummy_base_joint)

    def addFixedJoint(self, parent, child, matrix, name=None):
        if name is None:
            name = parent+'_'+child+'_fixing'

        self.json["joints"].append({
            "name": name,
            "origin": origin(matrix),
            "parent": {
                "link": parent
            },
            "child": {
                "link": child
            },
            "axis": {
                "xyz": "0 0 0"
            }
        })
        
    def getLinkIndex(self, link_name):
        for idx, link in enumerate(self.json["robot"]["link"]):
            if link["name"] == link_name:
                return idx
        return None

    def createLink(self, link_name):
        # Check if link already exists
        for link in self.json["robot"]["link"]:
            if link["name"] == link_name:
                raise Exception("Link name already exists: " + link_name)
        
        self.json["robot"]["link"].append({"name": link_name, "$t": None})
        

    def endLink(self, link_name, mass, com, inertia):
        
        inertial = {
            "origin": {
                "xyz": "{:.{precision}f} {:.{precision}f} {:.{precision}f}".format(com[0], com[1], com[2], precision=self.floating_precision),
                "rpy": "{:.{precision}f} {:.{precision}f} {:.{precision}f}".format(0, 0, 0, precision=self.floating_precision),
            },
            "mass": {
                "value": "{:.{precision}f}".format(mass, precision=self.floating_precision),
            },
            "inertia": {
                "ixx": "{:.{precision}f}".format(inertia[0, 0], precision=self.floating_precision),
                "ixy": "{:.{precision}f}".format(inertia[0, 1], precision=self.floating_precision),
                "ixz": "{:.{precision}f}".format(inertia[0, 2], precision=self.floating_precision),
                "iyy": "{:.{precision}f}".format(inertia[1, 1], precision=self.floating_precision),
                "iyz": "{:.{precision}f}".format(inertia[1, 2], precision=self.floating_precision),
                "izz": "{:.{precision}f}".format(inertia[2, 2], precision=self.floating_precision),
            }
        }
        
        idx = self.getLinkIndex(link_name)
        self.json["robot"]["link"][idx]["inertial"] = inertial

        if self.config.useFixedLinks:
            inertial["visual"] = {
                "geometry": {"box": {"size": "0 0 0"}}
            }

    def addFixedVisualLink(self, link_name):
        if self.config.useFixedLinks:
            n = 0
            for visual in self._visuals:
                n += 1
                visual_name = '%s_%d' % (link_name, n)
                self.addDummyLink(visual_name, visual[0], visual[1], visual[2])
                self.addJoint("fixed", link_name, visual_name,
                              np.eye(4), visual_name+'_fixing', None)

    def addFrame(self, frame_name, name, matrix):
        # Adding a dummy link
        self.addDummyLink(frame_name, name)

        # Linking it with last link with a fixed link
        self.addFixedJoint(frame_name, name, matrix, name+'_frame')

    def addSTL(self, link_name, matrix, stl_path, color, name, node='visual'):
        if self.config.flavor == "gym":
            mesh_path = stl_path
        elif self.config.flavor == "ros":
            package_name = self.config.ros_package_name
            mesh_path = "package://{package}/{uri}".format(package=package_name, uri=stl_path)
        node_data = {
            "origin": self.getOrigin(matrix),
            "geometry": {
                "mesh": {
                    "filename": mesh_path,
                },
            },
        }
        if node == 'visual':
            node_data["material"] = {
                "name": name+"_material",
                "color": {
                    "rgba": "{:.1f} {:.1f} {:.1f} 1.0".format(color[0], color[1], color[2]),
                },
            }
        
        idx = self.getLinkIndex(link_name)
        self.json["robot"]["link"][idx][node] = node_data



    def addJoint(self, joint_type, parent_link, child_link, transform, name, joint_limits, z_axis=[0, 0, 1]):
        for j in self.json["robot"]["joint"]:
            if j["name"] == name:
                raise Exception("Joint name already exists: " + name)
        joint = {
            "name": name,
            "type": joint_type,
            "parent": {
                "link": parent_link,
            },
            "child": {
                "link": child_link,
            },
            "origin": self.getOrigin(transform),
            "axis": {
                "xyz": "{:.{precision}f} {:.{precision}f} {:.{precision}f}".format(z_axis[0], z_axis[1], z_axis[2], precision=self.floating_precision),
            },
            "limit": {
                "effort": "{:.{precision}f}".format(self.jointMaxEffortFor(name), precision=self.floating_precision),
                "velocity": "{:.{precision}f}".format(self.jointMaxVelocityFor(name), precision=self.floating_precision),
            },
            "joint_properties": {
                "friction": "{:.{precision}f}".format(0, precision=self.floating_precision),
            },
        }
        if joint_limits is not None:
            joint["limit"]["lower"] = joint_limits[0]
            joint["limit"]["upper"] = joint_limits[1]

        self.json["robot"]["joint"].append(joint)

    def finalize(self):
        pass
        # self.append(self.additionalXML)

