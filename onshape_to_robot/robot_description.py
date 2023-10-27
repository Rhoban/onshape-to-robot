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
        self.current_link_idx = 0
        self.floating_precision = 6
        self.added_dummy_link = False

    def origin_(self, transform):
        rpy = rotationMatrixToEulerAngles(transform)
        return {
            "xyz": "{:.{precision}f} {:.{precision}f} {:.{precision}f}".format(transform[0, 3], transform[1, 3], transform[2, 3], precision=self.floating_precision),
            "rpy": "{:.{precision}f} {:.{precision}f} {:.{precision}f}".format(rpy[0], rpy[1], rpy[2], precision=self.floating_precision),
        }
    def shouldMergeSTLs(self, node):
        return self.config.mergeSTLs == 'all' or self.config.mergeSTLs == node

    def shouldSimplifySTLs(self, node):
        return self.config.simplifySTLs == 'all' or self.config.simplifySTLs == node

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

    def resetLink(self):
        self._mesh = {'visual': None, 'collision': None}
        self._color = np.array([0., 0., 0.])
        self._color_mass = 0
        self._link_childs = 0
        self._visuals = []
        self._dynamics = []

    def addLinkDynamics(self, matrix, mass, com, inertia):
        # Inertia
        I = np.matrix(np.reshape(inertia[:9], (3, 3)))
        R = matrix[:3, :3]
        # Expressing COM in the link frame
        com = np.array(
            (matrix*np.matrix([com[0], com[1], com[2], 1]).T).T)[0][:3]
        # Expressing inertia in the link frame
        inertia = R*I*R.T

        self._dynamics.append({
            'mass': mass,
            'com': com,
            'inertia': inertia
        })

    def mergeSTL(self, stl_path, matrix, color, mass, node='visual'):
        if node == 'visual':
            self._color += np.array(color) * mass
            self._color_mass += mass

        m = stl_combine.load_mesh(os.path.join(self.exporter.root_directory, stl_path))
        stl_combine.apply_matrix(m, matrix)

        if self._mesh[node] is None:
            self._mesh[node] = m
        else:
            self._mesh[node] = stl_combine.combine_meshes(self._mesh[node], m)

    def linkDynamics(self):
        mass = 0
        com = np.array([0.0]*3)
        inertia = np.matrix(np.zeros((3, 3)))
        identity = np.matrix(np.eye(3))

        for dynamic in self._dynamics:
            mass += dynamic['mass']
            com += dynamic['com']*dynamic['mass']

        if mass > 0:
            com /= mass

        # https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=246
        for dynamic in self._dynamics:
            r = dynamic['com'] - com
            p = np.matrix(r)
            inertia += dynamic['inertia'] + \
                (np.dot(r, r)*identity - p.T*p)*dynamic['mass']

        return mass, com, inertia


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

    def addDummyLink(self, name, visualMatrix=None, visualSTL=None, visualColor=None):
        if self.noDynamics:
            mass_value = "0"
        else:
            mass_value = "1e-9"
        dummy_link = {
            "name": name,
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
            self.addSTL(visualMatrix, visualSTL, visualColor,
                        name+"_visual", 'visual')

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

    def startLink(self, name, matrix):
        self._link_name = name
        self.resetLink()

        if not self.added_dummy_link and self.config.addDummyBaseLink:
            self.addDummyBaseLink(name)
            
            self.added_dummy_link = True

        self.json["robot"]["link"].append({"name": name, "$t": None})

        self.current_link_idx = len(self.json["robot"]["link"]) - 1
        

    def endLink(self):
        mass, com, inertia = self.linkDynamics()

        for node in ['visual', 'collision']:
            if self._mesh[node] is not None:
                if node == 'visual' and self._color_mass > 0:
                    color = self._color / self._color_mass
                else:
                    color = [0.5, 0.5, 0.5]

                stl_filename = "{link}_{node}.stl".format(link=self._link_name, node=node)
                stl_path = os.path.join(
                    self.exporter.config.outputDirectory.meshes, 
                    self.config.packageName.strip("/"), 
                    stl_filename)
                stl_combine.save_mesh(
                    self._mesh[node], os.path.join(self.exporter.root_directory, stl_path))
                if self.shouldSimplifySTLs(node):
                    stl_combine.simplify_stl(os.path.join(self.exporter.root_directory, stl_path), self.maxSTLSize)
                self.addSTL(np.identity(4), stl_path, color, self._link_name, node)

        link_target = None
        
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
        self.json["robot"]["link"][self.current_link_idx]["inertial"] = inertial

        if self.config.useFixedLinks:
            inertial["visual"] = {
                "geometry": {"box": {"size": "0 0 0"}}
            }


        if self.config.useFixedLinks:
            n = 0
            for visual in self._visuals:
                n += 1
                visual_name = '%s_%d' % (self._link_name, n)
                self.addDummyLink(visual_name, visual[0], visual[1], visual[2])
                self.addJoint('fixed', self._link_name, visual_name,
                              np.eye(4), visual_name+'_fixing', None)

    def addFrame(self, name, matrix):
        # Adding a dummy link
        self.addDummyLink(name)

        # Linking it with last link with a fixed link
        self.addFixedJoint(self._link_name, name, matrix, name+'_frame')

    def addSTL(self, matrix, stl_path, color, name, node='visual'):

        package_name = self.config.ros_package_name
        if self.config.flavor == "gym":
            mesh_path = stl_path
        elif self.config.flavor == "ros":
            mesh_path = "package://{package}/{uri}".format(package=package_name, uri=stl_path)
        node_data = {
            "origin": self.origin_(matrix),
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
        
        self.json["robot"]["link"][self.current_link_idx][node] = node_data


    def addPart(self, matrix, stl_path, mass, com, inertia, color, shapes=None, name=''):
        if stl_path is not None:
            # print(stl_path,self.packageName)
            if not self.config.drawCollisions:
                if self.config.useFixedLinks:
                    self._visuals.append(
                        [matrix, self.packageName + stl_path, color])
                elif self.shouldMergeSTLs('visual'):
                    self.mergeSTL(stl_path, matrix, color, mass)
                else:
                    self.addSTL(matrix, stl_path, color, name, 'visual')

            entries = ['collision']
            if self.config.drawCollisions:
                entries.append('visual')
            for entry in entries:

                if shapes is None:
                    # We don't have pure shape, we use the mesh
                    if self.shouldMergeSTLs(entry):
                        self.mergeSTL(stl_path, matrix, color, mass, entry)
                    else:
                        self.addSTL(matrix, stl_path, color, name, entry)
                else:
                    
                    # # Inserting pure shapes in the URDF model
                    for shape in shapes:
                        
                        node_data = {
                            "origin": self.origin_(matrix*shape['transform']),
                            "geometry": []
                        }
                        if shape['type'] == 'cube':
                            node_data["geometry"].append({
                                "box": {
                                    "size": "%.20g %.20g %.20g" % tuple(shape['parameters'])
                                }})
                        if shape['type'] == 'cylinder':
                            node_data["geometry"].append({
                                "cylinder": {
                                    "length": "%.20g" % tuple(shape['parameters'])[0],
                                    "radius": "%.20g" % tuple(shape['parameters'])[1],
                                }})
                        if shape['type'] == 'sphere':
                            node_data["geometry"].append({
                                "sphere": {
                                    "radius": "%.20g" % shape['parameters'],
                                }})
                        
                        if entry == 'visual':
                            node_data["material"] = {
                                "name": name+"_material",
                                "color": {
                                    "rgba": "%.20g %.20g %.20g 1.0" % (color[0], color[1], color[2]),
                                },
                            }
                            
                        self.json["robot"]["link"][self.current_link_idx][entry] = node_data
        
        
                    # # Inserting pure shapes in the URDF model
                    # self.append('<!-- Shapes for '+name+' -->')
                    # for shape in shapes:
                    #     self.append('<'+entry+'>')
                    #     self.append(origin(matrix*shape['transform']))
                    #     self.append('<geometry>')
                    #     if shape['type'] == 'cube':
                    #         self.append('<box size="%.20g %.20g %.20g" />' %
                    #                     tuple(shape['parameters']))
                    #     if shape['type'] == 'cylinder':
                    #         self.append(
                    #             '<cylinder length="%.20g" radius="%.20g" />' % tuple(shape['parameters']))
                    #     if shape['type'] == 'sphere':
                    #         self.append('<sphere radius="%.20g" />' %
                    #                     shape['parameters'])
                    #     self.append('</geometry>')

                    #     if entry == 'visual':
                    #         self.append('<material name="'+name+'_material">')
                    #         self.append('<color rgba="%.20g %.20g %.20g 1.0"/>' %
                    #                     (color[0], color[1], color[2]))
                    #         self.append('</material>')
                    #     self.append('</'+entry+'>')

        self.addLinkDynamics(matrix, mass, com, inertia)

    def addJoint(self, jointType, linkFrom, linkTo, transform, name, jointLimits, zAxis=[0, 0, 1]):
        
        joint = {
            "name": name,
            "type": jointType,
            "parent": {
                "link": linkFrom,
            },
            "child": {
                "link": linkTo,
            },
            "origin": self.origin_(transform),
            "axis": {
                "xyz": "{:.{precision}f} {:.{precision}f} {:.{precision}f}".format(zAxis[0], zAxis[1], zAxis[2], precision=self.floating_precision),
            },
            "limit": {
                "effort": "{:.{precision}f}".format(self.jointMaxEffortFor(name), precision=self.floating_precision),
                "velocity": "{:.{precision}f}".format(self.jointMaxVelocityFor(name), precision=self.floating_precision),
            },
            "joint_properties": {
                "friction": "{:.{precision}f}".format(0, precision=self.floating_precision),
            },
        }
        if jointLimits is not None:
            joint["limit"]["lower"] = jointLimits[0]
            joint["limit"]["upper"] = jointLimits[1]

        self.json["robot"]["joint"].append(joint)

    def finalize(self):
        pass
        # self.append(self.additionalXML)


# class RobotSDF(RobotDescription):
#     def __init__(self, name):
#         super().__init__(name)
#         self.ext = 'sdf'
#         self.relative = False
#         self.json = {
#             "sdf": {
#                 "version": "1.6",
#                 "model": {
#                     "name": self.robotName,
#                     "link": [],
#                     "joint": [],
#                 }
#             }
#         }
#         self.append('<sdf version="1.6">')
#         self.append('<model name="'+self.robotName + '">')
#         pass

#     def addFixedJoint(self, parent, child, matrix, name=None):
#         if name is None:
#             name = parent+'_'+child+'_fixing'

#         self.append('<joint name="'+name+'" type="fixed">')
#         self.append(pose(matrix))
#         self.append('<parent>'+parent+'</parent>')
#         self.append('<child>'+child+'</child>')
#         self.append('</joint>')
#         self.append('')

#     def addDummyLink(self, name, visualMatrix=None, visualSTL=None, visualColor=None):
#         self.append('<link name="'+name+'">')
#         self.append('<pose>0 0 0 0 0 0</pose>')
#         self.append('<inertial>')
#         self.append('<pose>0 0 0 0 0 0</pose>')
#         self.append('<mass>1e-9</mass>')
#         self.append('<inertia>')
#         self.append(
#             '<ixx>0</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0</iyy><iyz>0</iyz><izz>0</izz>')
#         self.append('</inertia>')
#         self.append('</inertial>')
#         if visualSTL is not None:
#             self.addSTL(visualMatrix, visualSTL, visualColor,
#                         name+"_visual", "visual")
#         self.append('</link>')

#     def startLink(self, name, matrix):
#         self._link_name = name
#         self.resetLink()
#         self.append('<link name="'+name+'">')
#         self.append(pose(matrix, name))

#     def endLink(self):
#         mass, com, inertia = self.linkDynamics()

#         for node in ['visual', 'collision']:
#             if self._mesh[node] is not None:
#                 color = self._color / self._color_mass
#                 filename = self._link_name+'_'+node+'.stl'
#                 stl_combine.save_mesh(
#                     self._mesh[node], self.meshDir+'/'+filename)
#                 if self.shouldSimplifySTLs(node):
#                     stl_combine.simplify_stl(
#                         self.meshDir+'/'+filename, self.maxSTLSize)
#                 self.addSTL(np.identity(4), filename, color, self._link_name, 'visual')

#         self.append('<inertial>')
#         self.append('<pose frame="'+self._link_name +
#                     '_frame">%.20g %.20g %.20g 0 0 0</pose>' % (com[0], com[1], com[2]))
#         self.append('<mass>%.20g</mass>' % mass)
#         self.append('<inertia><ixx>%.20g</ixx><ixy>%.20g</ixy><ixz>%.20g</ixz><iyy>%.20g</iyy><iyz>%.20g</iyz><izz>%.20g</izz></inertia>' %
#                     (inertia[0, 0], inertia[0, 1], inertia[0, 2], inertia[1, 1], inertia[1, 2], inertia[2, 2]))
#         self.append('</inertial>')

#         if self.useFixedLinks:
#             self.append(
#                 '<visual><geometry><box><size>0 0 0</size></box></geometry></visual>')

#         self.append('</link>')
#         self.append('')

#         if self.useFixedLinks:
#             n = 0
#             for visual in self._visuals:
#                 n += 1
#                 visual_name = '%s_%d' % (self._link_name, n)
#                 self.addDummyLink(visual_name, visual[0], visual[1], visual[2])
#                 self.addJoint('fixed', self._link_name, visual_name,
#                               np.eye(4), visual_name+'_fixing', None)

#     def addFrame(self, name, matrix):
#         # Adding a dummy link
#         self.addDummyLink(name)

#         # Linking it with last link with a fixed link
#         self.addFixedJoint(self._link_name, name, matrix, name+'_frame')

#     def material(self, color):
#         m = '<material>'
#         m += '<ambient>%.20g %.20g %.20g 1</ambient>' % (color[0], color[1], color[2])
#         m += '<diffuse>%.20g %.20g %.20g 1</diffuse>' % (color[0], color[1], color[2])
#         m += '<specular>0.1 0.1 0.1 1</specular>'
#         m += '<emissive>0 0 0 0</emissive>'
#         m += '</material>'

#         return m

#     def addSTL(self, matrix, stl, color, name, node='visual'):
#         self.append('<'+node+' name="'+name+'_visual">')
#         self.append(pose(matrix))
#         self.append('<geometry>')
#         self.append('<mesh><uri>file://'+stl+'</uri></mesh>')
#         self.append('</geometry>')
#         if node == 'visual':
#             self.append(self.material(color))
#         self.append('</'+node+'>')

#     def addPart(self, matrix, stl, mass, com, inertia, color, shapes=None, name=''):
#         name = self._link_name+'_'+str(self._link_childs)+'_'+name
#         self._link_childs += 1

#         # self.append('<link name="'+name+'">')
#         # self.append(pose(matrix))

#         if stl is not None:
#             if not self.drawCollisions:
#                 if self.useFixedLinks:
#                     self._visuals.append(
#                         [matrix, self.packageName + os.path.basename(stl), color])
#                 elif self.shouldMergeSTLs('visual'):
#                     self.mergeSTL(stl, matrix, color, mass)
#                 else:
#                     self.addSTL(matrix, os.path.basename(
#                         stl), color, name, 'visual')

#             entries = ['collision']
#             if self.drawCollisions:
#                 entries.append('visual')
#             for entry in entries:
#                 if shapes is None:
#                     # We don't have pure shape, we use the mesh
#                     if self.shouldMergeSTLs(entry):
#                         self.mergeSTL(stl, matrix, color, mass, entry)
#                     else:
#                         self.addSTL(matrix, stl, color, name, entry)
#                 else:
#                     # Inserting pure shapes in the URDF model
#                     k = 0
#                     self.append('<!-- Shapes for '+name+' -->')
#                     for shape in shapes:
#                         k += 1
#                         self.append('<'+entry+' name="'+name +
#                                     '_'+entry+'_'+str(k)+'">')
#                         self.append(pose(matrix*shape['transform']))
#                         self.append('<geometry>')
#                         if shape['type'] == 'cube':
#                             self.append('<box><size>%.20g %.20g %.20g</size></box>' %
#                                         tuple(shape['parameters']))
#                         if shape['type'] == 'cylinder':
#                             self.append(
#                                 '<cylinder><length>%.20g</length><radius>%.20g</radius></cylinder>' % tuple(shape['parameters']))
#                         if shape['type'] == 'sphere':
#                             self.append(
#                                 '<sphere><radius>%.20g</radius></sphere>' % shape['parameters'])
#                         self.append('</geometry>')

#                         if entry == 'visual':
#                             self.append(self.material(color))
#                         self.append('</'+entry+'>')

#         self.addLinkDynamics(matrix, mass, com, inertia)

#     def addJoint(self, jointType, linkFrom, linkTo, transform, name, jointLimits, zAxis=[0, 0, 1]):
#         self.append('<joint name="'+name+'" type="'+jointType+'">')
#         self.append(pose(transform))
#         self.append('<parent>'+linkFrom+'</parent>')
#         self.append('<child>'+linkTo+'</child>')
#         self.append('<axis>')
#         self.append('<xyz>%.20g %.20g %.20g</xyz>' % tuple(zAxis))
#         lowerUpperLimits = ''
#         if jointLimits is not None:
#             lowerUpperLimits = '<lower>%.20g</lower><upper>%.20g</upper>' % jointLimits
#         self.append('<limit><effort>%.20g</effort><velocity>%.20g</velocity>%s</limit>' %
#                     (self.jointMaxEffortFor(name), self.jointMaxVelocityFor(name), lowerUpperLimits))
#         self.append('</axis>')
#         self.append('</joint>')
#         self.append('')
#         # print('Joint from: '+linkFrom+' to: '+linkTo+', transform: '+str(transform))

#     def finalize(self):
#         self.append(self.additionalXML)
#         self.append('</model>')
#         self.append('</sdf>')
