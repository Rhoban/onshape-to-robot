import numpy as np
import os
import math
import uuid
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
    urdf = '<origin xyz="%g %g %g" rpy="%g %g %g" />'
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    rpy = rotationMatrixToEulerAngles(matrix)

    return urdf % (x, y, z, rpy[0], rpy[1], rpy[2])


def pose(matrix, frame=''):
    sdf = '<pose>%g %g %g %g %g %g</pose>'
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    rpy = rotationMatrixToEulerAngles(matrix)

    if frame != '':
        sdf = '<frame name="'+frame+'_frame">'+sdf+'</frame>'

    return sdf % (x, y, z, rpy[0], rpy[1], rpy[2])


class RobotDescription(object):
    def __init__(self, name):
        self.drawCollisions = False
        self.relative = True
        self.mergeSTLs = 'no'
        self.mergeSTLsCollisions = False
        self.useFixedLinks = False
        self.simplifySTLs = 'no'
        self.maxSTLSize = 3
        self.xml = ''
        self.jointMaxEffort = 1
        self.jointMaxVelocity = 10
        self.noDynamics = False
        self.packageName = ""
        self.addDummyBaseLink = False
        self.robotName = name
        self.meshDir = None

    def shouldMergeSTLs(self, node):
        return self.mergeSTLs == 'all' or self.mergeSTLs == node

    def shouldSimplifySTLs(self, node):
        return self.simplifySTLs == 'all' or self.simplifySTLs == node

    def append(self, str):
        self.xml += str+"\n"

    def jointMaxEffortFor(self, jointName):
        if isinstance(self.jointMaxEffort, dict):
            if jointName in self.jointMaxEffort:
                return self.jointMaxEffort[jointName]
            else:
                return self.jointMaxEffort['default']
        else:
            return self.jointMaxEffort

    def jointMaxVelocityFor(self, jointName):
        if isinstance(self.jointMaxVelocity, dict):
            if jointName in self.jointMaxVelocity:
                return self.jointMaxVelocity[jointName]
            else:
                return self.jointMaxVelocity['default']
        else:
            return self.jointMaxVelocity

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
        inertia = R.T*I*R

        self._dynamics.append({
            'mass': mass,
            'com': com,
            'inertia': inertia
        })

    def mergeSTL(self, stl, matrix, color, mass, node='visual'):
        if node == 'visual':
            self._color += np.array(color) * mass
            self._color_mass += mass

        m = stl_combine.load_mesh(stl)
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
    def __init__(self, name):
        super().__init__(name)
        self.ext = 'urdf'
        self.append('<robot name="' + self.robotName + '">')
        pass

    def addDummyLink(self, name, visualMatrix=None, visualSTL=None, visualColor=None):
        self.append('<link name="'+name+'">')
        self.append('<inertial>')
        self.append('<origin xyz="0 0 0" rpy="0 0 0" />')
        # XXX: We use a low mass because PyBullet consider mass 0 as world fixed
        if self.noDynamics:
            self.append('<mass value="0" />')
        else:
            self.append('<mass value="1e-9" />')
        self.append(
            '<inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0" />')
        self.append('</inertial>')
        if visualSTL is not None:
            self.addSTL(visualMatrix, visualSTL, visualColor,
                        name+"_visual", 'visual')
        self.append('</link>')

    def addDummyBaseLinkMethod(self, name):
        # adds a dummy base_link for ROS users
        self.append('<link name="base_link"></link>')
        self.append('<joint name="base_link_to_base" type="fixed">')
        self.append('<parent link="base_link"/>')
        self.append('<child link="' + name + '" />')
        self.append('<origin rpy="0.0 0 0" xyz="0 0 0"/>')
        self.append('</joint>')

    def addFixedJoint(self, parent, child, matrix, name=None):
        if name is None:
            name = parent+'_'+child+'_fixing'

        self.append('<joint name="'+name+'" type="floating">')
        self.append(origin(matrix))
        self.append('<parent link="'+parent+'" />')
        self.append('<child link="'+child+'" />')
        self.append('<axis xyz="0 0 0"/>')
        self.append('</joint>')
        self.append('')

    def startLink(self, name, matrix):
        self._link_name = name
        self.resetLink()

        if self.addDummyBaseLink:
            self.addDummyBaseLinkMethod(name)
            self.addDummyBaseLink = False
        self.append('<link name="'+name+'">')

    def endLink(self):
        mass, com, inertia = self.linkDynamics()

        for node in ['visual', 'collision']:
            if self._mesh[node] is not None:
                if node == 'visual' and self._color_mass > 0:
                    color = self._color / self._color_mass
                else:
                    color = [0.5, 0.5, 0.5]

                filename = self._link_name+'_'+node+'.stl'
                stl_combine.save_mesh(
                    self._mesh[node], self.meshDir+'/'+filename)
                if self.shouldSimplifySTLs(node):
                    stl_combine.simplify_stl(self.meshDir+'/'+filename, self.maxSTLSize)
                self.addSTL(np.identity(4), filename, color, self._link_name, node)

        self.append('<inertial>')
        self.append('<origin xyz="%g %g %g" rpy="0 0 0"/>' %
                    (com[0], com[1], com[2]))
        self.append('<mass value="%g" />' % mass)
        self.append('<inertia ixx="%g" ixy="%g"  ixz="%g" iyy="%g" iyz="%g" izz="%g" />' %
                    (inertia[0, 0], inertia[0, 1], inertia[0, 2], inertia[1, 1], inertia[1, 2], inertia[2, 2]))
        self.append('</inertial>')

        if self.useFixedLinks:
            self.append(
                '<visual><geometry><box size="0 0 0" /></geometry></visual>')

        self.append('</link>')
        self.append('')

        if self.useFixedLinks:
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

    def addSTL(self, matrix, stl, color, name, node='visual'):
        self.append('<'+node+'>')
        self.append(origin(matrix))
        self.append('<geometry>')
        self.append('<mesh filename="package://' +
                    self.packageName + stl+'"/>')
        self.append('</geometry>')
        self.append('<material name="'+name+'_material">')
        self.append('<color rgba="%g %g %g 1.0"/>' %
                    (color[0], color[1], color[2]))
        self.append('</material>')
        self.append('</'+node+'>')

    def addPart(self, matrix, stl, mass, com, inertia, color, shapes=None, name=''):
        if stl is not None:
            if not self.drawCollisions:
                if self.useFixedLinks:
                    self._visuals.append(
                        [matrix, self.packageName + os.path.basename(stl), color])
                elif self.shouldMergeSTLs('visual'):
                    self.mergeSTL(stl, matrix, color, mass)
                else:
                    self.addSTL(
                        matrix, os.path.basename(stl), color, name, 'visual')

            entries = ['collision']
            if self.drawCollisions:
                entries.append('visual')
            for entry in entries:

                if shapes is None:
                    # We don't have pure shape, we use the mesh
                    if self.shouldMergeSTLs(entry):
                        self.mergeSTL(stl, matrix, color, mass, entry)
                    else:
                        self.addSTL(matrix, os.path.basename(
                            stl), color, name, entry)
                else:
                    # Inserting pure shapes in the URDF model
                    self.append('<!-- Shapes for '+name+' -->')
                    for shape in shapes:
                        self.append('<'+entry+'>')
                        self.append(origin(matrix*shape['transform']))
                        self.append('<geometry>')
                        if shape['type'] == 'cube':
                            self.append('<box size="%g %g %g" />' %
                                        tuple(shape['parameters']))
                        if shape['type'] == 'cylinder':
                            self.append(
                                '<cylinder length="%g" radius="%g" />' % tuple(shape['parameters']))
                        if shape['type'] == 'sphere':
                            self.append('<sphere radius="%g" />' %
                                        shape['parameters'])
                        self.append('</geometry>')

                        if entry == 'visual':
                            self.append('<material name="'+name+'_material">')
                            self.append('<color rgba="%g %g %g 1.0"/>' %
                                        (color[0], color[1], color[2]))
                            self.append('</material>')
                        self.append('</'+entry+'>')

        self.addLinkDynamics(matrix, mass, com, inertia)

    def addJoint(self, jointType, linkFrom, linkTo, transform, name, jointLimits, zAxis=[0, 0, 1]):
        self.append('<joint name="'+name+'" type="'+jointType+'">')
        self.append(origin(transform))
        self.append('<parent link="'+linkFrom+'" />')
        self.append('<child link="'+linkTo+'" />')
        self.append('<axis xyz="%g %g %g"/>' % tuple(zAxis))
        lowerUpperLimits = ''
        if jointLimits is not None:
            lowerUpperLimits = 'lower="%g" upper="%g"' % jointLimits
        self.append('<limit effort="%g" velocity="%g" %s/>' %
                    (self.jointMaxEffortFor(name), self.jointMaxVelocityFor(name), lowerUpperLimits))
        self.append('<joint_properties friction="0.0"/>')
        self.append('</joint>')
        self.append('')

    def finalize(self):
        self.append(self.additionalXML)
        self.append('</robot>')


class RobotSDF(RobotDescription):
    def __init__(self, name):
        super().__init__(name)
        self.ext = 'sdf'
        self.relative = False
        self.append('<sdf version="1.6">')
        self.append('<model name="'+self.robotName + '">')
        pass

    def addFixedJoint(self, parent, child, matrix, name=None):
        if name is None:
            name = parent+'_'+child+'_fixing'

        self.append('<joint name="'+name+'" type="floating">')
        self.append(pose(matrix))
        self.append('<parent>'+parent+'</parent>')
        self.append('<child>'+child+'</child>')
        self.append('</joint>')
        self.append('')

    def addDummyLink(self, name, visualMatrix=None, visualSTL=None, visualColor=None):
        self.append('<link name="'+name+'">')
        self.append('<pose>0 0 0 0 0 0</pose>')
        self.append('<inertial>')
        self.append('<pose>0 0 0 0 0 0</pose>')
        self.append('<mass>1e-9</mass>')
        self.append('<inertia>')
        self.append(
            '<ixx>0</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0</iyy><iyz>0</iyz><izz>0</izz>')
        self.append('</inertia>')
        self.append('</inertial>')
        if visualSTL is not None:
            self.addSTL(visualMatrix, visualSTL, visualColor,
                        name+"_visual", "visual")
        self.append('</link>')

    def startLink(self, name, matrix):
        self._link_name = name
        self.resetLink()
        self.append('<link name="'+name+'">')
        self.append(pose(matrix, name))

    def endLink(self):
        mass, com, inertia = self.linkDynamics()

        for node in ['visual', 'collision']:
            if self._mesh[node] is not None:
                color = self._color / self._color_mass
                filename = self._link_name+'_'+node+'.stl'
                stl_combine.save_mesh(
                    self._mesh[node], self.meshDir+'/'+filename)
                if self.shouldSimplifySTLs(node):
                    stl_combine.simplify_stl(
                        self.meshDir+'/'+filename, self.maxSTLSize)
                self.addSTL(np.identity(4), filename, color, self._link_name, 'visual')

        self.append('<inertial>')
        self.append('<pose frame="'+self._link_name +
                    '_frame">%g %g %g 0 0 0</pose>' % (com[0], com[1], com[2]))
        self.append('<mass>%g</mass>' % mass)
        self.append('<inertia><ixx>%g</ixx><ixy>%g</ixy><ixz>%g</ixz><iyy>%g</iyy><iyz>%g</iyz><izz>%g</izz></inertia>' %
                    (inertia[0, 0], inertia[0, 1], inertia[0, 2], inertia[1, 1], inertia[1, 2], inertia[2, 2]))
        self.append('</inertial>')

        if self.useFixedLinks:
            self.append(
                '<visual><geometry><box><size>0 0 0</size></box></geometry></visual>')

        self.append('</link>')
        self.append('')

        if self.useFixedLinks:
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

    def material(self, color):
        m = '<material>'
        m += '<ambient>%g %g %g 1</ambient>' % (color[0], color[1], color[2])
        m += '<diffuse>%g %g %g 1</diffuse>' % (color[0], color[1], color[2])
        m += '<specular>0.1 0.1 0.1 1</specular>'
        m += '<emissive>0 0 0 0</emissive>'
        m += '</material>'

        return m

    def addSTL(self, matrix, stl, color, name, node='visual'):
        self.append('<'+node+' name="'+name+'_visual">')
        self.append(pose(matrix))
        self.append('<geometry>')
        self.append('<mesh><uri>file://'+stl+'</uri></mesh>')
        self.append('</geometry>')
        if node == 'visual':
            self.append(self.material(color))
        self.append('</'+node+'>')

    def addPart(self, matrix, stl, mass, com, inertia, color, shapes=None, name=''):
        name = self._link_name+'_'+str(self._link_childs)+'_'+name
        self._link_childs += 1

        # self.append('<link name="'+name+'">')
        # self.append(pose(matrix))

        if stl is not None:
            if not self.drawCollisions:
                if self.useFixedLinks:
                    self._visuals.append(
                        [matrix, self.packageName + os.path.basename(stl), color])
                elif self.shouldMergeSTLs('visual'):
                    self.mergeSTL(stl, matrix, color, mass)
                else:
                    self.addSTL(matrix, os.path.basename(
                        stl), color, name, 'visual')

            entries = ['collision']
            if self.drawCollisions:
                entries.append('visual')
            for entry in entries:
                if shapes is None:
                    # We don't have pure shape, we use the mesh
                    if self.shouldMergeSTLs(entry):
                        self.mergeSTL(stl, matrix, color, mass, entry)
                    else:
                        self.addSTL(matrix, stl, color, name, entry)
                else:
                    # Inserting pure shapes in the URDF model
                    k = 0
                    self.append('<!-- Shapes for '+name+' -->')
                    for shape in shapes:
                        k += 1
                        self.append('<'+entry+' name="'+name +
                                    '_'+entry+'_'+str(k)+'">')
                        self.append(pose(matrix*shape['transform']))
                        self.append('<geometry>')
                        if shape['type'] == 'cube':
                            self.append('<box><size>%g %g %g</size></box>' %
                                        tuple(shape['parameters']))
                        if shape['type'] == 'cylinder':
                            self.append(
                                '<cylinder><length>%g</length><radius>%g</radius></cylinder>' % tuple(shape['parameters']))
                        if shape['type'] == 'sphere':
                            self.append(
                                '<sphere><radius>%g</radius></sphere>' % shape['parameters'])
                        self.append('</geometry>')

                        if entry == 'visual':
                            self.append(self.material(color))
                        self.append('</'+entry+'>')

        self.addLinkDynamics(matrix, mass, com, inertia)

    def addJoint(self, jointType, linkFrom, linkTo, transform, name, jointLimits, zAxis=[0, 0, 1]):
        self.append('<joint name="'+name+'" type="'+jointType+'">')
        self.append(pose(transform))
        self.append('<parent>'+linkFrom+'</parent>')
        self.append('<child>'+linkTo+'</child>')
        self.append('<axis>')
        self.append('<xyz>%g %g %g</xyz>' % tuple(zAxis))
        lowerUpperLimits = ''
        if jointLimits is not None:
            lowerUpperLimits = '<lower>%g</lower><upper>%g</upper>' % jointLimits
        self.append('<limit><effort>%g</effort><velocity>%g</velocity>%s</limit>' %
                    (self.jointMaxEffortFor(name), self.jointMaxVelocityFor(name), lowerUpperLimits))
        self.append('</axis>')
        self.append('</joint>')
        self.append('')
        # print('Joint from: '+linkFrom+' to: '+linkTo+', transform: '+str(transform))

    def finalize(self):
        self.append(self.additionalXML)
        self.append('</model>')
        self.append('</sdf>')
