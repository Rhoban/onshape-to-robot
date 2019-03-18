import numpy as np
import math
import uuid
 
def rotationMatrixToEulerAngles(R) :     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

def origin(matrix):
    urdf = '<origin xyz="%f %f %f" rpy="%f %f %f" />'
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    rpy = rotationMatrixToEulerAngles(matrix)

    return urdf % (x, y, z, rpy[0], rpy[1], rpy[2])

class Robot:
    def __init__(self):
        self.drawCollisions = False
        self.urdf = ''
        self.append('<robot name="onshape">')
        pass

    def append(self, str):
        self.urdf += str+"\n"

    def addDummyLink(self, name):
        self.append('<link name="'+name+'">')
        self.append('<inertial>')
        self.append('<origin xyz="0 0 0" rpy="0 0 0" />')
        # XXX: We use a low mass because PyBullet consider mass 0 as world fixed
        self.append('<mass value="1e-9" />')
        self.append('<inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0" />')
        self.append('</inertial>')
        self.append('</link>')

    def addFixedJoint(self, parent, child, matrix, name=None):
        if name is None:
            name = parent+'_'+child+'_fixing'

        self.append('<joint name="'+name+'" type="fixed">')
        self.append(origin(matrix))
        self.append('<parent link="'+parent+'" />')
        self.append('<child link="'+child+'" />')
        self.append('<axis xyz="0 0 0"/>')
        self.append('</joint>')
        self.append('')

    def startLink(self, name):
        self._link_name = name
        self._link_childs = 0
        self.addDummyLink(name)

    def endLink(self):
        pass

    def addFrame(self, name, matrix):
        # Adding a dummy link
        self.addDummyLink(name)

        # Linking it with last link with a fixed link
        self.addFixedJoint(self._link_name, name, matrix, name+'_frame')

    def addPart(self, matrix, stl, mass, com, inertia, color, shapes=None):
        name = self._link_name+'_'+str(self._link_childs)
        self._link_childs += 1

        self.append('<link name="'+name+'">')

        if not self.drawCollisions:
            # Visual
            self.append('<visual>')
            self.append('<geometry>')
            self.append('<mesh filename="package://'+stl+'"/>')
            self.append('</geometry>')
            self.append('<material name="'+name+'_material">')
            self.append('<color rgba="%f %f %f 1.0"/>' % (color[0], color[1], color[2]))
            self.append('</material>')
            self.append('</visual>')

        entries = ['collision']
        if self.drawCollisions:
            entries.append('visual')
        for entry in entries:
            
            if shapes is None:
                # We don't have pure shape, we use the mesh
                self.append('<'+entry+'>')
                self.append('<geometry>')
                self.append('<mesh filename="package://'+stl+'"/>')
                self.append('</geometry>')
                self.append('<material name="'+name+'_material">')
                self.append('<color rgba="%f %f %f 1.0"/>' % (color[0], color[1], color[2]))
                self.append('</material>')
                self.append('</'+entry+'>')
            else:
                # Inserting pure shapes in the URDF model
                for shape in shapes:
                        self.append('<'+entry+'>')
                        self.append(origin(shape['transform']))
                        self.append('<geometry>')
                        if shape['type'] == 'cube':
                            self.append('<box size="%f %f %f" />' % tuple(shape['parameters']))
                        if shape['type'] == 'cylinder':
                            self.append('<cylinder length="%f" radius="%f" />' % tuple(shape['parameters']))
                        if shape['type'] == 'sphere':
                            self.append('<sphere radius="%f" />' % shape['parameters'])
                        self.append('</geometry>')

                        self.append('<material name="'+name+'_material">')
                        self.append('<color rgba="%f %f %f 1.0"/>' % (color[0], color[1], color[2]))
                        self.append('</material>')
                        self.append('</'+entry+'>')

        self.append('<inertial>')
        self.append('<origin xyz="%f %f %f" rpy="0 0 0"/>' % (com[0], com[1], com[2]))
        self.append('<mass value="%f"/>' % mass)
        self.append('<inertia ixx="%f" ixy="%f"  ixz="%f" iyy="%f" iyz="%f" izz="%f" />' %
            (inertia[0], inertia[1], inertia[2], inertia[4], inertia[5], inertia[8]))
        self.append('</inertial>')
        self.append('</link>')

        self.addFixedJoint(self._link_name, name, matrix)


    def addJoint(self, linkFrom, linkTo, transform, name, zAxis=[0,0,1]):
        self.append('<joint name="'+name+'" type="revolute">')
        self.append(origin(transform))
        self.append('<parent link="'+linkFrom+'" />')
        self.append('<child link="'+linkTo+'" />')
        self.append('<axis xyz="%f %f %f"/>' % tuple(zAxis))
        self.append('<limit effort="0.5" velocity="12.5664" />')
        self.append('<joint_properties friction="0.0"/>')
        self.append('</joint>')
        self.append('')
        # print('Joint from: '+linkFrom+' to: '+linkTo+', transform: '+str(transform))
    
    def finalize(self):
        self.append('</robot>')