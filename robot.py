import numpy as np
import math
 
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
        self.urdf = ''
        self.append('<robot name="onshape">')
        pass

    def append(self, str):
        self.urdf += str+"\n"

    def addLink(self, name):
        name = name.split(' ')[0].lower()
        print('Part name: '+name)
        self.append('<link name="'+name+'">')

        self.append('<inertial>')
        self.append('<origin xyz="0 0 0" rpy="0 0 0"/>')
        self.append('<mass value="0.00000001"/>')
        self.append('<inertia ixx="0.00001" ixy="0"  ixz="0" iyy="0.0001" iyz="0" izz="0.0001" />')
        self.append('</inertial>')

        self.append('</link>')
        self.append('')
        return name

    def addPart(self, parent, name, matrix, stl, mass, com, inertia):
        self.append('<link name="'+name+'">')
        for entry in ['visual', 'collision']:
            self.append('<'+entry+'>')
            self.append('<geometry>')
            self.append('<mesh filename="package://'+stl+'"/>')
            self.append('</geometry>')
            self.append('<origin xyz="0 0 0" rpy="0 0 0"/>')
            self.append('</'+entry+'>')

        self.append('<inertial>')
        self.append('<origin xyz="%f %f %f" rpy="0 0 0"/>' % (com[0], com[1], com[2]))
        self.append('<mass value="%f"/>' % mass)
        self.append('<inertia ixx="%f" ixy="%f"  ixz="%f" iyy="%f" iyz="%f" izz="%f" />' %
            (inertia[0], inertia[1], inertia[2], inertia[4], inertia[5], inertia[8]))
        self.append('</inertial>')

        self.append('</link>')

        self.append('<joint name="'+name+'_fixing" type="fixed">')
        self.append(origin(matrix))
        self.append('<parent link="'+parent+'"/>')
        self.append('<child link="'+name+'"/>')
        self.append('<axis xyz="0 0 0"/>')
        self.append('</joint>')
        self.append('')
        pass

    def addJoint(self, linkFrom, linkTo, transform):
        self.append('<joint name="'+linkFrom+'_'+linkTo+'" type="revolute">')
        self.append(origin(transform))
        self.append('<parent link="'+linkFrom+'" />')
        self.append('<child link="'+linkTo+'" />')
        self.append('<axis xyz="0 0 1"/>')
        self.append('<limit effort="0.5" velocity="12.5664" />')
        self.append('<joint_properties friction="0.0"/>')
        self.append('</joint>')
        self.append('')
        # print('Joint from: '+linkFrom+' to: '+linkTo+', transform: '+str(transform))
    
    def finalize(self):
        self.append('</robot>')