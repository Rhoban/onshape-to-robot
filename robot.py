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
        self.mergeLinks = False
        self.append('<robot name="onshape">')
        pass

    def append(self, str):
        self.urdf += str+"\n"

    def startLink(self, name):
        self._link_name = name
        self._link_childs = 0
        if self.mergeLinks:
            self.append('<link name="'+name+'">')
        self._dynamics = []

    def endLink(self):
        if self.mergeLinks:
            mass = 0
            com = np.array([0.0]*3)        
            inertia = np.matrix(np.zeros((3,3)))
            identity = np.matrix(np.eye(3))

            for dynamic in self._dynamics:
                mass += dynamic['mass']
                com += dynamic['com']*dynamic['mass']
            com /= mass

            # https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=246
            for dynamic in self._dynamics:
                r = dynamic['com'] - com
                p = np.matrix(r)
                inertia += dynamic['inertia'] + (np.dot(r, r)*identity - p.T*p)*dynamic['mass']

            self.append('<inertial>')
            self.append('<origin xyz="%f %f %f" rpy="0 0 0"/>' % (com[0], com[1], com[2]))
            self.append('<mass value="%f"/>' % mass)
            self.append('<inertia ixx="%f" ixy="%f"  ixz="%f" iyy="%f" iyz="%f" izz="%f" />' %
                (inertia[0, 0], inertia[0, 1], inertia[0, 2], inertia[1, 1], inertia[1, 2], inertia[2, 2]))
            self.append('</inertial>')

            self.append('</link>')
            self.append('')

    def addPart(self, matrix, stl, mass, com, inertia, color, main=False):
        if main:
            name = self._link_name
        else:
            name = self._link_name+'_'+str(self._link_childs)
        self._link_childs += 1

        if not self.mergeLinks:
            self.append('<link name="'+name+'">')

        for entry in ['visual', 'collision']:
            self.append('<'+entry+'>')
            self.append('<geometry>')
            self.append('<mesh filename="package://'+stl+'"/>')
            self.append('</geometry>')
            if self.mergeLinks:
                self.append(origin(matrix))

            self.append('<material name="'+name+'_material">')
            self.append('<color rgba="%f %f %f 1.0"/>' % (color[0], color[1], color[2]))
            self.append('</material>')

            self.append('</'+entry+'>')

        if self.mergeLinks:
            # Inertia
            I = np.matrix(np.reshape(inertia[:9], (3, 3)))
            R = matrix[:3, :3]
            # Expressing COM in the link frame
            com = np.array((matrix*np.matrix([com[0], com[1], com[2], 1]).T).T)[0][:3]
            # Expressing inertia in the link frame
            inertia = R.T*I*R

            self._dynamics.append({
                'mass': mass,
                'com': com,
                'inertia': inertia
            })
        else:
            self.append('<inertial>')
            self.append('<origin xyz="%f %f %f" rpy="0 0 0"/>' % (com[0], com[1], com[2]))
            self.append('<mass value="%f"/>' % mass)
            self.append('<inertia ixx="%f" ixy="%f"  ixz="%f" iyy="%f" iyz="%f" izz="%f" />' %
                (inertia[0], inertia[1], inertia[2], inertia[4], inertia[5], inertia[8]))
            self.append('</inertial>')
            self.append('</link>')

            if not main:
                self.append('<joint name="'+name+'_fixing" type="fixed">')
                self.append(origin(matrix))
                self.append('<parent link="'+self._link_name+'" />')
                self.append('<child link="'+name+'" />')
                self.append('<axis xyz="0 0 0"/>')
                self.append('</joint>')
                self.append('')


    def addJoint(self, linkFrom, linkTo, transform, name):
        self.append('<joint name="'+name+'" type="revolute">')
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