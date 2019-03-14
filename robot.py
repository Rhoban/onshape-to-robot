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
        self.urdf = '<robot name="onshape">'
        pass

    def addLink(self, name):
        name = name.split(' ')[0].lower()
        print('Part name: '+name)
        return name

    def addPart(self, parent, name, matrix):
        print(origin(matrix))
        pass

    def addJoint(self, linkFrom, linkTo, transform):
        print('Joint from: '+linkFrom+' to: '+linkTo+', transform: '+str(transform))