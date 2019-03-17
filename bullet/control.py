import math


def jointsPosition(t):
    joints = [0]*2

    # Envoi d'un sinus dans tous les joints
    for joint in range(len(joints)):
        joints[joint] = math.sin(t)

    return joints
