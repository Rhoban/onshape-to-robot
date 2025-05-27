def main():
    import math
    import sys
    import os
    import time
    import argparse
    import pybullet as p
    from .simulation import Simulation

    parser = argparse.ArgumentParser(prog="onshape-to-robot-bullet")
    parser.add_argument("-f", "--fixed", action="store_true")
    parser.add_argument("-n", "--no-self-collisions", action="store_true")
    parser.add_argument("-x", "--x", type=float, default=0)
    parser.add_argument("-y", "--y", type=float, default=0)
    parser.add_argument("-z", "--z", type=float, default=0)
    parser.add_argument("directory")
    args = parser.parse_args()

    robotPath = args.directory
    if not robotPath.endswith(".urdf"):
        robotPath += "/robot.urdf"

    sim = Simulation(
        robotPath,
        gui=True,
        panels=True,
        fixed=args.fixed,
        ignore_self_collisions=args.no_self_collisions,
    )
    pos, rpy = sim.getRobotPose()
    _, orn = p.getBasePositionAndOrientation(sim.robot)
    sim.setRobotPose([pos[0] + args.x, pos[1] + args.y, pos[2] + args.z], orn)

    controls = {}
    for name in sim.getJoints():
        if name.endswith("_speed"):
            controls[name] = p.addUserDebugParameter(name, -math.pi * 3, math.pi * 3, 0)
        else:
            infos = sim.getJointsInfos(name)
            low = -math.pi
            high = math.pi
            if "lowerLimit" in infos:
                low = infos["lowerLimit"]
            if "upperLimit" in infos:
                high = infos["upperLimit"]
            controls[name] = p.addUserDebugParameter(name, low, high, 0)

    lastPrint = 0
    while True:
        targets = {}
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        sim.setJoints(targets)

        if time.time() - lastPrint > 0.05:
            lastPrint = time.time()
            os.system("clear")
            frames = sim.getFrames()
            for frame in frames:
                print(frame)
                print("- x=%f\ty=%f\tz=%f" % frames[frame][0])
                print("- r=%f\tp=%f\ty=%f" % frames[frame][1])
                print("")
            print("Center of mass:")
            print(sim.getCenterOfMassPosition())

        sim.tick()


if __name__ == "__main__":
    main()
