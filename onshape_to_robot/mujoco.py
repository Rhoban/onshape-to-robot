import time
import mujoco
import argparse
import pybullet as p
import mujoco.viewer


def main():
    parser = argparse.ArgumentParser(prog="onshape-to-robot-mujoco")
    parser.add_argument("directory")
    args = parser.parse_args()

    robot_path = args.directory
    if not robot_path.endswith(".xml"):
        robot_path += "/robot.xml"

    model: mujoco.MjModel = mujoco.MjModel.from_xml_path(robot_path)
    data: mujoco.MjData = mujoco.MjData(model)

    viewer = mujoco.viewer.launch_passive(model, data)
    while True:
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
