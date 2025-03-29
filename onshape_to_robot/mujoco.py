def main():
    import time
    import mujoco
    import argparse
    import mujoco.viewer

    parser = argparse.ArgumentParser(prog="onshape-to-robot-mujoco")
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--x", type=float, default=0)
    parser.add_argument("--y", type=float, default=0)
    parser.add_argument("--z", type=float, default=0.5)
    parser.add_argument("directory")
    args = parser.parse_args()

    robot_path = args.directory
    if not robot_path.endswith(".xml"):
        robot_path += "/scene.xml"

    model: mujoco.MjModel = mujoco.MjModel.from_xml_path(robot_path)
    data: mujoco.MjData = mujoco.MjData(model)

    # Check for root existence
    if len(model.jnt_type) and model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE:
        data.qpos[:3] = [args.x, args.y, args.z]

    viewer = mujoco.viewer.launch_passive(model, data)
    while viewer.is_running():
        step_start = time.time()
        mujoco.mj_step(model, data)
        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
