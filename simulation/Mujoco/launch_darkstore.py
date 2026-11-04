#!/usr/bin/env python3
"""Launch the Darkstore MuJoCo environment in the interactive viewer.

Usage:
    python3 launch_darkstore.py [path/to/scene.xml]

If no path is given, defaults to darkstore_env.xml in this script's directory.
"""
import os
import sys
import time

import mujoco
import mujoco.viewer


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_scene = os.path.join(script_dir, "darkstore_env.xml")
    scene_path = sys.argv[1] if len(sys.argv) > 1 else default_scene

    if not os.path.isfile(scene_path):
        print(f"Scene file not found: {scene_path}")
        sys.exit(1)

    print(f"Loading MuJoCo model: {scene_path}")
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Viewer launched. Close the window to exit.")
        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(model, data)
            viewer.sync()

            # keep roughly real-time
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
