import mujoco.viewer
import time

def main():
    model = mujoco.MjModel.from_xml_path('franka_emika_panda/scene.xml')
    data = mujoco.MjData(model)

    data.ctrl[:8] = [0.0, -0.569, 0.0, -2.810, 0.0, 3.037, 0.741, 0] 

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.002)
 
if __name__ == "__main__":
    main()