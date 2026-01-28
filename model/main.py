# import mujoco.viewer
# import time

# def main():
#     model = mujoco.MjModel.from_xml_path('franka_emika_panda/scene.xml')
#     data = mujoco.MjData(model)

#     data.ctrl[:8] = [0.0, -0.569, 0.0, -2.810, 0.0, 3.037, 0.741, 0] 

#     with mujoco.viewer.launch_passive(model, data) as viewer:
#         while viewer.is_running():
#             mujoco.mj_step(model, data)
#             viewer.sync()
#             time.sleep(0.002)
 
# if __name__ == "__main__":
#     main()


import mujoco.viewer
import time
import numpy as np
from panda_control import PandaController 

def main():
    model = mujoco.MjModel.from_xml_path('franka_emika_panda/scene.xml')
    data = mujoco.MjData(model)

    controller = PandaController(model)
    target_qpos = np.array([0.0, -0.569, 0.0, -2.810, 0.0, 3.037, 0.741, 0.04]) 

    with mujoco.viewer.launch_passive(model, data) as viewer:
        mujoco.mj_resetData(model, data)
        while viewer.is_running():
            mujoco.mj_forward(model, data)
            step_start = time.time()
            # # 根据当前状态计算应该施加的力矩
            # tau = controller.compute_torque(data, target_qpos)
            # # 将计算出的力矩写入控制信号
            # data.ctrl[:8] = tau
            data.ctrl[:] = 0.0  # 先全部清零
            data.ctrl[1] = 100.0
            mujoco.mj_step(model, data)
            viewer.sync()
            # 保持实时频率 (500Hz)
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
 
if __name__ == "__main__":
    main()