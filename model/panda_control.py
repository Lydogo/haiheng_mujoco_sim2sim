import numpy as np
import mujoco

class PandaController:
    def __init__(self, model):
        self.model = model
        # 7个关节 + 1个夹爪的 PD 参数
        self.kp = np.array([4500.0, 4500.0, 3500.0, 3500.0, 2000.0, 2000.0, 2000.0, 1000.0])
        self.kd = np.array([450.0, 450.0, 350.0, 350.0, 200.0, 200.0, 200.0, 50.0])
        self.n_actuators = len(self.kp)
        
        # 预先获取 actuator 对应的 qpos 索引，防止切片出错
        # 这能确保即便 qpos 长度很长，我们也能准确拿到对应关节的数据
        self.qpos_indices = []
        for i in range(self.n_actuators):
            joint_id = self.model.actuator_trnid[i, 0] # 获取驱动器对应的关节ID
            self.qpos_indices.append(self.model.jnt_qposadr[joint_id])

        self.print_count = 0


    def compute_torque(self, data, target_pos):
        # 1. 获取当前状态
        current_pos = np.array([data.qpos[idx] for idx in self.qpos_indices])
        current_vel = np.array([data.qvel[idx] for idx in self.qpos_indices])

        # 2. 计算 PD 力矩
        error = target_pos - current_pos
        pd_torque = self.kp * error + self.kd * (0 - current_vel)

        # 3. 获取重力偏置补偿
        full_bias_forces = data.qfrc_bias
        bias_comp = np.array([full_bias_forces[self.model.actuator_trnid[i, 0]] 
                             for i in range(self.n_actuators)])

        # 4. 计算总力矩
        total_torque = -(pd_torque + bias_comp)

        # --- 打印 8 个关节的详细信息 ---
        self.print_count += 1
        if self.print_count % 100 == 0:
            print("\n" + "="*50)
            print(f"{'Joint':<10} | {'Error':<10} | {'PD_Torque':<12} | {'Bias_Comp':<12} | {'Total':<10}")
            print("-" * 50)
            for i in range(self.n_actuators):
                print(f"J{i+1:<9} | {error[i]:<10.3f} | {pd_torque[i]:<12.2f} | {bias_comp[i]:<12.2f} | {total_torque[i]:<10.2f}")
            print("="*50)

        # 暂时注释掉限幅，观察真实需求
        # ctrl_limits = self.model.actuator_forcerange[:self.n_actuators]
        # total_torque = np.clip(total_torque, ctrl_limits[:, 0], ctrl_limits[:, 1])

        return total_torque