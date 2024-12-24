# 这是一个9自由度的系统，每个自由度的电机都有一个目标角度。给你设定一个目标角度向量来表示9个电机的目标角度，而你可以随时查看每个电机的当前角度。请使用PID控制和PD控制分别实现这个控制算法。（Python实现）
import numpy as np
import matplotlib.pyplot as plt
import time

class PIDController:
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp = kp  # 比例系数
        self.ki = ki  # 积分系数
        self.kd = kd  # 微分系数
        self.dt = dt  # 控制周期，时间间隔
        self.integral_error = np.zeros(9)  # 积分误差，初始化为0，对应9个自由度
        self.prev_error = np.zeros(9)  # 上一时刻误差，初始化为0，对应9个自由度

    def update(self, target_angles, current_angles):
        error = target_angles - current_angles
        self.integral_error += error * self.dt
        derivative_error = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral_error + self.kd * derivative_error
        self.prev_error = error
        return output

class PDController:
    def __init__(self, kp, kd, dt=0.01):
        self.kp = kp  # 比例系数
        self.kd = kd  # 微分系数
        self.dt = dt  # 控制周期，时间间隔
        self.prev_error = np.zeros(9)  # 上一时刻误差，初始化为0，对应9个自由度

    def update(self, target_angles, current_angles):
        error = target_angles - current_angles
        derivative_error = (error - self.prev_error) / self.dt
        output = self.kp * error + self.kd * derivative_error
        self.prev_error = error
        return output
    
def simulate_and_plot():
    # 假设目标角度，这里示例为[0, 0, 0, 0, 0, 0, 0, 0, 0]，可根据实际修改
    target_angles = np.zeros(9)
    # 初始当前角度，示例为随机值，实际中从系统获取
    current_angles = np.random.rand(9)

    # PID控制器参数设置，示例参数，需根据实际系统调试
    kp_pid = 0.5
    ki_pid = 0.1
    kd_pid = 0.2
    pid_controller = PIDController(kp_pid, ki_pid, kd_pid)

    # PD控制器参数设置，示例参数，需根据实际系统调试
    kp_pd = 0.6
    kd_pd = 0.3
    pd_controller = PDController(kp_pd, kd_pd)

    max_iterations = 1000  # 最大迭代次数
    tolerance = 0.01  # 角度误差容忍度，认为达到目标的误差范围
    time_steps = []
    target_angles_history_pid = []
    current_angles_history_pid = []
    target_angles_history_pd = []
    current_angles_history_pd = []

    for i in range(max_iterations):
        time_steps.append(i)

        # 使用PID控制更新控制输出并更新角度
        control_output_pid = pid_controller.update(target_angles, current_angles)
        current_angles += control_output_pid * 0.01
        target_angles_history_pid.append(target_angles.copy())
        current_angles_history_pid.append(current_angles.copy())

        # 使用PD控制更新控制输出并更新角度
        control_output_pd = pd_controller.update(target_angles, current_angles)
        current_angles += control_output_pd * 0.01
        target_angles_history_pd.append(target_angles.copy())
        current_angles_history_pd.append(current_angles.copy())

        # 检查是否达到目标角度（所有自由度角度误差都在容忍度内）
        if np.all(np.abs(target_angles - current_angles) < tolerance):
            print("已达到目标角度")
            break

    if i == max_iterations - 1:
        print("在最大迭代次数内未完全达到目标角度")

    # 整理数据用于绘图（将列表中的数组按列堆叠，方便绘制多条曲线）
    target_angles_history_pid = np.column_stack(target_angles_history_pid)
    current_angles_history_pid = np.column_stack(current_angles_history_pid)
    target_angles_history_pd = np.column_stack(target_angles_history_pd)
    current_angles_history_pd = np.column_stack(current_angles_history_pd)

    # 绘制PID控制的目标值和实际值演变图
    plt.figure(figsize=(12, 6))
    time_steps = range(i+1)
    for j in range(9):
        plt.plot(time_steps, target_angles_history_pid[j, :], label=f'Target Angle (PID) - DOF {j + 1}')
        plt.plot(time_steps, current_angles_history_pid[j, :], label=f'Current Angle (PID) - DOF {j + 1}')
    plt.xlabel('Time Step')
    plt.ylabel('Angle')
    plt.title('PID Control - Target and Current Angles Evolution')
    plt.legend()
    plt.show()

    # 绘制PD控制的目标值和实际值演变图
    plt.figure(figsize=(12, 6))
    for j in range(1):
        plt.plot(time_steps, target_angles_history_pd[j, :], label=f'Target Angle (PD) - DOF {j + 1}')
        plt.plot(time_steps, current_angles_history_pd[j, :], label=f'Current Angle (PD) - DOF {j + 1}')
    plt.xlabel('Time Step')
    plt.ylabel('Angle')
    plt.title('PD Control - Target and Current Angles Evolution')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    simulate_and_plot()