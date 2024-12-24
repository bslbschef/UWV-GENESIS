import numpy as np
import matplotlib.pyplot as plt


# 模拟恒压供水系统（简单一阶惯性环节）的函数
def system_response(input_signal, prev_output, dt, time_constant):
    """
    根据输入信号和上一时刻的输出，计算当前时刻系统的输出（模拟一阶惯性环节）。
    :param input_signal: 当前时刻的输入信号
    :param prev_output: 上一时刻系统的输出
    :param dt: 采样时间间隔
    :param time_constant: 系统的时间常数
    :return: 当前时刻系统的输出
    """
    return (prev_output + (dt / time_constant) * (input_signal - prev_output))


# PID控制算法函数
def pid_control(error, prev_error, integral, dt, kp, ki, kd):
    """
    执行PID控制算法，计算控制信号。
    :param error: 当前时刻的误差（设定值 - 实际值）
    :param prev_error: 上一时刻的误差
    :param integral: 误差积分项
    :param dt: 采样时间间隔
    :param kp: 比例增益
    :param ki: 积分增益
    :param kd: 微分增益
    :return: 控制信号，更新后的积分项
    """
    integral += error * dt
    derivative = (error - prev_error) / dt
    control_signal = kp * error + ki * integral + kd * derivative
    return control_signal, integral


# 主函数，进行模拟和绘图
def main():
    # 系统参数
    time_constant = 5  # 供水系统的时间常数
    dt = 0.1  # 采样时间间隔（单位：秒）
    total_time = 100  # 总模拟时间（单位：秒）
    num_steps = int(total_time / dt)  # 模拟步数

    # PID控制器参数（控制增益）
    kp = 0.5  # 比例增益
    ki = 0.1  # 积分增益
    kd = 0.05  # 微分增益

    # 初始化数组
    setpoint = np.ones(num_steps) * 1  # 设定值（期望供水压力），这里设为常数1
    output = np.zeros(num_steps)  # 实际输出值（实际供水压力）
    error = np.zeros(num_steps)  # 误差
    prev_error = 0  # 上一时刻误差，初始化为0
    integral = 0  # 误差积分项，初始化为0
    control_signal_history = np.zeros(num_steps)  # 控制信号历史记录

    # 模拟系统运行
    for i in range(num_steps):
        if i == 0:
            output[i] = 0  # 初始输出设为0
            error[i] = setpoint[i] - output[i]
            continue

        error[i] = setpoint[i] - output[i - 1]
        control_signal, integral = pid_control(error[i], prev_error, integral, dt, kp, ki, kd)
        control_signal_history[i] = control_signal
        output[i] = system_response(control_signal, output[i - 1], dt, time_constant)
        prev_error = error[i]

    # 绘制结果
    plt.plot(np.arange(0, total_time, dt), setpoint, label='setting', color='r')
    plt.plot(np.arange(0, total_time, dt), output, label='real value', color='b')
    plt.xlabel('time, s')
    plt.ylabel('water pressure')
    plt.title('PID Control Simulation of Constant Pressure Water Supply System')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()