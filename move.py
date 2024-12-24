import numpy as np

import genesis as gs

########################## 初始化 ##########################
gs.init(backend=gs.gpu)

########################## 创建场景 ##########################
scene = gs.Scene(
    viewer_options = gs.options.ViewerOptions(
        camera_pos    = (0, -3.5, 2.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 30,
        res           = (960, 640),
        max_FPS       = 60,
    ),
    sim_options = gs.options.SimOptions(
        dt = 0.01,
    ),
    show_viewer = True,
)

########################## 实体 ##########################
plane = scene.add_entity(
    gs.morphs.Plane(),
)
franka = scene.add_entity(
    gs.morphs.MJCF(
        file  = 'xml/franka_emika_panda/panda.xml',
    ),
)
########################## 构建 ##########################
scene.build()

jnt_names = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
    'joint7',
    'finger_joint1',
    'finger_joint2',
]
# 使用.dof_idx_local来获取相对于机器人实体本身的局部自由度索引
# 你也可以使用joint.dof_idx来访问每个关节在场景中的全局自由度索引
dofs_idx = [franka.get_joint(name).dof_idx_local for name in jnt_names]

############ 可选：设置控制增益 ### #########（增益即放大缩小的倍数！）
# 设置位置增益
franka.set_dofs_kp(
    kp             = np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
    dofs_idx_local = dofs_idx,
)
# 设置速度增益
franka.set_dofs_kv(
    kv             = np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
    dofs_idx_local = dofs_idx,
)
# 设置安全的力范围
# “安全的力范围” 是指在机械臂（这里是 Franka 机械臂）的各个自由度（dofs）上允许施加的力的安全区间。它定义了下限（lower）和上限（upper）。这个范围的设定是为了确保机械臂在操作过程中的安全性、稳定性和避免对自身以及周围环境造成损害。
franka.set_dofs_force_range(
    lower          = np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
    upper          = np.array([ 87,  87,  87,  87,  12,  12,  12,  100,  100]),
    dofs_idx_local = dofs_idx,
)
# 我们先看看如何手动设置机器人的配置，而不是使用物理上真实的PD控制器。这些API可以在不遵守物理规律的情况下突然改变机器人的状态
# 硬重置
# 硬重置通常是指对某个设备、系统或者对象进行一种较为强制、彻底的复位操作，使其回到一个初始的、预定义的状态。
#   这种操作往往会直接覆盖当前的状态，不考虑设备或系统中正在进行的一些临时配置、缓存数据或者中间状态等，直接将相关参数、状态等恢复到默认或者指定的基础设定。
for i in range(150):
    if i < 50:
        franka.set_dofs_position(np.array([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04]), dofs_idx)
    elif i < 100:
        franka.set_dofs_position(np.array([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04]), dofs_idx)
    else:
        franka.set_dofs_position(np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]), dofs_idx)

    scene.step()

# PD控制
for i in range(1250):
    if i == 0:
        franka.control_dofs_position(
            np.array([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04]),
            dofs_idx,
        )
    elif i == 250:
        franka.control_dofs_position(
            np.array([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04]),
            dofs_idx,
        )
    elif i == 500:
        franka.control_dofs_position(
            np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]),
            dofs_idx,
        )
    elif i == 750:
        # 用速度控制第一个自由度，其余的用位置控制
        franka.control_dofs_position(
            np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])[1:],
            dofs_idx[1:],
        )
        franka.control_dofs_velocity(
            np.array([1.0, 0, 0, 0, 0, 0, 0, 0, 0])[:1],
            dofs_idx[:1],
        )
    elif i == 1000:
        franka.control_dofs_force(
            np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]),
            dofs_idx,
        )
    # 这是根据给定控制命令计算的控制力
    # 如果使用力控制，它与给定的控制命令相同
    print('控制力:', franka.get_dofs_control_force(dofs_idx))

    # 这是自由度实际经历的力
    print('内部力:', franka.get_dofs_force(dofs_idx))

    scene.step()