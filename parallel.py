import genesis as gs
import torch

########################## 初始化 ##########################
gs.init(backend=gs.gpu)

########################## 创建场景 ##########################
scene = gs.Scene(
    show_viewer    = True,
    viewer_options = gs.options.ViewerOptions(
        camera_pos    = (3.5, -1.0, 2.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 40,
    ),
    rigid_options = gs.options.RigidOptions(
        dt                = 0.01,
    ),
)

########################## 实体 ##########################
plane = scene.add_entity(
    gs.morphs.Plane(),
    # gs.morphs.Terrain()
)

franka = scene.add_entity(
    gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
)

########################## 构建 ##########################

# 创建20个并行环境
B = 20
scene.build(n_envs=B, env_spacing=(1.0, 1.0))

# 控制所有机器人
franka.control_dofs_position(
    torch.tile(
        torch.tensor([0, 0, 0, -1.0, 0, 0, 0, 0.02, 0.02], device=gs.device), (B, 1)
    ),
)

for i in range(100000):
    scene.step()