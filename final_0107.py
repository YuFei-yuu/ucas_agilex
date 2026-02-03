# ==============================================================================
# 1. 核心系统库
# ==============================================================================
import socket
import struct
import cv2
import numpy as np
import torch
import io
import os
from PIL import Image

# ==============================================================================
# 2. Habitat & Hydra 基础库
# ==============================================================================
import habitat
from habitat_baselines.common.baseline_registry import baseline_registry
from habitat_baselines.config.default import get_config
from habitat_baselines.common.tensor_dict import TensorDict
from gym import spaces

# ==============================================================================
# 3. 关键修复：手动注册 VLFM 所有模块
# (必须显式导入这些文件，@register 装饰器才会生效)
# ==============================================================================

# [A] 注册 Measurements (解决 "Could not load traveled_stairs")
import vlfm.measurements.traveled_stairs  

# [B] 注册 Policies (解决 "NoneType object has no attribute from_config")
# 你的 yaml 里写的是 name: "HabitatITMPolicyV2"，它就在这里面定义
import vlfm.policy.habitat_policies   
import vlfm.policy.reality_policies   # 如果用到真机特定策略也需要

# [C] 注册 Mapping (解决地图相关报错)
import vlfm.mapping.frontier_map          
import vlfm.mapping.obstacle_map          
import vlfm.mapping.value_map             

# ==============================================================================
# 下面接你的 load_vlfm_policy 函数和主循环...
# ==============================================================================

# =========================
# ⚙️ 配置区域
# =========================
SERVER_IP = "0.0.0.0"
SERVER_PORT = 9000
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
VIS_DIR = "vlfm_vis"
os.makedirs(VIS_DIR, exist_ok=True)

# 目标分辨率 (VLFM 默认训练分辨率)
TARGET_SIZE = (640, 480) 
TARGET_OBJECT_ID = 2  

# =========================
# 🛠️ 1. 复刻 VLFM Resize 逻辑
# =========================
# 来源: vlfm/obs_transformers/utils.py
def image_resize(
    img: torch.Tensor,
    size: tuple,
    channels_last: bool = False,
    interpolation_mode: str = "area",
) -> torch.Tensor:
    """
    完全一致的 PyTorch Resize 实现
    """
    img = torch.as_tensor(img)
    no_batch_dim = len(img.shape) == 3
    if len(img.shape) < 3 or len(img.shape) > 5:
        raise NotImplementedError()
    if no_batch_dim:
        img = img.unsqueeze(0)  # Adds a batch dimension (N, H, W, C)
    
    # 调整维度顺序以适应 torch.interpolate (需要 N, C, H, W)
    if channels_last:
        if len(img.shape) == 4:
            # NHWC -> NCHW
            img = img.permute(0, 3, 1, 2)
        else:
            # NDHWC -> NDCHW
            img = img.permute(0, 1, 4, 2, 3)

    # 核心操作：Float 转换 -> Interpolate -> 原类型转换(在Policy外通常保持Float)
    # 注意：源码中最后 .to(dtype=img.dtype) 会转回原类型
    # 但在推理时，我们希望尽快转成 Float32 进入网络，所以这里我们略微修改，
    # 既然输入已经是 Float32 (我们自己转的)，输出也是 Float32。
    img = torch.nn.functional.interpolate(img.float(), size=size, mode=interpolation_mode)
    
    if channels_last:
        if len(img.shape) == 4:
            # NCHW -> NHWC
            img = img.permute(0, 2, 3, 1)
        else:
            # NDCHW -> NDHWC
            img = img.permute(0, 1, 3, 4, 2)
    if no_batch_dim:
        img = img.squeeze(dim=0)  # Removes the batch dimension
    return img

def preprocess_obs_vlfm_style(rgb_np, depth_mm_np, device):
    """
    完全遵循 VLFM pipeline 的预处理
    """
    # 1. 转为 Tensor
    # RGB: [H, W, 3] -> Float
    rgb_tensor = torch.from_numpy(rgb_np).float().to(device)
    
    # Depth: [H, W] -> [H, W, 1] -> Float Meters
    depth_m_tensor = torch.from_numpy(depth_mm_np).float().to(device) / 1000.0
    depth_m_tensor = depth_m_tensor.unsqueeze(-1) # [H, W, 1]

    # 2. 执行 Resize (使用 "area" 插值，这非常关键！)
    # 对应源码 resize.py: interpolation_mode = "area"
    rgb_resized = image_resize(
        rgb_tensor, 
        TARGET_SIZE, 
        channels_last=True, 
        interpolation_mode="area"
    )
    
    depth_resized = image_resize(
        depth_m_tensor, 
        TARGET_SIZE, 
        channels_last=True, 
        interpolation_mode="area"
    )

    # 3. 归一化 / 类型转换
    # RGB 通常 Policy 内部会处理 (或者需要 /255.0)，VLFM Policy 内部有 NormalizeVisualInputs 吗？
    # 查看 pointnav_policy.py: normalize_visual_inputs=False (如果 habitat_version 0.1.5)
    # 但 VLFM 的 yaml 配置里可能有。
    # 安全起见：保持 uint8 范围的 float (0-255) 或者归一化。
    # 大多数 Habitat Policy 期望 RGB 是 [0, 255] 的 uint8 或者 float。
    # Depth 必须归一化到 [0, 1]。
    
    # Habitat 标准 Depth 归一化: clamp(d, min, max) -> normalize
    # 假设 min=0, max=5.0 (和之前 config 一致)
    depth_norm = torch.clamp(depth_resized, 0.0, 5.0) / 5.0

    return rgb_resized.byte(), depth_norm # RGB 转回 byte 以匹配 Space 定义, Depth 保持 Float

# =========================
# 2. 加载 VLFM Policy (不变)
# =========================
def load_vlfm_policy():
    print(f"🔥 Initializing VLFM Full System on {DEVICE}...")
    config_path = "/home/sxz/sjd/vlfm/config/experiments/vlfm_objectnav_hm3d.yaml"
    config = get_config(config_path, [
        "habitat_baselines.rl.policy.name=HabitatITMPolicyV2",
        # 即使这里写了 size，我们现在的 resize 是手动做的，
        # 所以这里的配置主要是为了让 Policy 初始化正确的卷积层
        "habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.width=224", 
        "habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.height=224",

    ])
    
    policy_cls = baseline_registry.get_policy(config.habitat_baselines.rl.policy.name)
    print("fov, camera_height:", config.habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.hfov, config.habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.position)
    obs_space = spaces.Dict({
        "rgb": spaces.Box(low=0, high=255, shape=(224, 224, 3), dtype=np.uint8),
        "depth": spaces.Box(low=0.0, high=1.0, shape=(224, 224, 1), dtype=np.float32),
        "objectgoal": spaces.Box(low=0, high=100, shape=(1,), dtype=np.int64),
        "gps": spaces.Box(low=-np.inf, high=np.inf, shape=(2,), dtype=np.float32),
        "compass": spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
    })
    action_space = spaces.Discrete(4)
    
    policy = policy_cls.from_config(config, obs_space, action_space)
    policy.to(DEVICE)
    policy.eval()
    print("✅ VLFM Policy loaded!")
    return policy

policy = load_vlfm_policy()

# =========================
# 3. 辅助函数
# =========================
def recv_all(conn, size: int) -> bytes:
    data = b""
    while len(data) < size:
        chunk = conn.recv(size - len(data))
        if not chunk: return b""
        data += chunk
    return data

def get_maps_from_policy(policy_instance):
    """
    修正后的地图提取函数：直接访问 VLFM Policy 的 _obstacle_map 和 _value_map
    """
    obs_bgr, val_bgr = None, None
    
    # 辅助函数：尝试从对象中获取地图
    def extract_from_obj(obj):
        o_bgr, v_bgr = None, None
        # 1. 提取障碍物地图 (Obstacle Map)
        if hasattr(obj, "_obstacle_map") and obj._obstacle_map is not None:
            try:
                # visualize() 返回 BGR 图像
                o_bgr = obj._obstacle_map.visualize()
            except Exception as e:
                print(f"⚠️ Obstacle Map visualize failed: {e}")

        # 2. 提取价值地图 (Value Map)
        if hasattr(obj, "_value_map") and obj._value_map is not None:
            try:
                # ValueMap 可视化通常需要 ObstacleMap 来做遮罩(mask)
                obs_map_ref = getattr(obj, "_obstacle_map", None)
                v_bgr = obj._value_map.visualize(obstacle_map=obs_map_ref)
            except Exception as e:
                print(f"⚠️ Value Map visualize failed: {e}")
        return o_bgr, v_bgr

    try:
        # A. 尝试直接从 policy 实例获取
        obs_bgr, val_bgr = extract_from_obj(policy_instance)

        # B. 如果没找到，可能被 wrap 了一层 (例如 ActorCritic wrapper)，尝试 policy.policy
        if obs_bgr is None and val_bgr is None:
            if hasattr(policy_instance, "policy"):
                obs_bgr, val_bgr = extract_from_obj(policy_instance.policy)
                
    except Exception as e:
        print(f"⚠️ Map extraction global error: {e}")
    
    return obs_bgr, val_bgr
# =========================
# 4. 主循环
# =========================
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((SERVER_IP, SERVER_PORT))
sock.listen(1)
print(f"🚀 Server listening on {SERVER_PORT}...")

counter=0
while True:
    conn, addr = sock.accept()
    print(f"✅ Robot connected: {addr}")
    
    # 初始化状态
    rnn_hidden_states = torch.zeros(1, policy.net.num_recurrent_layers, 512, device=DEVICE) if hasattr(policy, "net") else torch.zeros(1, 4, 512, device=DEVICE)
    prev_actions = torch.zeros(1, 1, device=DEVICE, dtype=torch.long)
    masks = torch.zeros(1, 1, device=DEVICE, dtype=torch.bool)
    object_goal_tensor = torch.tensor([[TARGET_OBJECT_ID]], device=DEVICE, dtype=torch.long)

    try:
        while True:
            
            # --- 接收 RGB (Native 640x480) ---
            header = recv_all(conn, 4)
            if not header: break
            rgb_size = struct.unpack("I", header)[0]
            rgb_bytes = recv_all(conn, rgb_size)
            rgb_np = cv2.imdecode(np.frombuffer(rgb_bytes, np.uint8), cv2.IMREAD_COLOR)
            
            # --- 接收 Depth (Native 640x480) ---
            header = recv_all(conn, 4)
            if not header: break
            depth_size = struct.unpack("I", header)[0]
            depth_bytes = recv_all(conn, depth_size)
            depth_img = Image.open(io.BytesIO(depth_bytes))
            depth_mm_np = np.array(depth_img).astype(np.float32)

            # --- 接收 State ---
            state_bytes = recv_all(conn, 12)
            if not state_bytes: break
            x, y, yaw = struct.unpack("fff", state_bytes)
            # yaw = - yaw
            # ==========================================
            # ⚡️ 核心修改：使用 VLFM 原生预处理
            # ==========================================
            rgb_tensor, depth_tensor = preprocess_obs_vlfm_style(rgb_np, depth_mm_np, DEVICE)

            # 构造 Batch (增加 Batch 维度)
            batch = TensorDict({
                "rgb": rgb_tensor.unsqueeze(0),       # [1, 224, 224, 3]
                "depth": depth_tensor.unsqueeze(0),   # [1, 224, 224, 1]
                "objectgoal": object_goal_tensor,
                "gps": torch.tensor([[x, y]], device=DEVICE, dtype=torch.float32),
                "compass": torch.tensor([[yaw]], device=DEVICE, dtype=torch.float32),
                "heading": torch.tensor([[yaw]], device=DEVICE, dtype=torch.float32)
            })
            
            # --- 推理 ---
            with torch.no_grad():
                action_data = policy.act(batch, rnn_hidden_states, prev_actions, masks, deterministic=True)
                
            if isinstance(action_data, tuple):
                action, rnn_hidden_states = action_data
            else:
                action = action_data.actions
                rnn_hidden_states = action_data.rnn_hidden_states
                
            action_int = int(action.item())
            prev_actions.copy_(action)
            masks.fill_(True)
            
            print(f"🤖 Action: {action_int} | Pos: ({x:.2f}, {y:.2f}, {yaw:.4f})")
            
            # --- 回传地图与动作 ---
            obs_bgr, val_bgr = get_maps_from_policy(policy)
            counter+=1
            # 可视化 (可选)
            if obs_bgr is not None and VIS_DIR:
                cv2.imwrite(f"{VIS_DIR}/obs_{counter}.png", obs_bgr)

            obs_bytes = b""
            if obs_bgr is not None: _, buf = cv2.imencode(".png", obs_bgr); obs_bytes = buf.tobytes()
            val_bytes = b""
            if val_bgr is not None: _, buf = cv2.imencode(".png", val_bgr); val_bytes = buf.tobytes()

            conn.sendall(struct.pack("B", action_int))
            conn.sendall(struct.pack("I", len(obs_bytes)) + obs_bytes)
            conn.sendall(struct.pack("I", len(val_bytes)) + val_bytes)

    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback; traceback.print_exc()
    finally:
        conn.close()