import socket
import struct
import time
import cv2
import numpy as np
import os
import ctypes
from ctypes import wintypes  # 恢复：用于 Windows 屏幕检测

# =========================
# 配置
# =========================
LOCAL_BIND_IP = "0.0.0.0"
LOCAL_BIND_PORT = 8888  # 对应小车连接的端口
SERVER_IP = "127.0.0.1"
SERVER_PORT = 9000      # 对应推理服务器的端口

# �� 可视化放到哪个屏幕：
#   - "primary"   : 主屏
#   - "secondary" : 次屏（推荐：接电视的那块）
#   - 整数         : 指定第 N 块屏幕（0/1/2...），按检测顺序
VIS_MONITOR = "secondary"

# 是否每帧都强制把窗口“铺回去”。
#   True  : 永远保持 2×2 贴满目标屏幕（推荐）
#   False : 只在启动时铺一次，你可以手动拖动窗口
FORCE_LAYOUT_EVERY_FRAME = True

# �� 数据保存配置
SAVE_DIR = "bridge_recorded_frames"
os.makedirs(SAVE_DIR, exist_ok=True)

frame_idx = 0
while True:
    if os.path.exists(os.path.join(SAVE_DIR, f"rgb_{frame_idx}.png")):
        frame_idx += 1
    else:
        print(f"�� 数据记录将从第 {frame_idx} 帧开始...")
        break

# =========================
# recv_all 工具函数
# =========================
def recv_all(conn: socket.socket, size: int) -> bytes:
    """阻塞式读取 size 字节；返回 b"" 表示对端断开。"""
    data = b""
    while len(data) < size:
        chunk = conn.recv(size - len(data))
        if not chunk:
            return b""
        data += chunk
    return data


# =========================
# 2x2 可视化布局 (已恢复 0107 版本的完整逻辑)
# =========================
WIN_RGB = "RGB"
WIN_DEPTH = "Depth"
WIN_OBS = "Obstacle Map"
WIN_VAL = "Value Map"

_TARGET_MON_CACHE = None

def _get_monitors_screeninfo():
    """优先用 screeninfo（跨平台，若已安装）。"""
    try:
        from screeninfo import get_monitors

        mons = []
        for m in get_monitors():
            mons.append(
                {
                    "x": int(getattr(m, "x", 0)),
                    "y": int(getattr(m, "y", 0)),
                    "w": int(getattr(m, "width", 0)),
                    "h": int(getattr(m, "height", 0)),
                    "primary": bool(getattr(m, "is_primary", False)),
                    "name": str(getattr(m, "name", "")),
                }
            )
        return mons if mons else None
    except Exception:
        return None


def _get_monitors_windows_ctypes():
    """Windows 下用 ctypes 枚举显示器（无需额外依赖）。"""
    try:
        user32 = ctypes.windll.user32
        MONITORINFOF_PRIMARY = 0x00000001

        class RECT(ctypes.Structure):
            _fields_ = [("left", wintypes.LONG), ("top", wintypes.LONG), ("right", wintypes.LONG), ("bottom", wintypes.LONG)]

        class MONITORINFOEXW(ctypes.Structure):
            _fields_ = [
                ("cbSize", wintypes.DWORD),
                ("rcMonitor", RECT),
                ("rcWork", RECT),
                ("dwFlags", wintypes.DWORD),
                ("szDevice", wintypes.WCHAR * 32),
            ]

        monitors = []

        MONITORENUMPROC = ctypes.WINFUNCTYPE(
            wintypes.BOOL,
            wintypes.HMONITOR,
            wintypes.HDC,
            ctypes.POINTER(RECT),
            wintypes.LPARAM,
        )

        def _cb(hMonitor, hdcMonitor, lprcMonitor, dwData):
            info = MONITORINFOEXW()
            info.cbSize = ctypes.sizeof(MONITORINFOEXW)
            user32.GetMonitorInfoW(hMonitor, ctypes.byref(info))
            r = info.rcMonitor
            monitors.append(
                {
                    "x": int(r.left),
                    "y": int(r.top),
                    "w": int(r.right - r.left),
                    "h": int(r.bottom - r.top),
                    "primary": bool(info.dwFlags & MONITORINFOF_PRIMARY),
                    "name": str(info.szDevice),
                }
            )
            return True

        user32.EnumDisplayMonitors(0, 0, MONITORENUMPROC(_cb), 0)
        return monitors if monitors else None
    except Exception:
        return None


def get_target_monitor_rect():
    """返回目标显示器矩形：(x, y, w, h)，坐标是“虚拟桌面”坐标系。"""
    global _TARGET_MON_CACHE
    try:
        if _TARGET_MON_CACHE is not None:
            return _TARGET_MON_CACHE
    except NameError:
        _TARGET_MON_CACHE = None

    monitors = _get_monitors_screeninfo() or _get_monitors_windows_ctypes()

    if not monitors:
        # 退化：用 tkinter 只能拿到主屏
        try:
            import tkinter as tk
            root = tk.Tk()
            root.withdraw()
            w = int(root.winfo_screenwidth())
            h = int(root.winfo_screenheight())
            root.destroy()
            monitors = [{"x": 0, "y": 0, "w": w, "h": h, "primary": True, "name": ""}]
        except Exception:
            monitors = [{"x": 0, "y": 0, "w": 1920, "h": 1080, "primary": True, "name": ""}]

    # 打印检测到的屏幕
    print("��️ Detected monitors:")
    for i, m in enumerate(monitors):
        flag = "PRIMARY" if m.get("primary") else ""
        print(f"   [{i}] {m.get('name','')}  ({m['x']},{m['y']}) {m['w']}x{m['h']}  {flag}")

    # 选择目标屏幕
    if isinstance(VIS_MONITOR, int):
        idx = max(0, min(VIS_MONITOR, len(monitors) - 1))
        m = monitors[idx]
    elif str(VIS_MONITOR).lower() == "primary":
        prim = [mm for mm in monitors if mm.get("primary")]
        m = prim[0] if prim else monitors[0]
    else:
        # secondary：选一个“非主屏里面积最大”的
        non = [mm for mm in monitors if not mm.get("primary")]
        if non:
            m = sorted(non, key=lambda mm: mm["w"] * mm["h"], reverse=True)[0]
        else:
            m = monitors[0]

    print(f"�� Using monitor for visualization: ({m['x']},{m['y']}) {m['w']}x{m['h']}  name={m.get('name','')}")
    _TARGET_MON_CACHE = (int(m["x"]), int(m["y"]), int(m["w"]), int(m["h"]))
    return _TARGET_MON_CACHE


def layout_2x2():
    """将 4 个窗口按 2x2 铺满目标屏幕（电视）。"""
    mx, my, mw, mh = get_target_monitor_rect()
    w = max(320, mw // 2)
    h = max(240, mh // 2)

    layout = [
        (WIN_RGB, mx + 0, my + 0),
        (WIN_DEPTH, mx + w, my + 0),
        (WIN_OBS, mx + 0, my + h),
        (WIN_VAL, mx + w, my + h),
    ]

    for name, x, y in layout:
        try:
            cv2.resizeWindow(name, w, h)
            cv2.moveWindow(name, x, y)
        except Exception:
            pass


def keep_visualization_alive(last_rgb, last_depth_vis, last_obs, last_val):
    """数据流停止后：保留四个窗口显示最后一帧，直到用户手动 Ctrl+C 退出。"""
    print("�� 数据流已停止：可视化窗口将保持显示最后一帧。按 Ctrl+C 退出本地 bridge。")
    while True:
        if last_rgb is not None:
            cv2.imshow(WIN_RGB, last_rgb)
        if last_depth_vis is not None:
            cv2.imshow(WIN_DEPTH, last_depth_vis)
        if last_obs is not None:
            cv2.imshow(WIN_OBS, last_obs)
        if last_val is not None:
            cv2.imshow(WIN_VAL, last_val)
        if cv2.waitKey(30) & 0xFF == 27:  # Esc key
            break


# =========================
# 主逻辑 (核心修改区域：协议保持 0204，界面恢复 0107)
# =========================
def main():
    # 创建窗口
    cv2.namedWindow(WIN_RGB, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_DEPTH, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_OBS, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_VAL, cv2.WINDOW_NORMAL)

    # 启动时铺一次
    layout_2x2()
    
    # 记录“最后一帧”
    last_rgb = None
    last_depth_vis = None
    last_obs = None
    last_val = None

    server_conn = None
    bridge_server = None
    robot_conn = None

    # 1) 连接推理服务器
    while True:
        try:
            print(f"�� Connecting to Inference Server ({SERVER_IP}:{SERVER_PORT})...")
            server_conn = socket.socket()
            server_conn.connect((SERVER_IP, SERVER_PORT))
            print("✅ Connected to Inference Server")
            break
        except Exception as e:
            print(f"❌ Server connect failed: {e}. Retrying in 2s...")
            time.sleep(2)

    # 2) 等待机器人连接
    print(f"�� Waiting for Robot on {LOCAL_BIND_IP}:{LOCAL_BIND_PORT} ...")
    bridge_server = socket.socket()
    bridge_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    bridge_server.bind((LOCAL_BIND_IP, LOCAL_BIND_PORT))
    bridge_server.listen(1)
    robot_conn, addr = bridge_server.accept()
    print(f"✅ Robot Connected from: {addr}")

    stop_reason = None

    try:
        while True:
            # ==========================================================
            # A. 接收来自机器人的数据 (RGB + Depth + Pose)
            # ==========================================================
            # 1. RGB Header (4 bytes)
            rgb_header = recv_all(robot_conn, 4)
            if not rgb_header:
                stop_reason = "Robot disconnected (rgb header)"
                break
            rgb_size = struct.unpack("I", rgb_header)[0]
            
            # 2. RGB Data
            rgb_bytes = recv_all(robot_conn, rgb_size)
            if not rgb_bytes: break
            rgb_np = cv2.imdecode(np.frombuffer(rgb_bytes, np.uint8), cv2.IMREAD_COLOR)

            # 3. Depth Header (4 bytes)
            depth_header = recv_all(robot_conn, 4)
            if not depth_header: break
            depth_size = struct.unpack("I", depth_header)[0]
            
            # 4. Depth Data
            depth_bytes = recv_all(robot_conn, depth_size)
            if not depth_bytes: break
            depth_mm = cv2.imdecode(np.frombuffer(depth_bytes, np.uint8), cv2.IMREAD_UNCHANGED)

            # 5. State/Pose (12 bytes: x, y, yaw)
            state_bytes = recv_all(robot_conn, 12)
            if not state_bytes: break

            # --- 本地可视化 (Robot发送的画面) ---
            if rgb_np is not None:
                last_rgb = rgb_np
                cv2.imshow(WIN_RGB, rgb_np)
            
            if depth_mm is not None:
                # 简单的深度图可视化映射
                vis_depth = depth_mm.copy()
                vis_depth[vis_depth > 5000] = 5000 # 裁剪到 5米
                vis_depth = (vis_depth.astype(np.float32) / 5000.0 * 255).astype(np.uint8)
                depth_vis_color = cv2.applyColorMap(vis_depth, cv2.COLORMAP_JET)
                last_depth_vis = depth_vis_color
                cv2.imshow(WIN_DEPTH, depth_vis_color)
            
            # 保存数据 (可选)
            global frame_idx
            if rgb_np is not None and depth_mm is not None:
                save_path_rgb = os.path.join(SAVE_DIR, f"rgb_{frame_idx}.png")
                save_path_depth = os.path.join(SAVE_DIR, f"depth_{frame_idx}.png")
                cv2.imwrite(save_path_rgb, rgb_np)
                cv2.imwrite(save_path_depth, depth_mm)
                frame_idx += 1
            
            # ==========================================================
            # B. 转发给推理服务器
            # ==========================================================
            try:
                # 原样打包转发
                payload = struct.pack("I", rgb_size) + rgb_bytes + \
                          struct.pack("I", depth_size) + depth_bytes + \
                          state_bytes
                server_conn.sendall(payload)
            except Exception as e:
                stop_reason = f"Server disconnected while sending: {e}"
                break

            # ==========================================================
            # C. 接收推理结果 (Goal + Maps) 并 筛选转发
            # ==========================================================
            
            # 1. 从 Server 接收目标包 (0204 协议: 10 bytes)
            # 协议: [Action(1B) Flag(1B) X(4B) Y(4B)]
            GOAL_PACKET_SIZE = 10 
            goal_data = recv_all(server_conn, GOAL_PACKET_SIZE)
            
            if not goal_data:
                stop_reason = "Server disconnected (goal)"
                break
            
            # 解包打印调试 (可选)
            # act_id, has_goal, tx, ty = struct.unpack("<BBff", goal_data)
            # print(f"�� Relay: Act={act_id} Goal={has_goal}")

            # 2. 从 Server 接收 Obs Map (用于本地显示)
            obs_head = recv_all(server_conn, 4)
            if not obs_head: break
            obs_size = struct.unpack("I", obs_head)[0]
            obs_bytes = recv_all(server_conn, obs_size) if obs_size > 0 else b""

            # 3. 从 Server 接收 Val Map (用于本地显示)
            val_head = recv_all(server_conn, 4)
            if not val_head: break
            val_size = struct.unpack("I", val_head)[0]
            val_bytes = recv_all(server_conn, val_size) if val_size > 0 else b""

            # --- Windows 本地可视化 ---
            if obs_size > 0:
                obs_img = cv2.imdecode(np.frombuffer(obs_bytes, np.uint8), cv2.IMREAD_COLOR)
                if obs_img is not None:
                    last_obs = obs_img
                    cv2.imshow(WIN_OBS, obs_img)
            
            if val_size > 0:
                val_img = cv2.imdecode(np.frombuffer(val_bytes, np.uint8), cv2.IMREAD_COLOR)
                if val_img is not None:
                    last_val = val_img
                    cv2.imshow(WIN_VAL, val_img)

            # ✅ 恢复：强制刷新布局，防止窗口重叠导致看不见地图
            if FORCE_LAYOUT_EVERY_FRAME:
                layout_2x2()

            cv2.waitKey(1)

            # 4. ⚡️⚡️ 关键修改：只把 Goal 转发给机器人 ⚡️⚡️
            # 我们不再拼接 obs/val 数据，只发 goal_data
            try:
                robot_conn.sendall(goal_data) 
            except Exception as e:
                stop_reason = f"Robot disconnected while sending response: {e}"
                break

    except Exception as e:
        print(f"❌ Bridge Loop Error: {e}")
        import traceback; traceback.print_exc()

    finally:
        print(f"⚠️ Bridge loop ended: {stop_reason}")
        try:
            if robot_conn: robot_conn.close()
        except: pass
        try:
            if server_conn: server_conn.close()
        except: pass
        try:
            if bridge_server: bridge_server.close()
        except: pass
        keep_visualization_alive(last_rgb, last_depth_vis, last_obs, last_val)

if __name__ == "__main__":
    main()