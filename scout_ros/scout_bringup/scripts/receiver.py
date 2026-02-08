#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import socket
import struct
import cv2
import numpy as np
import time
import math
import tf
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# ==========================================
# ⚙️ 配置区域 (请根据实际情况调整)
# ==========================================
WINDOWS_IP = "192.168.31.83"   # Windows IP
WINDOWS_PORT = 8888            

# # --- 动作参数 (根据实际测试调整校准) ---
# LINEAR_VEL = 0.25      # 前进速度 (m/s)
# ANGULAR_VEL = 0.8 #0.5       # 转向速度 (rad/s)
ROTATE_SPEED = 0.6  # rad/s

# # 时间补偿 (根据实测，通常需要 1.1~1.3 倍来抵消启动延迟)
# TIME_FORWARD = (0.25 / LINEAR_VEL) * 1.05
# TIME_TURN = (0.52 / ANGULAR_VEL) * 1.1

# ==========================================
# 全局变量
# ==========================================
bridge = CvBridge()
conn = None
goal_pub = None
tf_listener = None

latest_rgb = None
latest_depth = None
# x = 0.0
# y = 0.0
# yaw = 0.0

# ==========================================
# 辅助函数: 确保读取指定长度的数据
# ==========================================
def recv_all(sock, size):
    data = b""
    while len(data) < size:
        try:
            chunk = sock.recv(size - len(data))
            if not chunk:
                return None
            data += chunk
        except socket.timeout:
            return None
        except Exception:
            return None
    return data

# ==========================================
# 1. 网络连接逻辑
# ==========================================
def connect_to_windows():
    global conn
    while not rospy.is_shutdown():
        try:
            rospy.loginfo(f" Connecting to Windows {WINDOWS_IP}:{WINDOWS_PORT} ...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)
            sock.connect((WINDOWS_IP, WINDOWS_PORT))
            conn = sock
            conn.settimeout(None)
            rospy.loginfo("✅ Connected to Windows successfully!")
            return
        except Exception as e:
            rospy.logerr(f"❌ Connection failed: {e}. Retrying in 2s...")
            time.sleep(2)

# ==========================================
# 2. ROS 回调函数
# ==========================================
def rgb_cb(msg):
    global latest_rgb
    try:
        latest_rgb = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        pass

def depth_cb(msg):
    global latest_depth
    try:
        # ✅ 订阅的是 aligned 话题，直接读取 (640x480)
        latest_depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as e:
        pass

# def odom_cb(msg):
#     global x, y
#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y

# def imu_cb(msg):
#     global yaw
#     q = msg.orientation
#     siny = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     yaw = math.atan2(siny, cosy)


# ==========================================
# 4. 主同步循环 (Native 640x480)
# ==========================================
def main_sync_loop():
    # global conn, latest_rgb, latest_depth, goal_pub

    global conn, latest_rgb, latest_depth, tf_listener, goal_pub
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(10)

    rospy.loginfo("Starting Navigation Loop...")

    # --- 【新增 1】 初始化阶段的状态变量 ---
    initial_spin_done = False  # 是否完成了初始旋转
    total_yaw_rotated = 0.0    # 累计转过的角度 (弧度)
    last_yaw_capture = None    # 上一帧的 Yaw 角
    # ------------------------------------

    # ==========================================
    # 1. 初始化 TF 监听器
    # ==========================================
    # tf_listener = tf.TransformListener()
    
    # 等待 TF 树建立 (重要：给系统一点缓冲时间)
    rospy.sleep(1.0) 

    while not rospy.is_shutdown():
        if conn is None:
            connect_to_windows()
            continue

        if latest_rgb is None or latest_depth is None:
            rospy.logwarn_throttle(2, "⏳ Waiting for camera data...")
            rate.sleep()
            continuec

        # ==========================================
        # 2. 获取当前位姿 (Map Frame)
        # ==========================================
        current_x = 0.0
        current_y = 0.0
        current_yaw = 0.0
        
        try:
            # 【关键修复】: 先等待变换关系可用，最多等 1.0 秒
            # 这能防止因 TF 还没同步好就直接查询导致的报错
            tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(0.5))
            
            # 查询 /base_link 在 /map 下的坐标
            # 注意：如果你的底盘 frame 叫 /base_footprint，请修改这里
            # (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            (trans, rot) = tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))

            current_x = trans[0]
            current_y = trans[1]
            
            # 四元数转欧拉角
            (_, _, current_yaw) = tf.transformations.euler_from_quaternion(rot)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(1, f"⚠️ TF lookup failed: {e}. Waiting for TF tree...")
            # 如果获取不到位姿，建议跳过本次发送，防止地图画错
            rate.sleep()
            continue

        try:
            # ==========================================
            # STEP A: 原样发送 (No Resize!)
            # ==========================================
            # Realsense Aligned 默认就是 640x480
            
            # 1. RGB (JPG)
            _, rgb_encoded = cv2.imencode(".jpg", latest_rgb)
            rgb_bytes = rgb_encoded.tobytes()

            # 2. Depth (PNG)
            # 智能转换防止溢出，但尺寸不变
            if latest_depth.dtype == np.uint16:
                depth_mm = latest_depth
            else:
                depth_temp = latest_depth * 1000.0
                depth_temp = np.nan_to_num(depth_temp, nan=0.0, posinf=65535.0, neginf=0.0)
                depth_mm = np.clip(depth_temp, 0, 65535).astype(np.uint16)

            _, depth_encoded = cv2.imencode(".png", depth_mm)
            depth_bytes = depth_encoded.tobytes()

            # 3. 发送
            conn.sendall(struct.pack("I", len(rgb_bytes)))
            conn.sendall(rgb_bytes)
            conn.sendall(struct.pack("I", len(depth_bytes)))
            conn.sendall(depth_bytes)
            conn.sendall(struct.pack("fff", current_x, current_y, current_yaw))
            
            # 接收 10 字节数据
            goal_data = recv_all(conn, 10)
            if not goal_data: raise BrokenPipeError("Server closed")

            action_id, has_goal, target_x, target_y = struct.unpack("<BBff", goal_data)


            # ==========================================
            # 【新增 2】 初始化旋转逻辑 (优先权最高)
            # ==========================================
            if not initial_spin_done:
                # 第一次进入循环，初始化 last_yaw
                if last_yaw_capture is None:
                    last_yaw_capture = current_yaw
                
                # 计算这一帧转了多少度 (处理 -pi 到 pi 的跳变)
                delta_yaw = current_yaw - last_yaw_capture
                
                # 修正角度跳变：例如从 3.14 变到 -3.14，实际只转了一点点，但数学差值很大
                if delta_yaw < -math.pi:
                    delta_yaw += 2 * math.pi
                elif delta_yaw > math.pi:
                    delta_yaw -= 2 * math.pi
                
                # 累加绝对值
                total_yaw_rotated += abs(delta_yaw)
                last_yaw_capture = current_yaw # 更新上一帧角度

                # 判断是否转够了一圈 (2 * PI ≈ 6.28, 这里给 6.4 确保覆盖)
                if total_yaw_rotated < 6.3:
                    twist = Twist()
                    twist.angular.z = 0.3  # 旋转速度，请与你的 DWA 参数匹配
                    cmd_pub.publish(twist)
                    rospy.loginfo_throttle(1.0, f" Initial Scanning... Rotated: {math.degrees(total_yaw_rotated):.1f}/360 deg")
                    
                    # 关键：初始化阶段，跳过后面的导航逻辑，直接进入下一次循环
                    # 这样 Server 虽然发了 goal，但我们会忽略，直到扫图完成
                    continue 
                else:
                    # 转完了
                    initial_spin_done = True
                    cmd_pub.publish(Twist()) # 刹车一次
                    rospy.loginfo("✅ Initial Scan Completed! Switching to Auto Navigation.")
                    # 继续向下执行正常的导航逻辑...

            
            # ==========================================
            # ⚡️ 极简控制逻辑 (Direct Global Goal)
            # ==========================================
            
            if has_goal:
                # ✅ 核心逻辑：直接把 Frontier 给 move_base
                goal_msg = PoseStamped()
                goal_msg.header.stamp = rospy.Time.now()
                goal_msg.header.frame_id = "map"
                goal_msg.pose.position.x = target_x
                goal_msg.pose.position.y = target_y
                goal_msg.pose.position.z = 0.0
                goal_msg.pose.orientation.w = 1.0 
                
                goal_pub.publish(goal_msg)
                # rospy.loginfo(f"�� Navigation: Go to Frontier ({target_x:.2f}, {target_y:.2f})")
                
                # 如果之前在旋转，现在不需要手动停，move_base 会接管 cmd_vel
            
            else:
                # ❌ 没有 Frontier (通常是初始化阶段或死胡同)
                # 执行原地旋转来更新地图
                twist = Twist()
                if action_id == 2: # Server 请求旋转
                    twist.angular.z = 0.3
                    cmd_pub.publish(twist)
                    rospy.loginfo_throttle(1.0, "�� Searching for Frontier (Spinning)...")
                else:
                    cmd_pub.publish(Twist()) # 停

            conn.settimeout(None)

        except socket.timeout:
            rospy.logerr("⏰ Timeout waiting for server action")
            if conn: conn.close()
            conn = None
        except BrokenPipeError:
            rospy.logerr("❌ Broken Pipe.")
            if conn: conn.close()
            conn = None
        except Exception as e:
            rospy.logerr(f"❌ Error: {e}")
            if conn: conn.close()
            conn = None
            time.sleep(1)

if __name__ == "__main__":
    rospy.init_node("robot_vlfm_agent", anonymous=True)
    tf_listener = tf.TransformListener()
    # ✅ 订阅 RGB
    rospy.Subscriber("/camera/color/image_raw", Image, rgb_cb)
    
    # ✅✅✅ 关键修改：订阅对齐后的深度图！
    # 确保 roslaunch realsense2_camera rs_aligned_depth.launch 已启动
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_cb) 
    
    # rospy.Subscriber("/odom", Odometry, odom_cb)
    # rospy.Subscriber("/imu/data_raw", Imu, imu_cb)
    goal_pub = rospy.Publisher("/target_goal", PoseStamped, queue_size=1)

    try:
        main_sync_loop()
    except rospy.ROSInterruptException:
        pass