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
# from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# ==========================================
# ⚙️ 配置区域 (请根据实际情况调整)
# ==========================================
WINDOWS_IP = "192.168.31.142"   # Windows IP
WINDOWS_PORT = 8888            

# # --- 动作参数 (根据实际测试调整校准) ---
# LINEAR_VEL = 0.25      # 前进速度 (m/s)
# ANGULAR_VEL = 0.8 #0.5       # 转向速度 (rad/s)

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
    global conn, latest_rgb, latest_depth, tf_listener, goal_pub

    rate = rospy.Rate(10)

    rospy.loginfo("Starting Navigation Loop...")

    # ==========================================
    # 1. 初始化 TF 监听器
    # ==========================================
    tf_listener = tf.TransformListener()
    
    # 等待 TF 树建立 (重要：给系统一点缓冲时间)
    rospy.sleep(1.0) 

    while not rospy.is_shutdown():
        if conn is None:
            connect_to_windows()
            continue

        if latest_rgb is None or latest_depth is None:
            rospy.logwarn_throttle(2, "⏳ Waiting for camera data...")
            rate.sleep()
            continue

        # ==========================================
        # 2. 获取当前位姿 (Map Frame)
        # ==========================================
        current_x = 0.0
        current_y = 0.0
        current_yaw = 0.0
        
        try:
            # 查询 /base_link 在 /map 下的坐标
            # 注意：如果你的底盘 frame 叫 /base_footprint，请修改这里
            (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            
            current_x = trans[0]
            current_y = trans[1]
            
            # 四元数转欧拉角
            (_, _, current_yaw) = tf.transformations.euler_from_quaternion(rot)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(1, "⚠️ TF lookup failed. Waiting for TF tree...")
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
            
            # ==========================================
            # STEP 2: 接收服务器响应 (关键修改)
            # ==========================================
            
            # 1. 接收目标点数据 (9 bytes: Flag + X + Y)
            # 对应服务端: struct.pack("<Bff", ...)
            GOAL_PACKET_SIZE = 9
            conn.settimeout(15.0)
            goal_data = recv_all(conn, GOAL_PACKET_SIZE)
            
            if not goal_data:
                raise BrokenPipeError("Server closed connection")

            has_goal, target_x, target_y = struct.unpack("<Bff", goal_data)

            # 2. 发布目标点给 move_base
            if has_goal:
                goal_msg = PoseStamped()
                goal_msg.header.stamp = rospy.Time.now()
                goal_msg.header.frame_id = "map"  # 必须是 map 系
                goal_msg.pose.position.x = target_x
                goal_msg.pose.position.y = target_y
                goal_msg.pose.position.z = 0.0
                # 给一个默认朝向 (朝前)
                goal_msg.pose.orientation.w = 1.0 
                
                goal_pub.publish(goal_msg)
                rospy.loginfo(f"🎯 New Goal -> move_base: ({target_x:.2f}, {target_y:.2f})")
            else:
                # 没目标时可以不做处理，move_base 会保持当前状态或停下
                # rospy.loginfo("💤 No goal received")
                pass

            # ==========================================
            # STEP 3: 接收并排空地图数据 (协议同步)
            # ==========================================
            # 即使机器人不需要显示地图，也必须把socket里的数据读走！
            # 否则下一帧发送时，socket 缓冲区里还有上一帧的地图数据，会导致解析错误。

            # A. 读 Obs Map
            len_data = recv_all(conn, 4)
            if len_data:
                obs_len = struct.unpack("I", len_data)[0]
                _ = recv_all(conn, obs_len) # 读出来扔掉 (或者你可以 decode 显示)

            # B. 读 Value Map
            len_data = recv_all(conn, 4)
            if len_data:
                val_len = struct.unpack("I", len_data)[0]
                _ = recv_all(conn, val_len) # 读出来扔掉

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

    # ✅ 订阅 RGB
    rospy.Subscriber("/camera/color/image_raw", Image, rgb_cb)
    
    # ✅✅✅ 关键修改：订阅对齐后的深度图！
    # 确保 roslaunch realsense2_camera rs_aligned_depth.launch 已启动
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_cb) 
    
    # rospy.Subscriber("/odom", Odometry, odom_cb)
    # rospy.Subscriber("/imu/data_raw", Imu, imu_cb)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    try:
        main_sync_loop()
    except rospy.ROSInterruptException:
        pass