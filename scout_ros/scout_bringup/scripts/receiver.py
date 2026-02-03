#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import socket
import struct
import cv2
import numpy as np
import time
import math
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# ==========================================
# ⚙️ 配置区域 (请根据实际情况调整)
# ==========================================
WINDOWS_IP = "192.168.31.142"   # Windows IP
WINDOWS_PORT = 8888            

# --- 动作参数 (根据实际测试调整校准) ---
LINEAR_VEL = 0.25      # 前进速度 (m/s)
ANGULAR_VEL = 0.8 #0.5       # 转向速度 (rad/s)

# 时间补偿 (根据实测，通常需要 1.1~1.3 倍来抵消启动延迟)
TIME_FORWARD = (0.25 / LINEAR_VEL) * 1.05
TIME_TURN = (0.52 / ANGULAR_VEL) * 1.1

# ==========================================
# 全局变量
# ==========================================
bridge = CvBridge()
conn = None
cmd_pub = None

latest_rgb = None
latest_depth = None
x = 0.0
y = 0.0
yaw = 0.0

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

def odom_cb(msg):
    global x, y
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

def imu_cb(msg):
    global yaw
    q = msg.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)

# ==========================================
# 3. 动作执行器 (阻塞式)
# ==========================================
def execute_action_blocking(action_code):
    global cmd_pub
    cmd = Twist()
    action_name = ["STOP", "FORWARD", "LEFT", "RIGHT"]
    
    if action_code < 0 or action_code > 3: return

    rospy.loginfo(f"烙 Action: {action_name[action_code]}")

    if action_code == 0: # STOP
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        cmd_pub.publish(cmd)
        time.sleep(0.1)
        return
    elif action_code == 1: # FORWARD
        cmd.linear.x = LINEAR_VEL
        cmd_pub.publish(cmd)
        time.sleep(TIME_FORWARD)
    elif action_code == 2: # LEFT
        cmd.angular.z = ANGULAR_VEL
        cmd_pub.publish(cmd)
        time.sleep(TIME_TURN)
    elif action_code == 3: # RIGHT
        cmd.angular.z = -ANGULAR_VEL
        cmd_pub.publish(cmd)
        time.sleep(TIME_TURN)

    # 刹车 + 稳定
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    cmd_pub.publish(cmd)
    time.sleep(0.5) # 给相机一点稳定时间

# ==========================================
# 4. 主同步循环 (Native 640x480)
# ==========================================
def main_sync_loop():
    global conn, latest_rgb, latest_depth

    rate = rospy.Rate(10)

    rospy.loginfo(" Starting 640x480 Native Loop...")

    while not rospy.is_shutdown():
        if conn is None:
            connect_to_windows()
            continue

        if latest_rgb is None or latest_depth is None:
            rospy.logwarn_throttle(2, "⏳ Waiting for camera data...")
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
            conn.sendall(struct.pack("fff", x, y, yaw))
            
            # ==========================================
            # STEP B: 阻塞接收 Action
            # ==========================================
            conn.settimeout(15.0) # 15秒超时
            action_data = conn.recv(1)
            conn.settimeout(None)
            
            if not action_data:
                rospy.logwarn("⚠️ Server closed connection")
                conn.close()
                conn = None
                continue

            action_id = struct.unpack("B", action_data)[0]

            # ==========================================
            # STEP C: 执行
            # ==========================================
            execute_action_blocking(action_id)

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
    
    rospy.Subscriber("/odom", Odometry, odom_cb)
    rospy.Subscriber("/imu/data_raw", Imu, imu_cb)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    try:
        main_sync_loop()
    except rospy.ROSInterruptException:
        pass