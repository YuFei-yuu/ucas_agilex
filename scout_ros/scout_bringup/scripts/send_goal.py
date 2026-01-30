#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class SimpleNavNode:
    def __init__(self):
        rospy.init_node('simple_nav_commander', anonymous=True)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待 move_base 启动")
        self.client.wait_for_server()
        rospy.loginfo("move_base 已连接")

        self.last_goal_pose = None

        self.sub = rospy.Subscriber("/target_goal", PoseStamped, self.goal_callback)

    def goal_callback(self, msg):
        if self.last_goal_pose is not None:
            # 简单计算距离
            dx = msg.pose.position.x - self.last_goal_pose.pose.position.x
            dy = msg.pose.position.y - self.last_goal_pose.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)

            # 如果距离小于 0.1 米，则认为是重复指令，直接忽略
            if dist < 0.1:
                return 

        rospy.loginfo("收到新目标 (x:%.2f, y:%.2f)，正在执行", 
                      msg.pose.position.x, msg.pose.position.y)

        # 更新上一次的目标记录
        self.last_goal_pose = msg

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = msg.pose

        # 发送目标
        self.client.send_goal(goal, done_cb=self.done_cb, active_cb=self.active_cb)

    def active_cb(self):
        pass

    def done_cb(self, status, result):
        if status == 3:
            rospy.loginfo("到达目的地")
        elif status == 4:
            rospy.loginfo("目标无法到达")

if __name__ == '__main__':
    try:
        node = SimpleNavNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass