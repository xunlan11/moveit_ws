#!/usr/bin/env python3
# coding: utf-8

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np
from visualization_msgs.msg import Marker
import letter

class TrajectoryControl:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('iiwa7_trajectory_control', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.move_group.set_planning_time(15.0)
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20
            )
        self.marker_publisher = rospy.Publisher(
            '/eef_trajectory_marker',
            Marker,
            queue_size=10
            )
        self.planning_frame = self.move_group.get_planning_frame()
        rospy.loginfo("机器人参考坐标系: %s" % self.planning_frame)
        self.eef_link = self.move_group.get_end_effector_link()
        rospy.loginfo("末端执行器: %s" % self.eef_link)
        self.group_names = self.robot.get_group_names()
        rospy.loginfo("机器人组: %s" % self.group_names)
        self.initial_pose = self.move_group.get_current_pose().pose
        rospy.loginfo(f"初始末端位姿: {self.initial_pose}")
    # 移动到指定关节角度
    def move_J(self, joint_goal_array):
        current_joints = self.move_group.get_current_joint_values()
        if len(joint_goal_array) != len(current_joints):
            rospy.logerr(f"目标关节数 {len(joint_goal_array)} 与实际关节数 {len(current_joints)} 不符。")
            return False
        self.move_group.set_joint_value_target(joint_goal_array)
        plan_success, plan, _, _ = self.move_group.plan()
        if not plan_success:
            rospy.logerr(f"规划目标角度失败。")
            return False
        rospy.loginfo(f"规划目标角度成功，开始执行。")
        execute_success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        if not execute_success:
            rospy.logerr("执行目标角度失败。")
            return False
        rospy.loginfo("执行目标角度成功。")
        return True
    # 规划笛卡尔路径
    def plan_cartesian_path(self, waypoints):
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        waypoints_list = copy.deepcopy(waypoints)
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints_list,  # 路径点列表
            0.01,            # 末端执行器步长
        )
        rospy.loginfo(f"笛卡尔路径规划完成，成功比例: {fraction:.2f}。")
        return plan, fraction
    # 执行规划轨迹
    def execute_plan(self, plan):
        rospy.loginfo("开始执行规划的轨迹。")
        success = self.move_group.execute(plan, wait=True)
        if success:
            rospy.loginfo("轨迹执行成功。")
        else:
            rospy.logerr("轨迹执行失败。")
        return success
    # 清除指定空间下的所有RViz Markers
    def clear_markers(self, ns="trajectory"):
        marker = Marker()
        marker.header.frame_id = self.planning_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = 0 
        marker.action = Marker.DELETEALL
        self.marker_publisher.publish(marker)
        rospy.loginfo(f"已发送清除Marker命令到空间{ns}")
    # 显示路径
    def show_path(self, waypoints_pose_list, ns="trajectory", r=1.0, g=0.0, b=0.0):
        self.clear_markers(ns)
        rospy.sleep(0.05)
        marker = Marker()
        marker.header.frame_id = self.planning_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.8 
        marker.lifetime = rospy.Duration()
        for pose in waypoints_pose_list:
            marker.points.append(pose.position)
        rospy.loginfo(f"发布包含 {len(marker.points)} 个点的轨迹 Marker 到 /eef_trajectory_marker (ns: {ns})。")
        self.marker_publisher.publish(marker)
    # 字母（YZ平面）
    def run(self):
        rospy.loginfo("============ 轨迹控制 ============")
        # 初始姿态：非奇异
        rospy.loginfo("============ 移动到初始姿态 ============")
        ready_joint_angles = [0.0, np.deg2rad(-15), 0.0, np.deg2rad(-90), 0.0, np.deg2rad(90), 0.0]
        if not self.move_J(ready_joint_angles):
            rospy.logerr("移动到初始姿态失败，中止。")
            return
        rospy.loginfo("成功移动到初始姿态。")
        # 获取当前姿态
        current_eef_pose = self.move_group.get_current_pose().pose
        # 获取字母并生成轨迹
        rospy.loginfo("============ 输入字符（字母/数字）并生成轨迹 ============")
        input_char_str = ""
        while not input_char_str:
            try:
                input_char_str = input("请输入要绘制的单个字符（字母/数字）并按回车键: ").strip()
                if not input_char_str:
                    rospy.logwarn("输入为空，请重新输入。")
                elif len(input_char_str) > 1:
                    rospy.logwarn(f"输入了多个字符 ('{input_char_str}')。将仅使用第一个字符: '{input_char_str[0]}'")
                    input_char_str = input_char_str[0]
            except EOFError:
                rospy.logerr("接收到文件结束符，无法获取输入，使用 'A'。")
                input_char_str = 'A'
                break
            except Exception as e:
                rospy.logerr(f"读取输入时发生错误: {e}，使用 'A'。")
                input_char_str = 'A'
                break
        char_to_draw = input_char_str
        font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" 
        desired_letter_height_m = 0.1 # 10cm高
        num_samples_per_curve = 10    # 曲线段采样点数
        letter_waypoints = letter.generate_letter_waypoints_from_font(
            char_to_draw=char_to_draw,
            font_path=font_path,
            desired_letter_height_m=desired_letter_height_m,
            num_samples_per_curve=num_samples_per_curve,
            center_pose=current_eef_pose, # 使用当前位姿作为中心
            rospy_instance=rospy
        )
        if not letter_waypoints:
            rospy.logerr(f"未能为字符 '{char_to_draw}' 生成轨迹点。")
        else:
            rospy.loginfo(f"成功为字符 '{char_to_draw}' 生成 {len(letter_waypoints)} 个轨迹点。")
            # 显示生成的绿色轨迹点
            self.show_path(letter_waypoints, ns="letter_trajectory", r=0.0, g=1.0, b=0.0)
            rospy.sleep(1)
            rospy.loginfo("============ 规划笛卡尔路径 ============")
            letter_plan, fraction = self.plan_cartesian_path(letter_waypoints)
            if fraction > 0.9:
                rospy.loginfo(f"轨迹规划成功 (覆盖率 {fraction*100:.2f}%)，准备执行。")
                rospy.loginfo("============ 执行轨迹 ============")
                self.execute_plan(letter_plan)
                rospy.loginfo(f"字符 '{char_to_draw}' 绘制完成。")
            else:
                rospy.logwarn(f"轨迹规划覆盖率较低 ({fraction*100:.2f}%)，不执行。")
            rospy.sleep(2)
        # 回到初始姿态
        rospy.loginfo("============ 回到初始姿态 ============")
        home_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if not self.move_J(home_joint_angles):
            rospy.logwarn("回到初始姿态失败。")


if __name__ == '__main__':
    try:
        controller = TrajectoryControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)