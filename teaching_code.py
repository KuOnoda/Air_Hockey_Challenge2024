#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import pickle
import moveit_commander
from std_msgs.msg import UInt8, String
import sys, tty, termios, select

from geometry_msgs.msg import Pose

from ball_detection import CameraTracker


def preset_pid_gain(pid_gain_no):
    # サーボモータのPIDゲインをプリセットする
    # プリセットの内容はcrane_x7_control/scripts/preset_reconfigure.pyに書かれている
    rospy.loginfo("PID Gain Preset. No." + str(pid_gain_no))
    preset_no = UInt8()
    preset_no.data = pid_gain_no
    pub_preset.publish(preset_no)
    rospy.sleep(1) # PIDゲインがセットされるまで待つ


def save_states_to_file(states):
    filename = rospy.get_param('~output_file', '/root/work/catkin_ws/src/crane_x7_ros/crane_x7_teaching/data/2025-01-16-episode2.pkl')
    absolute_path = os.path.abspath(filename)
    with open(filename, 'wb') as f:
        pickle.dump(states, f)
    rospy.loginfo(f'save states to file{absolute_path}')


class TeachingNode:
    def __init__(self):
        self.collecting = 0
        self.data = []
        self.data_collection = []
        self.data_count = 0
        rospy.init_node("teaching_node")
        rospy.Subscriber("/data_collection", String, self.control_callback)

    def control_callback(self, msg):
        command = msg.data
        if command == "s":
            self.collecting = 1
            rospy.loginfo("start data collection")
        elif command == "stop":
            self.collecting = 2
            rospy.loginfo("stop data collection")
        elif command == "p":
            self.collecting = 3
        elif command == "exit":
            rospy.loginfo("exit")
            rospy.signal_shutdown()

    def run(self):
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.5)
        arm.set_max_acceleration_scaling_factor(1.0)
        gripper = moveit_commander.MoveGroupCommander("gripper")

        tracker = CameraTracker()
        tracker.find_camera_device()
        tracker.setup_camera()

        TORQUE_ON_PID = 0
        TORQUE_OFF_PID = 3

        # 何かを掴んでいた時のためにハンドを開く
        gripper.set_joint_value_target([0.9, 0.9])
        gripper.go()

        # SRDFに定義されている"home"の姿勢にする
        arm.set_named_target("home")
        arm.go()

        # preset_reconfigureノードが起動するまで待機
        rospy.sleep(1)

        # トルクをオフにする
        preset_pid_gain(TORQUE_OFF_PID)
        print("Torque OFF")



        rate = rospy.Rate(200) #200Hz   

        print("please input")     

        while not rospy.is_shutdown():
            if self.collecting == 1:
                current_pose = arm.get_current_pose().pose
                current_xy = (current_pose.position.x, current_pose.position.y)
                print(f'current_xy:{current_xy}')

                pack_x, pack_y, pack_vx, pack_vy = tracker.track_when_it_called()

                self.data.append((current_pose.position.x, current_pose.position.y, pack_x, pack_y, pack_vx, pack_vy))
            elif self.collecting == 3:
                self.data_collection.append(self.data)
                self.data = []
                self.collecting = 0
            elif self.collecting == 2:
                break

            
            rate.sleep()



        save_states_to_file(self.data_collection)

        #if finished Touque ON
        rospy.sleep(1)
        arm.set_pose_target(arm.get_current_pose())
        arm.go()
        gripper.set_joint_value_target(gripper.get_current_joint_values())
        gripper.go()
        preset_pid_gain(TORQUE_ON_PID)

        # SRDFに定義されている"vertical"の姿勢にする
        arm.set_named_target("vertical")
        arm.go()

        tracker.release_resources()


if __name__ == '__main__':
    node = TeachingNode()
    # PIDゲインプリセット用のPublisher
    pub_preset = rospy.Publisher("preset_gain_no", UInt8, queue_size=1)

    node.run()
