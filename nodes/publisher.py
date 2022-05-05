#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import PointStamped # 엔코더 메시지
import tf # tf broadcaster
import math
from time import time
from nav_msgs.msg import Odometry # 오도메트리 메시지
from sensor_msgs.msg import JointState # 바퀴 조인트 상태 메시지


def encoder_callback(data) : 

    #time_prev = 0
    global l_wheel_pulse
    global r_wheel_pulse
    global time_prev
    global x_pos
    global y_pos
    global theta
    new_l_wheel_pulse = data.point.x
    new_r_wheel_pulse = data.point.y
    delta_l_pulse = new_l_wheel_pulse - l_wheel_pulse
    delta_r_pulse = new_r_wheel_pulse - r_wheel_pulse
    l_wheel_pulse = new_l_wheel_pulse # 새 엔코더 값을 기존 엔코더 값에 덮어씌움
    r_wheel_pulse = new_r_wheel_pulse
    #rospy.loginfo("left encoder : %d", delta_l_pulse)
    #rospy.loginfo("right encoder : %d", delta_r_pulse)
    time_now = time()
    delta_time = time_now - time_prev
    time_prev = time_now


    l_vel = (2 * math.pi * 0.032 * (delta_l_pulse / 646)) / delta_time
    r_vel = (2 * math.pi * 0.032 * (delta_r_pulse / 646)) / delta_time

    linear_vel = (l_vel + r_vel) / 2.0
    angular_vel = (r_vel - l_vel) / 0.372
    delta_s = linear_vel * delta_time
    delta_theta = angular_vel * delta_time # rad

    delta_vx = delta_s * math.cos(delta_theta)
    delta_vy = delta_s * math.sin(delta_theta)

    delta_x = (delta_vx * math.cos(theta) - delta_vy * math.sin(theta)) * delta_time
    delta_y = (delta_vx * math.sin(theta) + delta_vy * math.cos(theta)) * delta_time
    
    x_pos += delta_x
    y_pos += delta_y
    theta += delta_theta

    if theta >= math.pi * 2 : theta -= math.pi * 2
    if theta <= -math.pi * 2 : theta += math.pi * 2

    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)

    # ----------------odometry 퍼블리시---------------------------

    odom = Odometry()
            
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_footprint"

    # robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos
    odom.pose.pose.position.y = y_pos
    odom.pose.pose.position.z = 0.0
    # robot's heading in quaternion
    odom.pose.pose.orientation.x = quaternion[0]
    odom.pose.pose.orientation.y = quaternion[1]
    odom.pose.pose.orientation.z = quaternion[2]
    odom.pose.pose.orientation.w = quaternion[3]

    odom.pose.covariance[0] = 1e-3
    odom.pose.covariance[7] = 1e-3
    odom.pose.covariance[14] = 1e6
    odom.pose.covariance[21] = 1e6
    odom.pose.covariance[28] = 1e6
    odom.pose.covariance[35] = 1e3

    # linear speed from encoders
    odom.twist.twist.linear.x = linear_vel
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.linear.z = 0.0
    odom.twist.twist.angular.x = 0.0
    odom.twist.twist.angular.y = 0.0
    odom.twist.twist.angular.z = angular_vel

    odom.twist.covariance[0] = 1e-3
    odom.twist.covariance[7] = 1e-3
    odom.twist.covariance[14] = 1e6
    odom.twist.covariance[21] = 1e3
    odom.twist.covariance[28] = 1e6
    odom.twist.covariance[35] = 1e3

    odom_publisher.publish(odom)
    #------------------------------------------------------------
    #---------------tf 퍼블리시------------------------------------
    br = tf.TransformBroadcaster()
    br.sendTransform((x_pos,y_pos,0),
                     (quaternion[0],quaternion[1],quaternion[2],quaternion[3]),
                     rospy.Time.now(),
                     "base_footprint",
                     "odom")
    #------------------------------------------------------------
    #---------------joint state 퍼블리시---------------------------
    joint_state = JointState()
    joint_state.header.frame_id = "base_link"
    joint_state.header.stamp = rospy.Time.now()

    joint_state.name = ["wheel_left_joint", "wheel_right_joint"]
    joint_state.position = [l_vel*delta_time , r_vel*delta_time]
    joint_state.velocity = [l_vel,r_vel]

    joint_state_publisher.publish(joint_state)
    #------------------------------------------------------------





if __name__=="__main__":
    
    rospy.init_node('publisher')   
    rospy.Subscriber("encoder_data", PointStamped, encoder_callback)
    odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=50)
    joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=50)

    global l_wheel_pulse
    global r_wheel_pulse
    global time_prev
    global x_pos
    global y_pos
    global theta
    l_wheel_pulse = 0
    r_wheel_pulse = 0
    time_prev = 0
    x_pos = 0
    y_pos = 0
    theta = 0

    rate = rospy.Rate(50)
    rospy.loginfo("publishing")
    rospy.spin()
    rate.sleep()


    