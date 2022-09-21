#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from math import pow
from sensor_msgs.msg import Imu # 바퀴 조인트 상태 메시지


def imu_callback(data) : 
    
    imu = Imu()

    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = data.header.frame_id

    imu.orientation_covariance = data.orientation_covariance

    imu.angular_velocity.x = data.angular_velocity.x
    imu.angular_velocity.y = data.angular_velocity.y
    imu.angular_velocity.z = data.angular_velocity.z
    imu.angular_velocity_covariance = data.angular_velocity_covariance

    imu.linear_acceleration.x = data.linear_acceleration.x
    imu.linear_acceleration.y = data.linear_acceleration.y
    imu.linear_acceleration.z = data.linear_acceleration.z
    imu.linear_acceleration_covariance = data.linear_acceleration_covariance



    if pow(imu.orientation.x,2) + pow(imu.orientation.y,2) + pow(imu.orientation.z,2) + pow(imu.orientation.w,2) > 1.2 : 
         imu.orientation.x = orientation_x_b
         imu.orientation.y = orientation_y_b
         imu.orientation.z = orientation_z_b
         imu.orientation.w = orientation_w_b
    else : 
         imu.orientation.x = data.orientation.x
         imu.orientation.y = data.orientation.y
         imu.orientation.z = data.orientation.z
         imu.orientation.w = data.orientation.w

         orientation_x_b = data.orientation.x
         orientation_y_b = data.orientation.y
         orientation_z_b = data.orientation.z
         orientation_w_b = data.orientation.w

    imu_publisher.publish(imu)


if __name__=="__main__":
    
    rospy.init_node('imu_filter')   
    rospy.Subscriber("/imu", Imu, imu_callback)
    imu_publisher = rospy.Publisher('/imu_data', Imu, queue_size=50)


    global orientation_x_b
    global orientation_y_b
    global orientation_z_b
    global orientation_w_b
    orientation_x_b = 0
    orientation_y_b = 0
    orientation_z_b = 0
    orientation_w_b = 0

    rate = rospy.Rate(50)
    rospy.loginfo("publishing new imu data")
    rospy.spin()
    rate.sleep()


    
