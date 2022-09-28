#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import tf
from math import pow
from sensor_msgs.msg import Imu # 바퀴 조인트 상태 메시지


def imu_callback(data) : 

    global orientation_x_b
    global orientation_y_b
    global orientation_z_b
    global orientation_w_b
    global orientation_filtered_x
    global orientation_filtered_y
    global orientation_filtered_z
    global orientation_filtered_w

    global gyro_x_b
    global gyro_y_b
    global gyro_z_b
    global gyro_filtered_x
    global gyro_filtered_y
    global gyro_filtered_z

    global accel_x_b
    global accel_y_b
    global accel_z_b
    global accel_filtered_x
    global accel_filtered_y
    global accel_filtered_z

    global count

    
    imu = Imu()

    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = data.header.frame_id

    imu.orientation_covariance = data.orientation_covariance
    imu.angular_velocity_covariance = data.angular_velocity_covariance
    imu.linear_acceleration_covariance = data.linear_acceleration_covariance

#----------------------------low pass filter---------------------------------
    if count == 0 : 
         orientation_filtered_x = data.orientation.x
         orientation_filtered_y = data.orientation.y
         orientation_filtered_z = data.orientation.z
         orientation_filtered_w = data.orientation.w
         gyro_filtered_x = data.angular_velocity.x
         gyro_filtered_y = data.angular_velocity.y
         gyro_filtered_z = data.angular_velocity.z
         accel_filtered_x = data.linear_acceleration.x
         accel_filtered_y = data.linear_acceleration.y
         accel_filtered_z = data.linear_acceleration.z
         count += 1
    else : 
         orientation_filtered_x = orientation_filtered_x * (1-0.1) + 0.1 * orientation_x_b
         orientation_filtered_y = orientation_filtered_y * (1-0.1) + 0.1 * orientation_y_b
         orientation_filtered_z = orientation_filtered_z * (1-0.1) + 0.1 * orientation_z_b
         orientation_filtered_w = orientation_filtered_w * (1-0.1) + 0.1 * orientation_w_b
         gyro_filtered_x = gyro_filtered_x * (1-0.1) + 0.1 * gyro_x_b
         gyro_filtered_y = gyro_filtered_y * (1-0.1) + 0.1 * gyro_y_b
         gyro_filtered_z = gyro_filtered_z * (1-0.1) + 0.1 * gyro_z_b
         accel_filtered_x = accel_filtered_x * (1-0.1) + 0.1 * accel_x_b
         accel_filtered_y = accel_filtered_y * (1-0.1) + 0.1 * accel_y_b
         accel_filtered_z = accel_filtered_z * (1-0.1) + 0.1 * accel_z_b
         count = 10

    imu.orientation.x = orientation_filtered_x
    imu.orientation.y = orientation_filtered_y
    imu.orientation.z = orientation_filtered_z
    imu.orientation.w = orientation_filtered_w    

    imu.angular_velocity.x = gyro_filtered_x
    imu.angular_velocity.y = gyro_filtered_y
    imu.angular_velocity.z = gyro_filtered_z

    imu.linear_acceleration.x = accel_filtered_x
    imu.linear_acceleration.y = accel_filtered_y
    imu.linear_acceleration.z = accel_filtered_z

    imu_publisher.publish(imu)
#-----------------------------------------------------------------------------------------

if __name__=="__main__":
    
    rospy.init_node('imu_filter2')   
    rospy.Subscriber("/imu", Imu, imu_callback)
    imu_publisher = rospy.Publisher('/imu_data', Imu, queue_size=50)


    global orientation_x_b
    global orientation_y_b
    global orientation_z_b
    global orientation_w_b
    global orientation_filtered_x
    global orientation_filtered_y
    global orientation_filtered_z
    global orientation_filtered_w

    global gyro_x_b
    global gyro_y_b
    global gyro_z_b
    global gyro_filtered_x
    global gyro_filtered_y
    global gyro_filtered_z

    global accel_x_b
    global accel_y_b
    global accel_z_b
    global accel_filtered_x
    global accel_filtered_y
    global accel_filtered_z

    global count

    orientation_x_b = 0.0
    orientation_y_b = 0.0
    orientation_z_b = 0.0
    orientation_w_b = 0.0
    orientation_filtered_x = 0.0
    orientation_filtered_y = 0.0
    orientation_filtered_z = 0.0
    orientation_filtered_w = 0.0

    gyro_x_b = 0.0
    gyro_y_b = 0.0
    gyro_z_b = 0.0
    gyro_filtered_x = 0.0
    gyro_filtered_y = 0.0
    gyro_filtered_z = 0.0

    accel_x_b = 0.0
    accel_y_b = 0.0
    accel_z_b = 9.81
    accel_filtered_x = 0.0
    accel_filtered_y = 0.0
    accel_filtered_z = 0.0

    count = 0


    rate = rospy.Rate(50)
    rospy.loginfo("publishing new imu data")
    rospy.spin()
    rate.sleep()


