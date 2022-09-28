#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pow
from sensor_msgs.msg import Imu # 바퀴 조인트 상태 메시지


def imu_callback(data) : 

    global orientation_x_b
    global orientation_y_b
    global orientation_z_b
    global orientation_w_b

    global gyro_x_b
    global gyro_y_b
    global gyro_z_b

    global accel_x_b
    global accel_y_b
    global accel_z_b


    imu = Imu()

    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = data.header.frame_id

    imu.orientation_covariance = data.orientation_covariance
    imu.angular_velocity_covariance = data.angular_velocity_covariance
    imu.linear_acceleration_covariance = data.linear_acceleration_covariance

#--------------------------------------------------------------    
    if (pow(data.orientation.x,2) + pow(data.orientation.y,2) + pow(data.orientation.z,2) + pow(data.orientation.w,2)) > 1.2 : 
         imu.orientation.x = orientation_x_b
         imu.orientation.y = orientation_y_b
         imu.orientation.z = orientation_z_b
         imu.orientation.w = orientation_w_b

    else : 

         imu.orientation.x = data.orientation.x
         imu.orientation.y = data.orientation.y
         imu.orientation.z = data.orientation.z
         imu.orientation.w = data.orientation.w

         orientation_x_b = imu.orientation.x
         orientation_y_b = imu.orientation.y
         orientation_z_b = imu.orientation.z
         orientation_w_b = imu.orientation.w

    quat_1 = (imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w)
    euler_angle = euler_from_quaternion(quat_1)

    quat = quaternion_from_euler(euler_angle[0], -euler_angle[1], -euler_angle[2])

    imu.orientation.x = quat[0]
    imu.orientation.y = quat[1]
    imu.orientation.z = quat[2]
    imu.orientation.w = quat[3]      

#---------------------------------------------------------------

    if abs(data.angular_velocity.x) > 0.7 or abs(data.angular_velocity.y) > 0.7 or abs(data.angular_velocity.z) > 2.84 : 
         imu.angular_velocity.x = gyro_x_b
         imu.angular_velocity.y = gyro_y_b
         imu.angular_velocity.z = gyro_z_b

    else : 
         imu.angular_velocity.x = data.angular_velocity.x
         imu.angular_velocity.y = 0
         imu.angular_velocity.z = data.angular_velocity.y + data.angular_velocity.z

         gyro_x_b = imu.angular_velocity.x
         gyro_y_b = imu.angular_velocity.y
         gyro_z_b = imu.angular_velocity.z
#-------------------------------------------------------------

    if abs(data.linear_acceleration.x) > 0.6 or abs(data.linear_acceleration.y) > 0.6 or abs(data.linear_acceleration.z) > 12 or  abs(data.linear_acceleration.z) < 9 : 
         imu.linear_acceleration.x = accel_x_b
         imu.linear_acceleration.y = accel_y_b
         imu.linear_acceleration.z = accel_z_b

    else : 
         imu.linear_acceleration.x = data.linear_acceleration.x
         imu.linear_acceleration.y = data.linear_acceleration.y
         imu.linear_acceleration.z = data.linear_acceleration.z

         accel_x_b = imu.linear_acceleration.x
         accel_y_b = imu.linear_acceleration.y
         accel_z_b = imu.linear_acceleration.z

    imu_publisher.publish(imu)


if __name__=="__main__":
    
    rospy.init_node('imu_filter')   
    rospy.Subscriber("/imu", Imu, imu_callback)
    imu_publisher = rospy.Publisher('/imu_data', Imu, queue_size=50)


    global orientation_x_b
    global orientation_y_b
    global orientation_z_b
    global orientation_w_b

    global gyro_x_b
    global gyro_y_b
    global gyro_z_b

    global accel_x_b
    global accel_y_b
    global accel_z_b


    orientation_x_b = 0
    orientation_y_b = 0
    orientation_z_b = 0
    orientation_w_b = 0

    gyro_x_b = 0
    gyro_y_b = 0
    gyro_z_b = 0

    accel_x_b = 0
    accel_y_b = 0
    accel_z_b = 9.81



    rate = rospy.Rate(50)
    rospy.loginfo("publishing new imu data")
    rospy.spin()
    rate.sleep()
