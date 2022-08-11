#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy

from sensor_msgs.msg import Imu # 바퀴 조인트 상태 메시지


def imu_callback(data) : 
    
    imu = Imu()

    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = data.header.frame_id

    imu.orientation.x = data.orientation.x
    imu.orientation.y = data.orientation.y
    imu.orientation.z = data.orientation.z
    imu.orientation.w = data.orientation.w
    imu.orientation_covariance = data.orientation_covariance

    imu.angular_velocity.x = data.angular_velocity.z
    imu.angular_velocity.y = -data.angular_velocity.x
    imu.angular_velocity.z = -data.angular_velocity.y
    imu.angular_velocity_covariance = data.angular_velocity_covariance

    imu.linear_acceleration.x = data.linear_acceleration.z
    imu.linear_acceleration.y = -data.linear_acceleration.x
    imu.linear_acceleration.z = -data.linear_acceleration.y
    imu.linear_acceleration_covariance = data.linear_acceleration_covariance

    imu_publisher.publish(imu)


if __name__=="__main__":
    
    rospy.init_node('imu_frame_changer')   
    rospy.Subscriber("/camera/imu", Imu, imu_callback)
    imu_publisher = rospy.Publisher('/raw_imu', Imu, queue_size=50)

    rate = rospy.Rate(50)
    rospy.loginfo("publishing new imu data")
    rospy.spin()
    rate.sleep()


    
