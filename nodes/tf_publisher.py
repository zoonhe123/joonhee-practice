#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import tf # tf broadcaster

from nav_msgs.msg import Odometry # 오도메트리 메시지



def odometry_callback(data) : 
    odom = Odometry()
            
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_footprint"

    # robot's position in x,y, and z
    odom.pose.pose.position.x = data.pose.pose.position.x
    odom.pose.pose.position.y = data.pose.pose.position.y
    odom.pose.pose.position.z = 0.0
    # robot's heading in quaternion
    odom.pose.pose.orientation.x = data.pose.pose.orientation.x
    odom.pose.pose.orientation.y = data.pose.pose.orientation.y
    odom.pose.pose.orientation.z = data.pose.pose.orientation.z
    odom.pose.pose.orientation.w = data.pose.pose.orientation.w

#    odom.pose.covariance[0] = 1e-3
#    odom.pose.covariance[7] = 1e-3
#    odom.pose.covariance[14] = 1e6
#    odom.pose.covariance[21] = 1e6
#    odom.pose.covariance[28] = 1e6
#    odom.pose.covariance[35] = 1e3
    odom.pose.covariance[0] = 0
    odom.pose.covariance[7] = 0
    odom.pose.covariance[14] = 0
    odom.pose.covariance[21] = 0
    odom.pose.covariance[28] = 0
    odom.pose.covariance[35] = 0

    # linear speed from encoders
    odom.twist.twist.linear.x = data.twist.twist.linear.x
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.linear.z = 0.0
    odom.twist.twist.angular.x = 0.0
    odom.twist.twist.angular.y = 0.0
    odom.twist.twist.angular.z = data.twist.twist.angular.z

#    odom.twist.covariance[0] = 1e-3
#    odom.twist.covariance[7] = 1e-3
#    odom.twist.covariance[14] = 1e6
#    odom.twist.covariance[21] = 1e3
#    odom.twist.covariance[28] = 1e6
#    odom.twist.covariance[35] = 1e3
    odom.twist.covariance[0] = 0
    odom.twist.covariance[7] = 0
    odom.twist.covariance[14] = 0
    odom.twist.covariance[21] = 0
    odom.twist.covariance[28] = 0
    odom.twist.covariance[35] = 0

    odom_publisher.publish(odom)



    br = tf.TransformBroadcaster()
    br.sendTransform((data.pose.pose.position.x , data.pose.pose.position.y , 0),
                     (data.pose.pose.orientation.x , data.pose.pose.orientation.y , data.pose.pose.orientation.z , data.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "base_footprint",
                     "odom")


if __name__=="__main__":
    
    rospy.init_node('tf_publisher')   
    rospy.Subscriber("/odometry/filtered", Odometry, odometry_callback)
    odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=50)

    rate = rospy.Rate(50)
    rospy.loginfo("publishing tf of odometry_filtered")
    rospy.spin()
    rate.sleep()


    
