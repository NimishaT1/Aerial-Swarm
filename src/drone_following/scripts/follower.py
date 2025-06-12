#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool

target_pose = PoseStamped()

def leader_cb(msg):
    # Offset to follow behind and above
    target_pose.pose.position.x = msg.pose.position.x - 2.0
    target_pose.pose.position.y = msg.pose.position.y
    target_pose.pose.position.z = msg.pose.position.z + 1.0
    target_pose.header.stamp = rospy.Time.now()

def main():
    rospy.init_node('drone_follower')

    rospy.Subscriber('/drone_a/mavros/local_position/pose', PoseStamped, leader_cb)
    pub = rospy.Publisher('/drone_b/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    rospy.wait_for_service('/drone_b/mavros/cmd/arming')
    rospy.wait_for_service('/drone_b/mavros/set_mode')
    arming = rospy.ServiceProxy('/drone_b/mavros/cmd/arming', CommandBool)
    set_mode = rospy.ServiceProxy('/drone_b/mavros/set_mode', SetMode)

    rate = rospy.Rate(20)
    for _ in range(100):
        target_pose.header.stamp = rospy.Time.now()
        pub.publish(target_pose)
        rate.sleep()

    set_mode(0, "OFFBOARD")
    arming(True)

    while not rospy.is_shutdown():
        target_pose.header.stamp = rospy.Time.now()
        pub.publish(target_pose)
        rate.sleep()

if __name__ == "__main__":
    main()
