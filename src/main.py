#!/usr/bin/python2.7
import rospy
from geometry_msgs.msg import Twist
from math import atan2, pi
from turtlesim.msg import Pose


def msg(x, z):
    twist = Twist()
    twist.linear.x = x
    twist.angular.z = z
    return twist


class PoseEstimation:
    current_pose = Pose()
    target_pose = Pose()

    def update_current(cls, new_pose):
        cls.current_pose = new_pose

    def update_target(cls, new_pose):
        cls.target_pose = new_pose

    def find_distance(cls):
        delta = cls.target_pose.x - cls.current_pose.x, cls.target_pose.y - cls.current_pose.y
        return (delta[0]**2 + delta[1]**2)**0.5

    def find_angle(cls):
        return (atan2(cls.target_pose.y - cls.current_pose.y, cls.target_pose.x - cls.current_pose.x) - cls.current_pose.theta + pi) % (2*pi) - pi

    def __str__(cls):
        return "current_pose x, y: {} {}, target_pose x, y: {} {}, distance: {}, angle: {}".format(cls.current_pose.x, cls.current_pose.y, cls.target_pose.x, cls.target_pose.y, cls.find_distance(), cls.find_angle())


if __name__ == "__main__":

    min_distance = 1

    rospy.init_node('turtle_follower')
    rate = rospy.Rate(2)

    pos = PoseEstimation()
    
    pub = rospy.Publisher('/turtle_follower/cmd_vel', Twist, queue_size=1)
    sub_follower = rospy.Subscriber(
        '/turtle_follower/pose', Pose, pos.update_current)
    sub_following = rospy.Subscriber(
        '/turtle1/pose', Pose, pos.update_target)

    while not rospy.is_shutdown():
        dist = pos.find_distance()
        rospy.loginfo(str(pos))
        if dist >= min_distance:
            angle = pos.find_angle()
            tw = msg(dist, angle)
            pub.publish(tw)
        rate.sleep()
