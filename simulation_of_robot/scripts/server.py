#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from simulation_of_robot.msg import Encod

import datetime


def server_callback(data):
    imp1 = data.leftWheel
    imp2 = data.rightWheel
    message_id = data.header.frame_id

    rospy.loginfo(f'\nCaller id: {rospy.get_caller_id()}'
                  f'{message_id}'
                  f'\nGeting from robot left encoder: {imp1}'
                  f'\nGeting from robot rightencoder: {imp2}'
                  f'\nReceiving time GMT:             {datetime.datetime.now()}'
                  f'\nReceiving Unix epoch time:      {rospy.Time.now()}\n')


def listen_to_robot():
    rospy.Subscriber('RobotToServer', Encod, server_callback)
    rospy.Subscriber('DebugToServer', Pose2D)
    rospy.spin()


def send_to_robot(v, omega):
    server_publisher = rospy.Publisher('ServerToRobot', Twist, queue_size=10)

    vel = Twist()
    vel.linear.x = v
    vel.angular.z = omega

    current_time = rospy.Time.now()
    frequency = rospy.Rate(10)
    while rospy.Time.now() < current_time + rospy.Duration.from_sec(0.2):
        server_publisher.publish(vel)
        frequency.sleep()

    rospy.loginfo(f"\nSending to robot linear velocity: {v}(m/s)"
                  f"\nSending to robot angular velicty: {omega}(rad/s)"
                  f"\nDispatching GMT time :            {datetime.datetime.now()}"
                  f"\nDispatching Unix epoch time:      {rospy.Time.now()} \n")


if __name__ == "__main__":
    rospy.init_node('server', anonymous=True)

    send_to_robot(1, -0.5)  # Таргетные значения линейной и угловой скоростей робота
    listen_to_robot()
