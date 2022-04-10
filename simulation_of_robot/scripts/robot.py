#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from simulation_of_robot.msg import Encod

from math import pi
import matplotlib.pyplot as plt
import datetime
import numpy as np

time = []  # time
message_number = []  # index

wheels_velocity = []  # w1, w2 (rad/s)
lin_velocity_of_robot = []  # V (m/s)
ang_velocity_of_robot = []  # omega (rad/s)
angle_of_robot = [0]  # theta (rad)

real_x_of_robot = [0]  # x (m)
real_y_of_robot = [0]  # y (m)


#                   Formulas

def angular_velocity(w1_prev, w2_prev, w1_target, w2_target):
    # B = dt / (T + dt) B = [0:1]

    dt = 0.1  # dt - шаг интегрирования
    T = 0.01  # T - постоянная времени

    B = dt / (T + dt)  # B - сглаживающий коэффициент

    # w[n] = Bw[n-1]+(1-B)wtarget

    w1 = B * w1_prev + (1 - B) * w1_target  # w1, w2 - угловые скорости колеса на текущей и предыдущей итерациях
    w2 = B * w2_prev + (1 - B) * w2_target  # w1_prev, w2_prev - угловые скорости колеса на и предыдущей итерациях

    return w1, w2


def wheel_velocity(V, omega):
    # ICC - мгновенный центр скоростей
    # w - угловая скорость вращения центра робота вокруг МЦС
    # V - линейная скорость робота
    # w_left, w_right - угловые скорости колес

    L = 0.287  # L - расстояние между колесами
    r = 0.033  # r - радиус колес

    R = V / omega  # R - расстояние от МЦС до центра робота

    l = R - L / 2  # расстояние от левого колеса до МЦС
    w_left = omega * l / r  # угловая скорость левого колеса
    l = R + L / 2  # расстояние от правого колеса до МЦС
    w_right = omega * l / r  # угловая скорость правого колеса

    return w_left, w_right


def velocity_to_encoders_impulse(w1, w2):
    # кол-во оборотов = W(рад/с) * deltaТ(с) / 2*pi,
    # где deltaT - время между отправкой сообщений
    # если частота отправки сообщений 10Гц, то deltaT = 0.1(сек)

    n1 = w1 * 0.1 / (2 * pi)
    n2 = w2 * 0.1 / (2 * pi)

    # кол-во оборотов * разрешение энкодеров на 1 оборот = кол-во импульсов энкодера
    imp1 = int(n1 * 4096)
    imp2 = int(n2 * 4096)

    return imp1, imp2


def get_coordinates(w1, w2, measure_step, ros_time):
    r = 0.033
    L = 0.287

    time.append(ros_time)
    message_number.append(measure_step)

    v = r / 2 * (w1 + w2)  # (m/s)
    lin_velocity_of_robot.append(v)

    omega = r / L * (w2 - w1)
    ang_velocity_of_robot.append(omega)

    if len(time) > 1:
        theta = integrate_theta()
        x = integrate_x()
        y = integrate_y()

        send_debug_to_server(x, y, theta)


def integrate_theta():  # угол, на который повернут робот
    dt = 0.1
    temp = (ang_velocity_of_robot[-1] + ang_velocity_of_robot[-2]) * dt / 2
    theta = temp + angle_of_robot[-1]
    angle_of_robot.append(theta)
    return theta


def integrate_x():
    dt = 0.1
    temp = (lin_velocity_of_robot[-1] * np.cos(angle_of_robot[-1]) + lin_velocity_of_robot[-2] * np.cos(
        angle_of_robot[-2])) * dt / 2
    x = temp + real_x_of_robot[-1]
    real_x_of_robot.append(x)
    return x


def integrate_y():
    dt = 0.1
    temp = (lin_velocity_of_robot[-1] * np.sin(angle_of_robot[-1]) + lin_velocity_of_robot[-2] * np.sin(
        angle_of_robot[-2])) * dt / 2
    y = temp + real_y_of_robot[-1]
    real_y_of_robot.append(y)
    return y


#                   Messages

def send_encoder_to_server(w1_target, w2_target):
    encoder_publisher = rospy.Publisher('RobotToServer', Encod, queue_size=10)

    w1_prev = w2_prev = 0
    imp1_prev = imp2_prev = 0
    msg_num = 0  # номер сообщения

    current_time = rospy.Time.now()  # текущее время
    frequency = rospy.Rate(10)  # частота отправки сообщения

    while rospy.Time.now() < current_time + rospy.Duration.from_sec(10):  # 10 сек

        w1, w2 = angular_velocity(w1_prev, w2_prev, w1_target, w2_target)

        imp1, imp2 = velocity_to_encoders_impulse(w1, w2)

        msg = Encod()
        msg.leftWheel = imp1 + imp1_prev
        msg.rightWheel = imp2 + imp2_prev

        string_msg_num = f"\n           Mesurment number {msg_num}"  # вывод номера сообщения

        msg.header.stamp = rospy.Time.now()
        msg.header.seq = msg_num
        msg.header.frame_id = string_msg_num

        try:
            encoder_publisher.publish(msg)
        except rospy.ROSInterruptException:
            continue

        rospy.loginfo(f"{string_msg_num}"
                      f"\nSending to server left encoder:  {imp1}"
                      f"\nSending to server right encoder: {imp2} "
                      f"\nDispatching GTM time:            {datetime.datetime.now()}"
                      f"\nDispatching Unix epoch time:     {msg.header.stamp} \n")

        get_coordinates(w1, w2, msg.header.seq, msg.header.stamp)

        msg_num += 1
        w1_prev, w2_prev = w1, w2
        imp1_prev, imp2_prev = imp1, imp2

        frequency.sleep()


def send_debug_to_server(x, y, theta):
    debug_publisher = rospy.Publisher('DebugToServer', Pose2D, queue_size=10)

    position = Pose2D()

    position.x = x
    position.y = y
    position.theta = theta

    debug_publisher.publish(position)

    rospy.loginfo(f'\n           Sending coordinates to server (debug)'
                  f'\nx = {x}(m) '
                  f'\ny = {y}(m)'
                  f'\nθ = {theta}(rad)'
                  f'\nDispathing GMT time:             {datetime.datetime.now()}'
                  f'\nDispathing Unix epoch time:      {rospy.Time.now()}\n')


def robot_callback(data):
    v = data.linear.x
    omega = data.angular.z

    rospy.loginfo(f'\nCaller id: {rospy.get_caller_id()}'
                  f'\nGetting from server linear velocity: {v}(m/s)'
                  f'\nGetting from server angular velicty: {omega}(rad/s)'
                  f'\nGetting GTM time:                    {datetime.datetime.now()}'
                  f'\nGetting Unix epoch time:             {rospy.Time.now()} \n')

    wl_target, wr_target = wheel_velocity(v, omega)

    send_encoder_to_server(wl_target, wr_target)


def listen_to_server():
    rospy.init_node('robot', anonymous=True)
    rospy.Subscriber('ServerToRobot', Twist, robot_callback)
    rospy.spin()


#                   Plots

def trajectory(x_r, y_r):
    plt.figure()
    plt.subplot(1, 2, 1)

    plt.plot(x_r, y_r, 'r')
    plt.title("Real trajectory")
    plt.grid()

    ideal_v = lin_velocity_of_robot[-1]
    ideal_omega = ang_velocity_of_robot[-1]
    x_target = [0]
    y_target = [0]
    dt = 0.1
    for i in range(len(time) - 1):
        ideal_theta = ideal_omega * dt * i
        x = ideal_v * np.cos(ideal_theta) * dt
        x_target.append(x + x_target[-1])
        y = ideal_v * np.sin(ideal_theta) * dt
        y_target.append(y + y_target[-1])

    plt.subplot(1, 2, 2)
    plt.plot(x_target, y_target, 'g')
    plt.title("Target trajectory")
    plt.grid()
    plt.show()


if __name__ == "__main__":
    listen_to_server()

    trajectory(real_x_of_robot, real_y_of_robot)
