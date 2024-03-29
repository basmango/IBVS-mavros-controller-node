#!/usr/bin/env python
import rospy
import sys
import numpy as np
from std_msgs.msg import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Quaternion, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import time
import math
import tf
from math import cos, exp, pi, sin


global V_Q, V_I, error, velocity, e_fs_prev, h_s_prev, e_fpsi_prev, h_psi_prev, est_vel1
global orientation
global est_vel_puba


error = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
velocity = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
est_vel1 = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
global ev_0, e_alpha0
global time_now, time_prev
ev_0 = None
e_alpha0 = None
orientation = Quaternion()

V_Q = np.array([0, 0, 0, 0, 0, 0])

V_I = np.array([0, 0, 0, 0, 0, 0])

e_fs_prev = 0

h_s_prev = 0

e_fpsi_prev = 0

h_psi_prev = 0

time_now = 0

time_prev = 0


def R_x(theta):
    R_phi = np.vstack(
        ([1, 0, 0], [0, cos(theta), -sin(theta)], [0, sin(theta), cos(theta)])
    )

    return R_phi


def R_y(theta):
    R_theta = np.vstack(
        ([cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)])
    )

    return R_theta


def R_z(theta):
    R_psi = np.vstack(
        ([cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1])
    )

    return R_psi


def velocity_callback(data):
    global velocity
    # data is in twist stamped format, extract linear velocity,angular velocity and print
    velocity_east, velocity_north, velocity_up = (
        data.twist.linear.x,
        data.twist.linear.y,
        data.twist.linear.z,
    )
    angular_velocity_x, angular_velocity_y, angular_velocity_z = (
        data.twist.angular.x,
        data.twist.angular.y,
        data.twist.angular.z,
    )
    velocity = np.array(
        [
            velocity_east,
            velocity_north,
            velocity_up,
            angular_velocity_x,
            angular_velocity_y,
            angular_velocity_z,
        ]
    )
    # print("velocity ",velocity)


def pose_callback(pose):
    global orientation
    orientation = pose.pose.orientation


def Feature_vec(data):
    # Parameters
    cu = 640
    cv = 360
    f = 595.55  # Pixels
    K = np.array([0.45, 0.45, 0.45])
    k_yaw = 0.4
    dT = 1 / 50
    gamma1 = 0.7
    gamma2 = 0.7
    global V_Q
    global orientation
    global V_I
    global error, est_vel1
    global velocity
    global e_fs_prev, e_fpsi_prev, h_s_prev, h_psi_prev
    global ev_0, e_alpha0
    global time_now, time_prev

    time_now = rospy.get_time()

    dT = time_now - time_prev

    if time_prev == 0:
        dT = 0

    time_prev = time_now

    q = orientation

    # Image measurements of point features
    s = data.data
    pt_star = np.array(
        [619.0, 340.0, 660.0, 339.0, 662.0, 381.0, 620.0, 382.0]
    )  # Desired pixel points location
    s_star = pt_star - np.array([cu, cv, cu, cv, cu, cv, cu, cv])
    # Virtual Plane Conversion
    vs1 = np.array([s[0] - cu, s[1] - cv])
    vs2 = np.array([s[2] - cu, s[3] - cv])
    vs3 = np.array([s[4] - cu, s[5] - cv])
    vs4 = np.array([s[6] - cu, s[7] - cv])

    vs = np.array(
        [vs1, vs2, vs3, vs4]
    )  # vs = np.array([vs1,vs2,vs3,vs4,vs5,vs6,vs7,vs8])

    # Image Moment Based Features
    h_d = 2.04
    x_g = (vs1[0] + vs2[0] + vs3[0] + vs4[0]) / 4
    y_g = (vs1[1] + vs2[1] + vs3[1] + vs4[1]) / 4
    x_g_star = (s_star[0] + s_star[2] + s_star[4] + s_star[6]) / 4
    y_g_star = (s_star[1] + s_star[3] + s_star[5] + s_star[7]) / 4
    a = (
        (np.square(vs1[0] - x_g) + np.square(vs1[1] - y_g))
        + (np.square(vs2[0] - x_g) + np.square(vs2[1] - y_g))
        + (np.square(vs3[0] - x_g) + np.square(vs3[1] - y_g))
        + (np.square(vs4[0] - x_g) + np.square(vs4[1] - y_g))
    )
    a_d = (
        (np.square(s_star[0] - x_g_star) + np.square(s_star[1] - y_g_star))
        + (np.square(s_star[2] - x_g_star) + np.square(s_star[3] - y_g_star))
        + (np.square(s_star[4] - x_g_star) + np.square(s_star[5] - y_g_star))
        + (np.square(s_star[6] - x_g_star) + np.square(s_star[7] - y_g_star))
    )

    a_n = (h_d) * np.sqrt(a_d / a)
    x_n = (a_n / f) * x_g
    y_n = (a_n / f) * y_g
    s_v = np.array([x_n, y_n, a_n])
    a_n_star = h_d
    x_n_star = (a_n_star / f) * x_g_star
    y_n_star = (a_n_star / f) * y_g_star
    s_v_star = np.array([x_n_star, y_n_star, a_n_star])

    e_v = np.subtract(s_v, s_v_star)

    R_V_I = np.matmul(
        R_z(tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]),
        np.matmul(R_x(pi), R_z(pi / 2)),
    )

    # calculate arctan of orientation
    alpha = np.arctan2(vs1[1] - vs3[1], vs1[0] - vs3[0])
    alpha_star = np.arctan2(s_star[1] - s_star[5], s_star[0] - s_star[4])

    heading_error = np.arctan2(np.sin(alpha - alpha_star), np.cos(alpha - alpha_star))

    if ev_0 is None:
        ev_0 = e_v

    if e_alpha0 is None:
        e_alpha0 = heading_error

    # Filter signal for Linear Velocity
    lin_vel = np.array([velocity[0], velocity[1], velocity[2]])
    Sk_psi_dot = np.vstack(([0, -velocity[5], 0], [velocity[5], 0, 0], [0, 0, 0]))

    # Velocity Estimation
    v_p_bar = h_s_prev + (1 / gamma1) * np.subtract(
        e_v, e_fs_prev + exp(-(1 / gamma1) * time_now) * ev_0
    )

    # v_p_bar[2] = 0.0

    # if dT == 0:
    #       print()
    # e_v is 3x1 error vector
    V_c_body = K * e_v + v_p_bar

    # apply rotation matrix to V_c_body
    V_I = np.matmul(R_V_I, V_c_body)

    # Filter signal for Angular Velocity
    h_psi = -velocity[5]

    # Dynamic Surface Filter

    psi_p_bar = h_psi_prev + (1 / gamma2) * np.subtract(
        heading_error, e_fpsi_prev + exp(-(1 / gamma2) * time_now) * e_alpha0
    )

    V_omega = np.array([0, 0, -k_yaw * heading_error - psi_p_bar])

    V_I = np.append(V_I, V_omega)

    V_Q = np.array([V_I[0], V_I[1], V_I[2], 0, 0, V_I[5]])

    # make 6x1 array for errors
    est_vel = np.matmul(R_V_I, v_p_bar)
    est_vel1 = np.array(est_vel, dtype=np.float32)
    est_vel1 = np.append(est_vel1, psi_p_bar)

    error = np.array([e_v[0], e_v[1], e_v[2], 0, 0, heading_error])
    print(error)
    # print(est_vel1)
    alpha1 = exp(-(1 / gamma1) * dT)
    # v_bar = K*e_v + v_p_bar
    v_bar = np.matmul(np.transpose(R_V_I), lin_vel)
    h = v_bar + np.matmul(Sk_psi_dot, e_v)

    e_fs_prev = alpha1 * e_fs_prev + (1 - alpha1) * e_v
    h_s_prev = alpha1 * h_s_prev + (1 - alpha1) * h

    alpha2 = exp((-1 / gamma2) * dT)
    e_fpsi_prev = alpha2 * e_fpsi_prev + (1 - alpha2) * heading_error
    h_psi_prev = alpha2 * h_psi_prev + (1 - alpha2) * h_psi


def Controller():
    global V_Q
    global orientation
    global error
    global est_vel_pub, est_vel1
    rospy.init_node("IBVS_Control", anonymous=False)
    rospy.Subscriber("/aruco_coordinates", Int32MultiArray, Feature_vec)
    err_pub = rospy.Publisher("/tracking_error", Float32MultiArray, queue_size=3)
    vel_pub = rospy.Publisher(
        "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=3
    )
    est_vel_pub = rospy.Publisher(
        "/estimated_velocity", Float32MultiArray, queue_size=5
    )

    # vel_pub = rospy.Publisher("/hector/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber(
        "/mavros/local_position/velocity_local", TwistStamped, velocity_callback
    )
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
    rate = rospy.Rate(30)  # 25hz
    while not rospy.is_shutdown():
        data = V_Q  # np.array([0,0,0,0,0,0])
        # print(data)
        vel_cmd = Twist()
        v_lin_max = 0.5
        v_lin_max_z = 0.2
        v_ang_max = pi / 4
        if data[0] > v_lin_max:
            vel_cmd.linear.x = v_lin_max
        elif data[0] < -v_lin_max:
            vel_cmd.linear.x = -v_lin_max
        else:
            vel_cmd.linear.x = data[0]
        if data[1] > v_lin_max:
            vel_cmd.linear.y = v_lin_max
        elif data[1] < -v_lin_max:
            vel_cmd.linear.y = -v_lin_max
        else:
            vel_cmd.linear.y = data[1]
        if data[2] > v_lin_max_z:
            vel_cmd.linear.z = v_lin_max_z
        elif data[2] < -v_lin_max_z:
            vel_cmd.linear.z = -v_lin_max_z
        else:
            vel_cmd.linear.z = data[2]
        if data[5] > v_ang_max:
            vel_cmd.angular.z = v_ang_max
        elif data[5] < -v_ang_max:
            vel_cmd.angular.z = -v_ang_max
        else:
            vel_cmd.angular.z = data[5]

        # vel_cmd.twist.angular.x = 0.0
        # vel_cmd.twist.angular.y = 0.0
        vel_pub.publish(vel_cmd)
        # publish tracking error
        err = Float32MultiArray()
        err.data = error
        err_pub.publish(err)
        est_vel = Float32MultiArray()
        est_vel.data = est_vel1
        est_vel_pub.publish(est_vel)

        rate.sleep()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    try:
        Controller()
    except rospy.ROSInterruptException:
        pass
