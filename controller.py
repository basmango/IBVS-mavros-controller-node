#!/usr/bin/env python
import rospy
import sys
import numpy as np
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion,PoseStamped
from nav_msgs.msg import Odometry
import time
import math
import tf
from math import cos, exp, pi, sin



global V_Q,V_I, error, err_pub
err_pub = rospy.Publisher("/tracking_error", Float32MultiArray, queue_size=10)

error  = np.array([0,0,0,0,0,0],dtype=np.float32)

V_Q = np.array([0,0,0,0,0,0])

V_I = np.array([0,0,0,0,0,0])



def pose_callback(pose):
    global orientation
    orientation = pose.pose.orientation

def Feature_vec(data):
     
        # Parameters
        cu = 640
        cv = 360
        f =  954 # Pixels
#        K = np.array([0.4, 0.4, 0.23])

        K = np.array([0.41, 0.41, 0.4])
        k_yaw = -0.2

        global V_Q
        global orientation
        global V_I
        global error 
        # Image measurements of point features 
        s = data.data

        pt_star = np.array([592.00, 316.00, 682.00, 309.00, 688.00, 399.00, 599.00, 406.00]) #Desired pixel points location

        s_star = pt_star - np.array([cu,cv,cu,cv,cu,cv,cu,cv])

        #Virtual Plane Conversion
        vs1 = np.array([s[0]-cu, s[1]-cv])
        vs2 = np.array([s[2]-cu, s[3]-cv])
        vs3 = np.array([s[4]-cu, s[5]-cv])
        vs4 = np.array([s[6]-cu, s[7]-cv])    
     
        vs = np.array([vs1,vs2,vs3,vs4])#vs = np.array([vs1,vs2,vs3,vs4,vs5,vs6,vs7,vs8])
 
        # Image Moment Based Features 
        h_d = 0.98
        x_g = (vs1[0]+vs2[0]+vs3[0]+vs4[0])/4
        y_g = (vs1[1]+vs2[1]+vs3[1]+vs4[1])/4
        x_g_star = (s_star[0] + s_star[2] + s_star[4] + s_star[6])/4
        y_g_star = (s_star[1] + s_star[3] + s_star[5] + s_star[7])/4

        a = (np.square(vs1[0]-x_g) + np.square(vs1[1]-y_g)) + (np.square(vs2[0]-x_g) + np.square(vs2[1]-y_g)) + (np.square(vs3[0]-x_g) + np.square(vs3[1]-y_g)) + (np.square(vs4[0]-x_g) + np.square(vs4[1]-y_g)); 

        a_d = (np.square(s_star[0]-x_g_star) + np.square(s_star[1]-y_g_star)) + (np.square(s_star[2]-x_g_star) + np.square(s_star[3]-y_g_star)) + (np.square(s_star[4]-x_g_star) + np.square(s_star[5]-y_g_star)) + (np.square(s_star[6]-x_g_star) + np.square(s_star[7]-y_g_star));
        
        a_n = (h_d)*np.sqrt(a_d/a)
        x_n = (a_n/f)*x_g
        y_n = (a_n/f)*y_g
        s_v = np.array([x_n,y_n,a_n])
        a_n_star = h_d 
        x_n_star = (a_n_star/f)*x_g_star
        y_n_star = (a_n_star/f)*y_g_star
        s_v_star = np.array([x_n_star,y_n_star,a_n_star])
        
        e_v = np.subtract(s_v,s_v_star)




        # calculate arctan of orientation
        alpha = np.arctan2(vs1[1]-vs3[1],vs1[0]-vs3[0])


        alpha_star = np.arctan2(s_star[1]-s_star[5],s_star[0]-s_star[4])            
        

        heading_error = np.arctan2(np.sin(alpha-alpha_star), np.cos(alpha-alpha_star))

        
        V_c_body =  K*e_v

        V_I = V_c_body

        V_omega = np.array([0,0,k_yaw * heading_error])
                
        V_I = np.append(V_I,V_omega)

        V_Q = np.array([-V_I[1],-V_I[0],-V_I[2],0,0,k_yaw * heading_error])
        
        error = np.array([e_v[0],e_v[1],e_v[2],0,0,heading_error])     
        err = Float32MultiArray()
        err.data = error
        err_pub.publish(err)




def Controller():
    global V_Q
    global orientation
    global error
    rospy.init_node("IBVS_Control", anonymous=False)
    rospy.Subscriber("/aruco_coordinates", Int32MultiArray, Feature_vec)
    vel_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

    rate = rospy.Rate(50) # 25hz
    while not rospy.is_shutdown():
         data = V_Q #np.array([0,0,0,0,0,0])
         #print(data)
         msg = PositionTarget()
         msg.header.stamp = rospy.Time.now()
         msg.header.frame_id = "world"
         msg.coordinate_frame = 8
         msg.type_mask = 1991
         v_lin_max = 1
         v_lin_max_z = 0.6
         v_ang_max =pi/3
         if data[0]>v_lin_max:
                msg.velocity.x = v_lin_max
         elif data[0]<-v_lin_max:
                msg.velocity.x = -v_lin_max
         else:
                msg.velocity.x = data[0]
         if data[1]>v_lin_max:
                msg.velocity.y = v_lin_max
         elif data[1]<-v_lin_max:
                msg.velocity.y = -v_lin_max
         else:
                msg.velocity.y = data[1]
         if data[2]>v_lin_max_z:
                msg.velocity.z = v_lin_max_z
         elif data[2]<-v_lin_max_z:
                msg.velocity.z = -v_lin_max_z
         else:
                msg.velocity.z = data[2]
         if data[5]>v_ang_max:
                msg.yaw_rate = v_ang_max
         elif data[5]<-v_ang_max:
                msg.yaw_rate = -v_ang_max
         else:
                msg.yaw_rate = data[5]
       
         #vel_cmd.twist.angular.x = 0.0
         #vel_cmd.twist.angular.y = 0.0
         vel_pub.publish(msg)
         # publish tracking error
         
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
    
