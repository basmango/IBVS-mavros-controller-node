# generate python code for this bash command:
# rostopic pub /mavros/setpoint_raw/local mavros_msgs/PositionTarget '{header: {stamp: now, frame_id: "world"}, coordinate_frame: 8, type_mask: 1991, velocity: {x: 0, y: 0, z: 0}, yaw_rate: 30}' -r 10

#!/usr/bin/env python3
# Compare this snippet from controller.py:
#     err_pub = rospy.Publisher("/tracking_error", Float32MultiArray, queue_size=10)
#     vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
#
#     #vel_pub = rospy.Publisher("/hector/cmd_vel", Twist, queue_size=10)
#     rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
#     rate = rospy.Rate(60) # 25hz
#     while not rospy.is_shutdown():
#          data = V_Q #np.array([0,0,0,0,0,0])
#          #print(data)
#          vel_cmd = TwistStamped()
#          v_lin_max = 1
#          v_lin_max_z = 0.6
#          v_ang_max = pi/3
#          if data[0]>v_lin_max:
#                 vel_cmd.twist.linear.x = v_lin_max
#          elif data[0]<-v_lin_max:
#                 vel_cmd.twist.linear.x = -v_lin_max
#          else:
#                 vel_cmd.twist.linear.x = data[0]
#          if data[1]>v_lin_max:
#                 vel_cmd.twist.linear.y = v_lin_max