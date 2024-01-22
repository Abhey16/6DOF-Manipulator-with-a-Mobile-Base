#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import sys
import select
import tty
import termios
import math
from statistics import mean
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time 

class IMUSubscriber(Node):
        # ... (rest of the IMUSubscriber class code from the first script)
        # super().__init__('proportional_controller')
    def __init__(self):

        super().__init__('proportional_controller')

        qos_profile = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT, history = HistoryPolicy.KEEP_LAST, depth = 10)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.imu_subscription = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback,qos_profile)
        self.joint_state_subscription = self.create_subscription(JointState,'joint_states', self.joint_state_callback, qos_profile)

        self.current_pos = [0,0]
        self.target_pos = [10,10]

        self.kp = 0.2
        self.control_output = 0.0
        self.yaw = 0.0

        self.time_period = 0.5
        self.timer = self.create_timer(self.time_period, self.publish_commands)

    def imu_callback(self, msg):
        quat_data = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w]
        (roll,pitch, self.yaw) = euler_from_quaternion(quat_data)
        # self.yaw -= 1.5707

    def joint_state_callback(self, msg):
        self.AvgAng_velocity= (msg.velocity[4]+msg.velocity[5])/2
        linear_velocity= 0.1015*self.AvgAng_velocity

        self.current_pos[0]+= linear_velocity*(math.cos(self.yaw))*self.time_period
        self.current_pos[1]+= linear_velocity*(math.sin(self.yaw))*self.time_period

        delta_x = self.target_pos[0] - self.current_pos[0]
        delta_y = self.target_pos[1] - self.current_pos[1]

        self.desired_yaw = np.arctan2(delta_y, delta_x)

        
    def publish_commands(self):
        error = self.desired_yaw - self.yaw
        print(self.yaw)
        Control_output = self.kp*error

        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()

        linear_vel = 0
        wheel_velocities.data = [0.0,0.0,linear_vel,-linear_vel]
        joint_positions.data = [Control_output,Control_output]

        # self.joint_position_pub.publish(joint_positions)
        # self.wheel_velocities_pub.publish(wheel_velocities)

class KinematicsController(Node):
    def __init__(self):
        super().__init__('kinematics_controller')

        qos_profile = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT, history = HistoryPolicy.KEEP_LAST, depth = 10)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        thetarad, alpharad, a, d = sp.symbols('thetadeg alphadeg a d')
        theta1, theta2, theta3, theta4, theta5, theta6 = sp.symbols('theta1 theta2 theta3 theta4 theta5 theta6')

        def Transformation_matrix(thetarad, alpharad, a, d):
            return sp.Matrix([
                [sp.cos(thetarad), -sp.sin(thetarad) * sp.cos(alpharad), sp.sin(thetarad) * sp.sin(alpharad), a * sp.cos(thetarad)],
                [sp.sin(thetarad), sp.cos(thetarad) * sp.cos(alpharad), -sp.cos(thetarad) * sp.sin(alpharad), a * sp.sin(thetarad)],
                [0, sp.sin(alpharad), sp.cos(alpharad), d],
                [0, 0, 0, 1]
            ])

        # Transformation for each frame (1 to 6)
        def T0():
            return Transformation_matrix(0, 0, 0, 334)

        def T1(thetadeg1):
            return Transformation_matrix(thetadeg1, sp.pi/2, 150, 160)
        
        T10 = T0() * T1(theta1)
        
        def Td1():
            return Transformation_matrix(0, 0, 0, 20)
        
        Td10 = T0() * T1(theta1)*Td1()

        def T2(thetadeg2):
            return Transformation_matrix(thetadeg2, 0, 9.61, 0)

        T20 = T0() * T1(theta1)*Td1()* T2(theta2)

        def Td2():
            return Transformation_matrix(sp.pi/2, 0, 349.87, 0)

        Td20 = T0() * T1(theta1)*Td1()* T2(theta2)*Td2()


        def T3(thetadeg3):
            return Transformation_matrix(thetadeg3, 90, 45, 0)

        T30 = T0() * T1(theta1)*Td1()* T2(theta2)*Td2()* T3(theta3)


        def Td3():
            return Transformation_matrix(90, 0, -48, 246)

        Td30 = T0() * T1(theta1)*Td1()* T2(theta2)*Td2()* T3(theta3)*Td3()

        def T4(thetadeg4):
            return Transformation_matrix((thetadeg4 + sp.pi/2), sp.pi/2, 0, 115)

        T40 = T0() * T1(theta1)*Td1()* T2(theta2)*Td2()* T3(theta3)*Td3()* T4(theta4)

        def T5(thetadeg5):
            return Transformation_matrix((thetadeg5 + sp.pi/2),90, 0, 0)

        T50 = T0() * T1(theta1)*Td1()* T2(theta2)*Td2()* T3(theta3)*Td3()* T4(theta4) * T5(theta5)

        def Td4():
            return Transformation_matrix(0,0, 0, 94)

        Td40 = T0() * T1(theta1)*Td1()* T2(theta2)*Td2()* T3(theta3)*Td3()* T4(theta4) * T5(theta5)*Td4()


        def T6(thetadeg6):
            return Transformation_matrix(thetadeg6, 0, 0, 116.67)

        # Final Transformation Martix
        T = T0()*T1(theta1)*Td1()* T2(theta2)*Td2()* T3(theta3) *Td3()* T4(theta4) * T5(theta5)*Td4()* T6(theta6)

        # Positoion of End Effector Tool
        P_f = T[:3, -1]

        # Finding Partial Derivatives 

        dP_dtheta1 = P_f.diff(theta1)
        dP_dtheta2 = P_f.diff(theta2)
        dP_dtheta3 = P_f.diff(theta3)
        dP_dtheta4 = P_f.diff(theta4)
        dP_dtheta5 = P_f.diff(theta5)
        dP_dtheta6 = P_f.diff(theta6)

        # Finding Z axis for each joint

        Z1 = T10[:3, 2]
        Z2 = T20[:3, 2]
        Z3 = T30[:3, 2]
        Z4 = T40[:3, 2]
        Z5 = T50[:3, 2]
        Z6 = T[:3, 2]

        # Jacobian matrix
        J = sp.Matrix([
            [dP_dtheta1, dP_dtheta2, dP_dtheta3, dP_dtheta4, dP_dtheta5, dP_dtheta6],
            [Z1, Z2, Z3, Z4, Z5, Z6]
        ])

        # Printing Jacobian Matrix
        # sp.pprint(J)

        # Initial joint angles

        # Starting plot in 3D
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')

        # Required circle trajectory
        # required_trajectory = []

        # Output circle points
        # output_trajectory = []
        q = sp.Matrix([0.000054, 0.000055, 0, 0.000055, 0.000055, 0])


        for i_val in range(0, 100):
            print("calculating joint angles")
            # x coordinate of circle drawn by end effector tool
            x = 100 * sp.sin((2 * np.pi / 100) * i_val)  
            x_dot = 100 * sp.cos((2 * np.pi / 100) * i_val) * (2 * np.pi / 20)

            # z coordinate of circle drawn by end effector tool
            z = 1328 + 100 * sp.cos((2 * np.pi / 100) * i_val)
            z_dot = -100 * sp.sin((2 * np.pi / 100) * i_val) * (2 * np.pi / 20)

            # Velocity matrix of end effector
            E = sp.Matrix([
                [x_dot],
                [0],
                [z_dot],
                [0],
                [0],
                [0]
            ])
            # Jacobian with updated values 
            J0 = J.subs({
                theta1: q[0], theta2: q[1], theta3: q[2], theta4: q[3], theta5: q[4], theta6: q[5]
            })

            # qdot - Join velocities
            qdot = J0.pinv() * E

            # q - Joint Matix
            q += qdot
            print('asdfffff')

            print('obtained q values')
            joint_positions = Float64MultiArray()
            joint_positions.data = [0.0,0.0,float(q[0]),float(q[1]),float(q[2]),float(q[3]),float(q[4]),float(q[5])]
            self.joint_position_pub.publish(joint_positions)
            print('publishing')

            # ... (rest of the KinematicsController class code from the second script)

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = IMUSubscriber()
    kinematics_controller = KinematicsController()
    rclpy.spin(imu_subscriber)
    # kinematics_controller.joint_position_pub.publish(joint_positions)
    imu_subscriber.destroy_node()
    kinematics_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
