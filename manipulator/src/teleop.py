#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from pynput import keyboard

# Define key codes
LIN_VEL_STEP_SIZE = 5
ANG_VEL_STEP_SIZE = 0.1

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)


        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d

        q : force stop

        Moving the joints of the robot:
        
        t: Move the link 1 backwards
        y: Move the link 2 forward

        u: Move the link 2 backwards
        i: Move the link 2 forward

        o: Move the link 3 backwards
        p: Move the link 3 forward

        f: Move the link 4 backwards
        g: Move the link 4 forward

        h: Move the link 5 backwards
        j: Move the link 5 forward

        k: Move the link 6 backwards
        l: Move the link 6 forward

        Esc to quit
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        link_angle1 = Float64MultiArray()
        link_angle2 = Float64MultiArray()
        link_angle3 = Float64MultiArray()
        link_angle4 = Float64MultiArray()
        link_angle5 = Float64MultiArray()
        link_angle6 = Float64MultiArray()

        linear_vel=0.0
        steer_angle=0.0
        joint_angle1 = 0.0
        joint_angle2 = 0.0
        joint_angle3 = 0.0
        joint_angle4 = 0.0
        joint_angle5 = 0.0
        joint_angle6 = 0.0


        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    linear_vel=0.0
                    steer_angle=0.0
                    joint_angle1 = 0.0
                    joint_angle2 = 0.0
                    joint_angle3 = 0.0
                    joint_angle4 = 0.0
                    joint_angle5 = 0.0
                    joint_angle6 = 0.0

                elif key == 'w':  # Forward
                    linear_vel += LIN_VEL_STEP_SIZE
                elif key == 's':  # Reverse
                    linear_vel -= LIN_VEL_STEP_SIZE
                elif key == 'd':  # Right
                    steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'a':  # Left
                    steer_angle += ANG_VEL_STEP_SIZE

                elif key == 't':  # Front joint 1
                    joint_angle1 -= ANG_VEL_STEP_SIZE
                
                elif key == 'y':  # Front joint 1
                    joint_angle1 += ANG_VEL_STEP_SIZE
                
                elif key == 'u':  # Front joint 2
                    joint_angle2 -= ANG_VEL_STEP_SIZE
                
                elif key == 'i':  # Front joint 2
                    joint_angle2 += ANG_VEL_STEP_SIZE
                
                elif key == 'o':  # Front joint 3
                    joint_angle3 -= ANG_VEL_STEP_SIZE
                
                elif key == 'p':  # Front joint 3
                    joint_angle3 += ANG_VEL_STEP_SIZE
                
                elif key == 'f':  # Front joint 4
                    joint_angle4 -= ANG_VEL_STEP_SIZE
                
                elif key == 'g':  # Front joint 4
                    joint_angle4 += ANG_VEL_STEP_SIZE
                
                elif key == 'h':  # Front joint 5
                    joint_angle5 -= ANG_VEL_STEP_SIZE
                
                elif key == 'j':  # Front joint 5
                    joint_angle5 += ANG_VEL_STEP_SIZE
                
                elif key == 'k':  # Front joint 6
                    joint_angle6 -= ANG_VEL_STEP_SIZE
                
                elif key == 'l':  # Front joint 6
                    joint_angle6 += ANG_VEL_STEP_SIZE



                if steer_angle>1.0:
                        steer_angle=1.0
                if steer_angle<-1.0:
                    steer_angle=-1.0

                if joint_angle1>1.0:
                        joint_angle1=1.0
                if joint_angle1<-1.0:
                    joint_angle1=-1.0
                
                if joint_angle2>1.0:
                        joint_angle2=1.0
                if joint_angle2<-1.0:
                    joint_angle2=-1.0
                
                if joint_angle3>1.0:
                        joint_angle3=1.0
                if joint_angle3<-1.0:
                    joint_angle3=-1.0
                
                if joint_angle4>1.0:
                        joint_angle4=1.0
                if joint_angle4<-1.0:
                    joint_angle4=-1.0
                
                if joint_angle5>1.0:
                        joint_angle5=1.0
                if joint_angle5<-1.0:
                    joint_angle5=-1.0
                
                if joint_angle6>1.0:
                        joint_angle6=1.0
                if joint_angle6<-1.0:
                    joint_angle6=-1.0

                print("Steer Angle",steer_angle)
                print("Linear Velocity",linear_vel)
                print("Joint Angle 1",joint_angle1)
                print("Joint Angle 2",joint_angle2)
                print("Joint Angle 3",joint_angle3)
                print("Joint Angle 4",joint_angle4)
                print("Joint Angle 5",joint_angle5)
                print("Joint Angle 6",joint_angle6)
                # Publish the twist message
                wheel_velocities.data = [0.0,0.0,linear_vel,-linear_vel]
                joint_positions.data = [steer_angle,steer_angle,joint_angle1,joint_angle2,joint_angle3,joint_angle4,joint_angle5,joint_angle6]

                #Publish the angles
                # link_angle1.data = [joint_angle1]
                # link_angle2.data = [joint_angle2]
                # link_angle3.data = [joint_angle3]
                # link_angle4.data = [joint_angle4]
                # link_angle5.data = [joint_angle5]
                # link_angle6.data = [joint_angle6]

                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)

                # self.link_angle1_pub.publish(link_angle1)
                # self.link_angle2_pub.publish(link_angle2)
                # self.link_angle3_pub.publish(link_angle3)
                # self.link_angle4_pub.publish(link_angle4)
                # self.link_angle5_pub.publish(link_angle5)
                # self.link_angle6_pub.publish(link_angle6)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()