#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64MultiArray

# --- AXES ---
FORWARD_AXIS  = 5   # RT
BACKWARD_AXIS = 4   # LT
AXIS_LEFTX    = 0   # Left stick X
AXIS_RIGHTX   = 2   # Right stick X

# --- BUTTONS ---
ENABLE_BTN = 6   # RB
BOOST_BTN  = 7   # LB
AUTO_LIDAR_BTN  = 4   # Y

V_SPEED = 10.5       # m/s
OMEGA_SPEED = 50   # rad/s

class JoyButtonDrive(Node):
    def __init__(self):
        super().__init__('joy_button_drive')

        #self.pub_vel = self.create_publisher(Twist, '/diff_controller/cmd_vel_unstamped', 10)
        self.pub_vel = self.create_publisher(TwistStamped, '/diff_controller/cmd_vel', 10)
        self.pub_lidar = self.create_publisher(Float64MultiArray, '/servo_controller/commands', 10)
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        # Only move if ENABLE button is pressed
        #if not msg.buttons[ENABLE_BTN]:
        #    return

        #t = Twist()
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        
        boost = msg.buttons[BOOST_BTN]

        forward  = 0.5 * (1 - msg.axes[FORWARD_AXIS])
        backward = 0.5 * (1 - msg.axes[BACKWARD_AXIS])

        if boost:
            t.twist.linear.x = -1.5 * V_SPEED * (forward - backward)
            t.twist.angular.z = -1.5 * OMEGA_SPEED * msg.axes[AXIS_LEFTX]
        else:
            t.twist.linear.x = -V_SPEED * (forward - backward)
            t.twist.angular.z = -OMEGA_SPEED * msg.axes[AXIS_LEFTX]

        # Right stick X → servo
        pose = Float64MultiArray()
        alpha = np.interp(msg.axes[AXIS_RIGHTX], [-1, 1], [0, np.pi])
        #alpha = np.interp(msg.axes[AXIS_RIGHTX], [-1, 1], [-90, 90])
        cmd = float(alpha)
        pose.layout.data_offset = 0
        pose.data = [cmd]

        self.pub_vel.publish(t)
        self.pub_lidar.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = JoyButtonDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
