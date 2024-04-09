#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import datetime
import os

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

DIRECTORY = 'records/'
FILE = 'dcmotor' 

wheels_distance = 0.297
left_wheel_radius = 0.033
right_wheel_radius = 0.033

class DataCollectorSubscriber(Node):

    def __init__(self):
        super().__init__('data_collector')
        # setting up variable
        self.joint = JointState()
        self.cmd = Twist()

        # setting up csv file
        if not os.path.exists(DIRECTORY):
            os.makedirs(DIRECTORY)
            self.get_logger().info('Created directory "%s"' % DIRECTORY)
        current_date = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        self.file_name = DIRECTORY + FILE + '_' + str(current_date) + '.csv'

        with open(self.file_name, 'a', newline='') as file:
            writer = csv.writer(file) 
            header=['time [ms]', 'name_left', 'pwm_left [duty cycle]', 'omega_left [rpm]', 'name_right', 'pwm_right [duty cycle]', 'omega_right [rpm]']            
            writer.writerow(header)
            file.close()
        self.get_logger().info('Created file: "%s"' % self.file_name)

        # setting up subscriber
        topic_name = '/joint_states'
        self.joints_subscriber = self.create_subscription(JointState, topic_name,
            self.joint_callback, 10)
        self.get_logger().info('Started on topic: "%s"' % topic_name)

        topic_name = '/cmd_vel_unstamped'
        self.joints_subscriber = self.create_subscription(Twist, topic_name,
            self.cmd_callback, 10)
        self.get_logger().info('Started on topic: "%s"' % topic_name)

        # setting up timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def joint_callback(self, msg):
        # download data
        self.joint = msg

    def cmd_callback(self, msg):
        # download data
        self.cmd = msg

    def timer_callback(self):
        # download data
        time = rclpy.time.Time()

        # joint values
        names = self.joint.name
        positions = self.joint.position
        velocities = self.joint.velocity
        # command values
        pwm = [.0, .0]
        v = self.cmd.linear.x
        w = self.cmd.angular.z
        pwm[0] = (v - w * wheels_distance / 2.0) / left_wheel_radius
        pwm[1] = (v + w * wheels_distance / 2.0) / right_wheel_radius
        
        # store data
        row = [time, names[0], pwm[0], velocities[0], names[1], pwm[1], velocities[1]]
        with open(self.file_name, 'a', newline='') as file:
            writer = csv.writer(file)       
            writer.writerow(row)
            file.close()
            self.get_logger().info('writing: "[%s,%s,%s,%s,%s]"' % (str(time),str(pwm[0]),str(pwm[1]),str(velocities[0]),str(velocities[1])))




def main(args=None):
    rclpy.init(args=args)

    data_collector_subscriber = DataCollectorSubscriber()

    rclpy.spin(data_collector_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    data_collector_subscriber.destroy_node()
    rclpy.shutdown()

    return


if __name__ == '__main__':
    main()