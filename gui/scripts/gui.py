#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

import tkinter as tk
import psutil
import numpy as np


Vel = .5
Omega = 1.
pwm = 255

def getDescription():
    return """  This node takes keypresses from the keyboard and publishes them 
    as Twist messages. It works best with a US keyboard layout.
    ---------------------------
    Moving around:
    /    w    /
    a    /    d
    /    s    /
                    
    For PWM raw input:
    ---------------------------
    /    W    /
    A    /    D
    /    S    /
                    
    t : up (+z)
    b : down (-z)
                    
    anything else : stop
                    
    P/p : increase/decrease pwm
    V/v : increase/decrease only linear speed by 10%
    O/o : increase/decrease only angular speed by 10%
                    
    CTRL-C to quit
                    
    currently:	    speed """+str(Vel)+"""	turn """+  str(Omega)+""" 
    currently pwm:  """+str(pwm) 

def my_constraint(value,min,max):
    if (value>=max): return max
    elif (value<=min): return min
    else: return value

def getStatus():
    battery = psutil.sensors_battery()

    seconds = battery.secsleft
    minutes, seconds = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)

    return  "Percentuale batteria: %d%%\n" % (battery.percent) + \
            "Attaccato alla corrente: " + str(battery.power_plugged) + "\n" + \
            "Tempo rimanente: %d:%02d:%02d" % (hours, minutes, seconds)

description = getDescription()
wheels_distance = 0.297
left_wheel_radius = 0.033
right_wheel_radius = 0.033


def handle_keypress(event, description_wheels, node):
    """Move diffbot accordingly to the key pressed"""
    global Vel,Omega,pwm,description
    twist = Twist()
    cmd = event.char
    if cmd:
        if (cmd == "w"):
            twist.linear.x = Vel
            twist.angular.z = .0
        elif (cmd == "s"):
            twist.linear.x = -Vel
            twist.angular.z = .0
        elif (cmd == "a"):
            twist.linear.x = .0
            twist.angular.z = Omega
        elif (cmd == "d"):
            twist.linear.x = .0
            twist.angular.z = -Omega
        

        elif (cmd == "W"):
            twist.linear.x = (pwm*left_wheel_radius + pwm*right_wheel_radius)/2.0
            twist.angular.z = .0
        elif (cmd == "S"):
            twist.linear.x = -(pwm*left_wheel_radius + pwm*right_wheel_radius)/2.0
            twist.angular.z = .0
        elif (cmd == "A"):
            twist.linear.x = .0
            twist.angular.z = -(pwm*left_wheel_radius + pwm*right_wheel_radius)/ (2.0*wheels_distance)
        elif (cmd == "D"):
            twist.linear.x = .0
            twist.angular.z = (pwm*left_wheel_radius + pwm*right_wheel_radius)/ (2.0*wheels_distance)

        elif (cmd == "P"):
            pwm = my_constraint(pwm+1,0,255)
            description_wheels.config(text=getDescription())
        elif (cmd == "p"):
            pwm = my_constraint(pwm-1,0,255)
            description_wheels.config(text=getDescription())
        elif (cmd == "L"):
            Vel = round(Vel+(Vel/10.0),2)
            description_wheels.config(text=getDescription())
        elif (cmd == "l"):
            Vel = round(Vel-(Vel/10.0),2)
            description_wheels.config(text=getDescription())
        elif (cmd == "O"):
            Omega = round(Omega+(Omega/10.0),2)
            description_wheels.config(text=getDescription())
        elif (cmd == "o"):
            Omega = round(Omega+(Omega/10.0),2)
            description_wheels.config(text=getDescription())


        else:
            twist.linear.x = .0
            twist.angular.x = .0
            node.get_logger().info('Invalid command')
            return
            
        node.vel_publisher_.publish(twist)
        node.get_logger().info('Publishing: V = %.2f, w = %.2f' % (twist.linear.x, twist.angular.z))
    

def handle_move(slider_servo, node):
    """Move servo accordingly to the slider as button is pressed"""
    pose = Float64MultiArray()
    #cmd  = float(slider_servo.get()*np.pi/180)
    cmd  = float(slider_servo.get()+90)
    
    pose.layout.data_offset = 0
    pose.data = [cmd]
    node.pos_publisher_.publish(pose)
    node.get_logger().info("Move to %.2f" % cmd)


def handle_update_satus(description_status, window):
    description_status.config(text=getStatus())
    window.after(1000, handle_update_satus, description_status, window)

def handle_destroy(event, node):
    node.window.destroy()
    node.get_logger().info('Exiting by Contrl+C')



def setup(node):
    # GUI windows
    window = tk.Tk()
    window.title("GUI")
    window.resizable(width=True, height=True)
    for i in range(6):
        if i%2==0: window.rowconfigure(i, minsize = 20, weight=0)  
        else: window.rowconfigure(i, minsize = 20 , weight=1)   
    window.columnconfigure(0, minsize = 400, weight=1)
    # create subsection in windows
    frame_status = tk.Frame(padx=10,pady=10)
    frame_wheels = tk.Frame(padx=10,pady=10)
    frame_servo= tk.Frame(padx=10,pady=10)
    
    # 1) BATTERY SECTION
    label_status = tk.Label(
        master = window,
        text="Status",
        fg="white",
        bg="black")
    
    description_status = tk.Label(
        master = frame_status,
        text=getStatus(),
        fg="black",
        anchor="w", justify="left",
        padx=10,pady=10)
    description_status.pack(fill = tk.X, side=tk.LEFT)
    
    # 2) WHEELS SECTION
    # text label
    label_wheels = tk.Label(
        master = window,
        text="Wheels",
        fg="white",
        bg="black")
    
    description_wheels = tk.Label(
        master = frame_wheels,
        text=description,
        fg="black",
        anchor="w", justify="left",
        padx=10,pady=10)
    description_wheels.pack(fill = tk.X, side=tk.LEFT)
    
    # 3) SERVO SECTION 
    # text label
    label_servo = tk.Label(
        master = window,
        text="Servo",
        fg="white",
        bg="black")
    slider_servo = tk.Scale(
        master = frame_servo, 
        from_ = -90, to = 90, 
        length = 200,
        orient = tk.HORIZONTAL)
    # button
    button_servo = tk.Button(
        master = frame_servo,
        command = lambda:handle_move(slider_servo,node),
        text = "Move!",
        bg = "blue",
        fg = "yellow",
        pady=10)
    slider_servo.grid(row=0, column=0, )#sticky="nsw")
    button_servo.grid(row=0, column=1, )#sticky="nse")

    # setup windows layout
    label_status.grid(row=0, column=0, sticky="new")
    frame_status.grid(row=1, column=0, sticky="ew")
    label_wheels.grid(row=2, column=0, sticky="new")
    frame_wheels.grid(row=3, column=0, sticky="ew")
    label_servo.grid(row=4, column=0, sticky="ew")
    frame_servo.grid(row=5, column=0, )#sticky="sew")

    return window,description_wheels, description_status


class GUIPublisher(Node):

    def __init__(self):
        super().__init__('gui')
        # setting up gui window
        self.window, description_wheels, description_status = setup(self)

        # setting up publishers
        self.vel_publisher_ = self.create_publisher(Twist, 'diff_controller/cmd_vel_unstamped', 10)
        self.pos_publisher_ = self.create_publisher(Float64MultiArray, 'servo_controller/commands', 10)

        # setting up windows event
        self.window.bind("<Key>",lambda event: handle_keypress(event,description_wheels,self))
        self.window.bind("<Control-c>", lambda event: handle_destroy(event, self))
        
        self.get_logger().info("GUI Started")
        # star windows loop
        self.window.after(1000, handle_update_satus, description_status, self.window)
        self.window.mainloop()
        raise SystemExit





def main(args=None):
    rclpy.init(args=args)

    gui_publisher = GUIPublisher()

    rclpy.spin(gui_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gui_publisher.destroy_node()
    rclpy.shutdown()

    return


if __name__ == '__main__':
    main()