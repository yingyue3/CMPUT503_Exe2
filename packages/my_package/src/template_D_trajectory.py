#!/usr/bin/env python3

# import required libraries
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import LEDPattern 
from std_msgs.msg import Header, ColorRGBA 
import numpy as np

class TrajectoryNode(DTROS):
    def __init__(self, node_name):
        super(TrajectoryNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # add your code here

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        self.wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self.led_topic = f"/{self._vehicle_name}/led_emitter_node/led_pattern"
        
        self._ticks_left = 0
        self._ticks_right = 0
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        self.pub = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)
        self.led_pub = rospy.Publisher(self.led_topic, LEDPattern, queue_size=1)  # LED publisher
        self._radius = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/radius', 100)
        self.DISTANCE_PER_TICK = np.pi*2*self._radius/135
        self.start_dist = 0
        self.wheelbase = 0.205

        # subscribe to the left and right wheel encoder topics
        # publish to the wheels command topic

        # LEDs

        # define other variables
        # initialize ros bag file
        self.ROTATION_TARGET = np.pi*self.wheelbase / 4
        self.ARC_TARGET = np.pi*((self.wheelbase/2)+0.54) / 4
        pass

    def callback_left(self, data):
        # log general information once at the beginning
        # rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        # rospy.loginfo_once(f"Left encoder type: {data.type}")
        # store data value
        self._ticks_left = data.data

    def callback_right(self, data):
        # log general information once at the beginning
        # rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        # rospy.loginfo_once(f"Right encoder type: {data.type}")
        # store data value
        self._ticks_right = data.data
        
    def compute_distance_traveled(self, ticks):
        # left_distance = self._ticks_left* self.DISTANCE_PER_TICK
        # right_distance = self._ticks_right* self.DISTANCE_PER_TICK
        # dist = []
        # dist.append(left_distance)
        # dist.append(right_distance)
        distance = ticks* self.DISTANCE_PER_TICK
        return distance
    
    def drive_straight(self, speed=0.2, direction=1, distance=1.25):
        msg = WheelsCmdStamped()
        msg.vel_left = speed * direction 
        msg.vel_right = speed * direction * 0.9

        # self._ticks_left = 0  # Reset encoders before movement
        # self._ticks_right = 0
        self.start_dist = self.compute_distance_traveled(self._ticks_left)

        

        while np.abs(self.start_dist - self.compute_distance_traveled(self._ticks_left)) < distance and not rospy.is_shutdown():
            self.pub.publish(msg)
            # rospy.sleep(0.1)  


        rospy.loginfo(self.compute_distance_traveled(self._ticks_left))
        # Stop after reaching the target distance
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)

    def rotate_clockwise(self, speed=0.2):
        msg = WheelsCmdStamped()
        msg.vel_left = speed 
        # msg.vel_right = -speed *0.8
        msg.vel_right = 0

        # self._ticks_left = 0  
        # self._ticks_right = 0

        # Calculate target rotation distance (90 degrees)
         
        self.start_dist = self.compute_distance_traveled(self._ticks_left)


        while np.abs(self.start_dist - self.compute_distance_traveled(self._ticks_left)) < self.ROTATION_TARGET and not rospy.is_shutdown():
            self.pub.publish(msg)
            # rospy.sleep(0.1)


        # Stop the robot after rotation
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
        rospy.loginfo("Rotation complete (90 degrees clockwise).")
    
    def drive_arc(self, speed=0.2):
        # add your code here
        msg = WheelsCmdStamped()
        msg.vel_left = speed 
        # msg.vel_right = -speed *0.8
        msg.vel_right = speed * 0.38

        # self._ticks_left = 0  
        # self._ticks_right = 0

        # Calculate target rotation distance (90 degrees)
         
        self.start_dist = self.compute_distance_traveled(self._ticks_left)


        while np.abs(self.start_dist - self.compute_distance_traveled(self._ticks_left)) < self.ARC_TARGET and not rospy.is_shutdown():
            self.pub.publish(msg)
            # rospy.sleep(0.1)


        # Stop the robot after rotation
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
        rospy.loginfo("Rotation complete (90 degrees clockwise).")
        pass
    
    def draw_d_shape(self, **kwargs):
        # add your code here
        self.use_leds("blue")
        self.drive_straight(speed=0.8, direction=1, distance=1.1)
        rospy.sleep(1) 
        self.rotate_clockwise(speed=0.8)
        rospy.sleep(1) 
        self.drive_straight(speed=0.8, direction=1, distance=0.65)
        rospy.sleep(1) 
        self.use_leds("green")
        self.drive_arc(speed=0.8)
        rospy.sleep(1) 
        self.use_leds("blue")
        self.drive_straight(speed=0.8, direction=1, distance=0.5)
        rospy.sleep(1) 
        self.use_leds("green")
        self.drive_arc(speed=0.8)
        rospy.sleep(1) 
        self.use_leds("blue")
        self.drive_straight(speed=0.8, direction=1, distance=0.65)
        rospy.sleep(1) 
        self.use_leds("red")

        pass

    def use_leds(self, color):
        # add your code here
        x = ()
        if color == "blue":
            x = (0.0, 0.0, 1.0, 1.0)
        elif color == "red":
            x = (1.0, 0.0, 0.0, 1.0)
        elif color == "green":
            x = (0.0, 1.0, 0.0, 1.0)

        msg = LEDPattern()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        color_msg = ColorRGBA()
        color_msg.r, color_msg.g, color_msg.b, color_msg.a = x

        # Set LED colors
        msg.rgb_vals = [color_msg] * 5
        self.led_pub.publish(msg) 


    # define other functions if needed

    def run(self):
        self.use_leds("red")
        rospy.sleep(2)  # wait for the node to initialize
        rospy.Rate(10)
        self.draw_d_shape()

        # add your code here
        pass

if __name__ == '__main__':
    # define class TrajectoryNode
    # call the function run of class TrajectoryNode
    node = TrajectoryNode(node_name='move_node_D')
    node.run()
    rospy.spin()