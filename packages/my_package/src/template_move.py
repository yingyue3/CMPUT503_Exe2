#!/usr/bin/env python3

# import required libraries

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import WheelsCmdStamped
import numpy as np

class MoveNode(DTROS):
    def __init__(self, node_name):
        super(MoveNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # add your code here
        # subscribe to the left and right wheel encoder topics
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        self.wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        # publish to the wheels command topic
        self._ticks_left = 0
        self._ticks_right = 0
        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        self.pub = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)
        self._radius = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/radius', 100)
        self.DISTANCE_PER_TICK = np.pi*2*self._radius/135
        self.start_dist = 0
        self.wheelbase = 0.07

        # LEDs

        # define other variables    
        self.TARGET_DISTANCE = 1.25
        self.ROTATION_TARGET = np.pi*self.wheelbase / 4
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
        
    def compute_distance_traveled(self):
        left_distance = self._ticks_left* self.DISTANCE_PER_TICK
        right_distance = self._ticks_right* self.DISTANCE_PER_TICK
        return left_distance 
    
    def drive_straight(self, speed=0.2, direction=1):
        msg = WheelsCmdStamped()
        msg.vel_left = speed * direction
        msg.vel_right = speed * direction

        # self._ticks_left = 0  # Reset encoders before movement
        # self._ticks_right = 0
        self.start_dist = self.compute_distance_traveled()

        

        while np.abs(self.start_dist - self.compute_distance_traveled()) < self.TARGET_DISTANCE and not rospy.is_shutdown():
            self.pub.publish(msg)
            rospy.sleep(0.1)  


        rospy.loginfo(self.compute_distance_traveled())
        # Stop after reaching the target distance
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
    
    def rotate_clockwise(self, speed=0.2):
        msg = WheelsCmdStamped()
        msg.vel_left = speed   
        msg.vel_right = -speed 

        # self._ticks_left = 0  
        # self._ticks_right = 0

        # Calculate target rotation distance (90 degrees)
         
        self.start_dist = self.compute_distance_traveled()


        while np.abs(self.start_dist - self.compute_distance_traveled()) < self.ROTATION_TARGET and not rospy.is_shutdown():
            rospy.loginfo(self.start_dist - self.compute_distance_traveled())
            self.pub.publish(msg)
            rospy.sleep(0.1)


        # Stop the robot after rotation
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
        rospy.loginfo("Rotation complete (90 degrees clockwise).")

    def rotate_anticlockwise(self, speed=0.2):
        msg = WheelsCmdStamped()
        msg.vel_left = -speed   
        msg.vel_right = speed 

        # self._ticks_left = 0  
        # self._ticks_right = 0

        # Calculate target rotation distance (90 degrees)
         
        self.start_dist = self.compute_distance_traveled()


        while np.abs(self.start_dist - self.compute_distance_traveled()) < self.ROTATION_TARGET and not rospy.is_shutdown():
            rospy.loginfo(self.start_dist - self.compute_distance_traveled())
            self.pub.publish(msg)
            rospy.sleep(0.1)


        # Stop the robot after rotation
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
        rospy.loginfo("Rotation complete (90 degrees clockwise).")

    def use_leds(self, **kwargs):
        # add your code here
        pass

    # define other functions if needed
    
    def run(self): 

        # self.drive_straight(speed=0.5, direction=1)

        # rospy.sleep(1)  

        # self.drive_straight(speed=0.5, direction=-1)

        

        # rospy.sleep(1)

        self.rotate_clockwise(speed=1)

        rospy.sleep(1)

        self.rotate_anticlockwise(speed=1)

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.pub.publish(stop)

if __name__ == '__main__':
    # define class MoveNode
    # call the function run of class MoveNode
    node = MoveNode(node_name='move_node')
    node.run()
    rospy.spin()
