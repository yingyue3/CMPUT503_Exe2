#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image

import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self.image = None
        self.raw_image = None
        self._custom_topic = f"/{self._vehicle_name}/custom_node/image/compressed"
        self.pub = rospy.Publisher(self._custom_topic, CompressedImage) #queue_size=10)

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        self.raw_image = msg
        # display frame
        shape = image.shape
        cv2.imshow(self._window, image)
        cv2.waitKey(1)
        
        # convert RGB to Gray
        # https://www.geeksforgeeks.org/python-grayscaling-of-images-using-opencv/
        # https://learnopencv.com/annotating-images-using-opencv/
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        text = "Duck {name} says, 'Cheese! Capturing {size1}x{size2} - quack-tastic!'".format(name=self._vehicle_name, size1=shape[0], size2=shape[1])
        org = (50,350)
        cv2.putText(gray_image, text, org, fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (250,225,100))
        cv2.imshow('Gray', gray_image)
        cv2.waitKey(0)
        self.image = gray_image
    
    def start(self):
        # https://stackoverflow.com/questions/55377442/how-to-subscribe-and-publish-images-in-ros
        rate = rospy.Rate(1)
        rospy.loginfo("compressed image")
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            if self.image is not None:
                self.pub.publish(self._bridge.cv2_to_compressed_imgmsg(self.image, "passthrough"))
            #self.pub.publish(self.raw_image)
            rate.sleep()
        


if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    node.start()
    # keep spinning
    rospy.spin()
