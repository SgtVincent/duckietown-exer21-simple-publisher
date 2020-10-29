#!/usr/bin/env python3
# modified on the basis of https://github.com/duckietown/dt-duckiebot-interface/blob/daffy/packages/camera_driver/src/camera_node.py

import os
import rospy
import cv2
import atexit
import numpy as np
from threading import Thread

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from cv_bridge import CvBridge


class CameraPub(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraPub, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        
        self.pub_img = rospy.Publisher('~image/compressed', 
            CompressedImage, 
            queue_size=1
        )
        
        # self.pub_camera_info = rospy.Publisher(
        #     "~camera_info",
        #     CameraInfo,
        #     queue_size=1
        # )   
        self.value = np.empty((480, 640, 3), dtype=np.uint8)
        self.cap = cv2.VideoCapture(2)
        
        # open camera if not 
        if not self.cap.isOpened():
            self.start_camera()
        
        # test if camera opened
        re, image = self.cap.read()
        if not re:
            raise RuntimeError("Could not read image from camera.")
    
    def start_camera(self):
        if not self.cap.isOpened():
            self.cap.open(2)

    def stop_camera(self):
        if hasattr(self, 'cap'):
            self.cap.release()

    def __del__(self):
        self.stop_camera()
        super(CameraPub, self).__del__()

    def run(self):
        
        bridge = CvBridge()

        while(1):
            re, image = self.cap.read()

            if image is not None:
                image = np.uint8(image)
                
            stamp = rospy.Time.now()
            image_message = bridge.cv2_to_compressed_imgmsg(image, dst_format='jpeg')
                
            #not sure if this actualy does something/ gets published
            image_message.header.stamp = stamp
            image_message.header.frame_id = self.frame_id

            #Publish the compressed image
            self.pub_img.publish(image_message)

            # # Publish the CameraInfo message 
            # self.current_camera_info.header.stamp = stamp
            # self.pub_camera_info.publish(self.current_camera_info)
            rospy.sleep(rospy.Duration.from_sec(0.001))



if __name__ == '__main__':
    # create the node
    node = CameraPub(node_name='camerapub')
    # run node
    frame_thread = Thread(target=node.run)
    frame_thread.start()
    # keep spinning
    rospy.spin()