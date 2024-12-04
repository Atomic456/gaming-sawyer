#! /usr/bin/python3.8
# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as Image

import rospy
import intera_interface

bridge = CvBridge()
images_taken = 15

def image_callback(msg):
    global images_taken
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        cv2.imshow("image", cv2_img)
        cv2.waitKey(3)
        if images_taken < 16:
        	cv2.imwrite('../images/calibration/img'+str(images_taken)+'.png', cv2_img)
        images_taken = images_taken + 1

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/io/internal_camera/right_hand_camera/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    main()
