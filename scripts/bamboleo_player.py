#! /usr/bin/python3.8

########## Imports ############
import time
import cv2
import numpy as np

import rospy
import intera_interface

from intera_interface import CHECK_VERSION
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
###############################


# prepare the robot for executing comands
def robot_init():
	# reset robot if necessary
	rs = intera_interface.RobotEnable(CHECK_VERSION)
	init_state = rs.state().enabled
	if not init_state:
		rs.enable()
		
	# initialize gripper
	gripper = intera_interface.Gripper()
	gripper.reboot()
	time.sleep(0.5)
	gripper.calibrate()
	while not gripper.is_ready():
		time.sleep(0.1)
	rospy.loginfo("Robot initialization complete!")

def camera_calibration(img):
	img = cv2.GaussianBlur(img, (9,9), 2)
	circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=45, minRadius=0, maxRadius=0)
	board_center = []
	object_center = []
	pixel_size = 0
	if circles is not None:
		circles = np.round(circles[0, :]).astype("int")
		for i in range(len(circles)):
			x,y,r = circles[i]
			if i == 0:
				board_center = [x, y]
				pixel_size = 360/(2*r)
			elif i == 1:
				object_center = [x, y]
	return board_center, object_center, pixel_size
	

# main method
def main():
	rospy.init_node('bamboleo_player')
	img_topic = "/io/internal_camera/right_hand_camera/image_rect"
	bridge = CvBridge()
	#robot_init()
	
	# wait for image
	img = rospy.wait_for_message(img_topic, Image)

	cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
	cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
	hight_y, width_x = cv2_img.shape
	print(hight_y, width_x)
	board_center, object_center, pixel_size = camera_calibration(cv2_img)
	if board_center and object_center and pixel_size > 0:
		print(f"Board Center: {board_center}, Object Center: {object_center}, Pixel Size: {pixel_size}")
		print(((board_center[0]-(width_x/2))*pixel_size)/1000)
		print(((board_center[1]-(hight_y/2))*pixel_size)/1000)
	else:
		print("error")
	
	rospy.spin()

# script entry point
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
