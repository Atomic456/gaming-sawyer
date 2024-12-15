#! /usr/bin/python3.8
from object_detection import *
from robot_controll import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ROS_IMAGE

import rospy
import cv2
import numpy as np
import intera_interface


# main method
def main():
	rospy.init_node('bamboleo')
	bridge = CvBridge()
	img_topic = "/io/internal_camera/right_hand_camera/image_raw"
	camera_pos_angles = {'right_j0': 2*math.pi/16, 'right_j1': 0.0, 'right_j2': -2*math.pi/4, 'right_j3': 2*math.pi/3.8, 'right_j4': 2*math.pi/4, 'right_j5': 0.0, 'right_j6': 0.0}
	rotation_x_90 = np.array([
		[1, 0, 0],
		[0, 0, 1],
		[0, -1, 0]
	])
	
	# 0. initialise robot
	robot_init()
	
	# 1. move robot to overhead camera postion
	ee_pos, ee_ori = set_joint_angels(camera_pos_angles)
	
	# 2. setup camera setting
	cameras = intera_interface.Cameras()
	cameras.start_streaming('right_hand_camera')
	cameras.set_gain('right_hand_camera', 2)
	cameras.set_exposure('right_hand_camera', 14)
	
	camera_centerd = False
	while not camera_centerd:
		print("3. take picture")
		img = rospy.wait_for_message(img_topic, ROS_IMAGE)
		cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
		height, width, cannels = cv2_img.shape
		
		print("3.1 find center of the board")
		pixel_size, board_center = get_pixel_size(cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY))
		dx, dy = calc_center_offset(board_center[0], board_center[1], height, width)
		dx_m = dx*pixel_size*0.001
		dy_m = dy*pixel_size*0.001
		
		print(f"dx_m: {dx_m} | dy_m: {dy_m}")
		if round(dx_m, 2) != 0.00 and round(dy_m, 2) != 0.00:
			print("3.2 move to center postion")
			cam_pos, cam_ori = get_camera_pos(ee_pos, ee_ori)
			if round(dy_m, 3) == 0.000:
				print("dy_m: 0.000")
				center_pos = cam_pos + np.dot(cam_ori, np.array([dx_m, 0, 0]))
			elif round(dx_m, 3) == 0.000:
				print("dx_m: 0.000")
				center_pos = cam_pos + np.dot(cam_ori, np.array([0, dy_m, 0]))
			else:
				print("else")
				center_pos = cam_pos + np.dot(cam_ori, np.array([dx_m, dy_m, 0]))
			new_joint_angles = ik_service_client(center_pos, np.array([ee_ori.x, ee_ori.y, ee_ori.z, ee_ori.w]), "right_hand")
			new_joint_angles['right_j6'] = 0.0
			ee_pos, ee_ori = set_joint_angels(new_joint_angles)
		else:
			camera_centerd = True
	
	print("4. take new picture from centerd postion")
	img = rospy.wait_for_message(img_topic, ROS_IMAGE)
	cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
	height, width, cannels = cv2_img.shape
	
	# 5. detect playing pieces in image
	object_list = detect_objects(cv2_img)[0]
	object_center = calc_object_center(object_list[0], object_list[1], object_list[2], object_list[3])
	dx, dy = calc_center_offset(object_center[0], object_center[1], height, width)
	print(f"dx: {dx} | dy: {dy}")
	pixel_size, _ = get_pixel_size(cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY))
	dx_m = dx*pixel_size*0.001
	dy_m = dy*pixel_size*0.001
	
	# 6. calculate orientation of postion of picken the object
	cam_pos, cam_ori = get_camera_pos(ee_pos, ee_ori)
	new_ori = matrix2quaternion(np.dot(quaternion2matrix(ee_ori.x, ee_ori.y, ee_ori.z, ee_ori.w), rotation_x_90))
	new_pos = cam_pos + np.dot(cam_ori, np.array([dx_m, dy_m, cam_pos[2]-0.30]))
	ee_pos, ee_ori = set_joint_angels(ik_service_client(new_pos,new_ori, "right_hand"))
	down_pos = np.array([ee_pos.x, ee_pos.y, 0.21])
	down_ori = np.array([ee_ori.x, ee_ori.y, ee_ori.z, ee_ori.w])
	ee_pos, ee_ori = set_joint_angels(ik_service_client(down_pos, down_ori, "right_hand"))
	
	
	
	
	
	
	
# script entry point
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
