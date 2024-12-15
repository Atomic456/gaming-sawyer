#! /usr/bin/python3.8

########## Imports ############
import time
import cv2
import numpy as np
import math

import rospy
import intera_interface

from typing import List
from intera_interface import CHECK_VERSION
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
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

def set_joint_angels(joint_angles):
	print('[DEBUG] ==> set_joint_angels')
	joint_angles['right_j6'] += 0.1496263401595
	robot_params = intera_interface.RobotParams()
	limb_names = robot_params.get_limb_names()
	joint_names = robot_params.get_joint_names(limb_names[0])
	limb = intera_interface.Limb(limb=limb_names[0])
	# set joint speed 
	limb.set_joint_position_speed(0.1)

	# move to meximum arme extentation
	limb.move_to_joint_positions(joint_angles)
	return limb.tip_state('right_hand').pose.position, limb.tip_state('right_hand').pose.orientation

def get_joint_angles():
	robot_params = intera_interface.RobotParams()
	limb_names = robot_params.get_limb_names()
	limb = intera_interface.Limb(limb=limb_names[0])
	return limb.joint_angles()

def get_endpoint():
	robot_params = intera_interface.RobotParams()
	limb_names = robot_params.get_limb_names()
	limb = intera_interface.Limb(limb=limb_names[0])
	return limb.tip_state('right_hand').pose.position, limb.tip_state('right_hand').pose.orientation

def get_camera_pos(pos, ori):
	print('[DEBUG] ==> get_camera_pos')
	rotation_gripper = quaternion2matrix(ori.x, ori.y, ori.z, ori.w)
	rotation_x_90 = np.array([
		[1, 0, 0],
		[0, 0, -1],
		[0, 1, 0]
	])
	rotation_camera = np.dot(rotation_x_90, rotation_gripper)
	camera_offset = [0.04, 0, -0.095]
	camera_pos = [pos.x, pos.y, pos.z] + (rotation_gripper.dot(camera_offset))
	return camera_pos, rotation_camera
	
def quaternion2matrix(x, y, z, w):
	return np.array([
		[1-2*(y**2+z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
		[2*(x*y + z*w), 1-2*(x**2 + z**2), 2*(y*z - x*w)],
		[2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x**2 + y**2)]
	])
	

def matrix2quaternion(r):
	quaternion = np.empty((4, ))
	# sum of diagonal values
	trace = np.trace(r)
	
	if trace > 0:
		s = np.sqrt(trace+1.0)*2
		w = 0.25 * s
		x = (r[2,1] - r[1,2])/s
		y = (r[0,2] - r[2,0])/s
		z = (r[1,0] - r[0,1])/s
	else:
		if (r[0,0] > r[1,1]) and (r[0,0] > r[2,2]):
			s = 2.0 * np.sqrt(1.0+ r[0,0]-r[1,1]-r[2,2])
			w = (r[2,1] - r[1,2])/s
			x = 0.25 * s 
			y = (r[0,1] + r[1,0])/s
			z = (r[0,2] + r[2,0])/s
		elif r[1,1] > r[2,2]:
			s = 2.0 * np.sqrt(1.0+ r[1,1]-r[0,0]-r[2,2])
			w = (r[0,2] - r[2,0])/s
			x = (r[0,1] + r[1,0])/s
			y = 0.25 * s
			z = (r[1,2] + r[2,1])/s
		else:
			s = 2.0 * np.sqrt(1.0+ r[2,2]-r[0,0]-r[1,1])
			w = (r[1,0] - r[0,1])/s
			x = (r[0,2] + r[2,0])/s
			y = (r[1,2] + r[2,1])/s
			z = 0.25 * s
	
	return [x, y, z, w]

def calc_offset(img_h, img_w, item_center):
	img_center_x = img_w/2
	img_center_y = img_h/2
	dx = item_center[0]-img_center_x
	dy = item_center[1]-img_center_y
	return dx, dy
	
def angles2quaterion(roll_x, pitch_y, yaw_z):
	roll = np.radians(roll_x)
	pitch = np.radians(pitch_y)
	yaw = np.radians(yaw_z)
	print(roll, pitch, yaw)
	
	cos_r = np.cos(roll / 2)
	sin_r = np.sin(roll / 2)
	
	cos_p = np.cos(pitch / 2)
	sin_p = np.sin(pitch / 2)
	
	cos_y = np.cos(yaw / 2)
	sin_y = np.sin(yaw / 2)
	
	qw = (cos_r * cos_p * cos_y) + (sin_r * sin_p * sin_y)
	qx = (sin_r * cos_p * cos_y) - (cos_r * sin_p * sin_y)
	qy = (cos_r * sin_p * cos_y) + (sin_r * cos_p * sin_y)
	qz = (cos_r * cos_p * sin_y) - (sin_r * sin_p * cos_y)
	
	return np.array([qx, qy, qz, qw])
	
	
def ik_service_client(pos:List[float], ori:List[float], tip_name:str) -> bool:
	if len(pos) < 3 or len(ori) < 4:
		rospy.logerr("Wrong size of positon or orientation argument specifyed!")
		return False
	ns = "ExternalTools/right/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	poses = {
    	'right': PoseStamped(
			header=hdr,
			pose=Pose(
				position=Point(
					x=pos[0],
					y=pos[1],
					z=pos[2],
				),
				orientation=Quaternion(
					x=ori[0],
					y=ori[1],
					z=ori[2],
					w=ori[3],
				),
			),
		),
	}
    
	ikreq.pose_stamp.append(poses["right"])
	ikreq.tip_names.append(tip_name)

	try:
		rospy.wait_for_service(ns, 5.0)
		resp = iksvc(ikreq)
	except (rospy.ServiceException, rospy.ROSException) as e:
		rospy.logerr("Service call failed: %s" % (e,))
		return False

	# Check if result valid, and type of seed ultimately used to get solution
	if (resp.result_type[0] > 0):
		seed_str = {
			ikreq.SEED_USER: 'User Provided Seed',
			ikreq.SEED_CURRENT: 'Current Joint Angles',
			ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
		}.get(resp.result_type[0], 'None')
		rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %(seed_str,))
		# Format solution into Limb API-compatible dictionary
		limb_joints = dict(list(zip(resp.joints[0].name, resp.joints[0].position)))
		#rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
		#rospy.loginfo("------------------")
		#rospy.loginfo("Response Message:\n%s", limb_joints["right_j0"])
	else:
		#rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
		rospy.logerr("Result Error %d", resp.result_type[0])
		return False

	return limb_joints   
	

# main method
def main():
	# config
	rospy.init_node('bamboleo_player')
	bridge = CvBridge()
	img_topic = "/io/internal_camera/right_hand_camera/image_rect"
	camera_pos_angles = {'right_j0': 2*math.pi/16, 'right_j1': 0.0, 'right_j2': -2*math.pi/4, 'right_j3': 2*math.pi/3.8, 'right_j4': 2*math.pi/4, 'right_j5': 0.0, 'right_j6': 0.0}
	
	print(angles2quaterion(90,90,0))
	
	# 1. init
	robot_init()
	
	# 2. move to camera position
	ee_pos, ee_ori = set_joint_angels(camera_pos_angles)
	camera_traslation, camera_rotation = get_camera_pos(ee_pos, ee_ori)
	
	robot_centerd = False
	while not robot_centerd:
		# 3. take image
		img = rospy.wait_for_message(img_topic, Image)
		cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
		cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
		hight_y, width_x = cv2_img.shape
		board_center, object_center, pixel_size = camera_calibration(cv2_img)
		pixel_dx, pixel_dy = calc_offset(hight_y, width_x, board_center)
		dx = (pixel_dx*pixel_size)*0.001
		dy = (pixel_dy*pixel_size)*0.001
		print(f"Dx: {dx}, Dy: {dy}")
		print(f"Board center: {board_center}")
		print(f"Object center: {object_center}")
		
		# 4. rotate arm if nessassary to adjust
		if round(dx,2) == 0.00 and round(dy, 2) == 0.00:
			robot_centerd = True
		else:
			pos_offset = np.array([dx, dy, camera_traslation[2]-0.3])
			new_pos = camera_traslation + (camera_rotation.dot(pos_offset))
			new_joint_angles = ik_service_client(new_pos, [ee_ori.x, ee_ori.y, ee_ori.z, ee_ori.w], "right_hand")
			if new_joint_angles:
				new_joint_angles['right_j6'] = 0.0
				ee_pos, ee_ori = set_joint_angels(new_joint_angles)
				camera_traslation, camera_rotation = get_camera_pos(ee_pos, ee_ori)
				
		# 5. object detection and messuring
		obj_pix_dx, obj_pix_dy = calc_offset(hight_y, width_x, object_center)
		dx = (obj_pix_dx*pixel_size)*0.001
		dy = (obj_pix_dy*pixel_size)*0.001
		pos_offset = np.array([dx, dy, 0.3])
		
		# 6. define orientation for girpping
		grip_ori = angles2quaterion(90, 0, 0)
		new_joint_angles = ik_service_client(pos_offset, grip_ori, "right_hand")
		if new_joint_angles:
			set_joint_angels(new_joint_angles)
		else:
			print('IKSolverError')
			
	#rospy.spin()

# script entry point
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
