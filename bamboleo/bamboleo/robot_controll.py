#! /usr/bin/python3.8
from robot_math import *
import intera_interface
import time
import rospy
from typing import List
from intera_interface import CHECK_VERSION
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
	
def gripper_controle(action:str):
	print('[DEBUG] ==> gripper_controle')
	gripper = intera_interface.Gripper()
	if action == 'open':
		gripper.open()
	if action == 'close':
		gripper.close()

def mov_limb_neutral_pos():
	print('[DEBUG] ==> limb_neutral_pos')
	robot_params = intera_interface.RobotParams()
	limb_names = robot_params.get_limb_names()
	joint_names = robot_params.get_joint_names(limb_names[0])
	limb = intera_interface.Limb(limb=limb_names[0])
	print(limb.endpoint_pose())
	# move to nutral positon
	limb.move_to_neutral(speed=0.1)

def set_joint_angels(joint_angles, corrected=False):
	print('[DEBUG] ==> set_joint_angels')
	if corrected:
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
		[0, 0, 1],
		[0, -1, 0]
	])
	rotation_camera = np.dot(rotation_x_90, rotation_gripper)
	camera_offset = [0.042, 0, -0.105]
	camera_pos = np.add(np.array([pos.x, pos.y, pos.z]), (np.dot(rotation_gripper, camera_offset)))
	return camera_pos, rotation_camera

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
