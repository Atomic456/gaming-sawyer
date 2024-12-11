#! /usr/bin/python3.8

from typing import List
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

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
		rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
		rospy.loginfo("------------------")
		rospy.loginfo("Response Message:\n%s", resp)
	else:
		rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
		rospy.logerr("Result Error %d", resp.result_type[0])
		return False

	return True   

def main():
	rospy.init_node("rsdk_ik_service_client")
	pos = [0.8823970358775916, 0.15828035764230267, 0.18079174237566487]
	ori = [0.7659923239667895, -0.6428418763319106, 0.0031286275807636565, 0.0005416156269654382]
	if ik_service_client(pos, ori, "right_hand"):
		rospy.loginfo("Simple IK call passed!")
	else:
		rospy.logerr("Simple IK call FAILED")


if __name__ == '__main__':
	main()
    
