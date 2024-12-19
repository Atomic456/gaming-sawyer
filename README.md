This repository provides difference files for working with the sawyer robot.
In order to run this scripts the ros1 noetic must be used.
And the scripts should be copyed into the /intera_examples/scripts folder
The function is run with following comand: rosrun intera_examples bamboleo.py

Robot_Controller:
  - robot_init: restarts the robot after the emergency stop was hit and calibrates the gripper
  - gripper_controle(action:str): takes a string open/close and changes the gripper
  - move_limb_neutral_pos: moves the arm to the neutral pose provided by the sawyer robot
  - set_joint_angels: takes a dictionary which is provided by the ik_solver_client
  - get_joint_angles: returs a dictionary containing the angles of all joints
  - get_endpoint: retuns the endpoint in real world coordinates and the roation of the tool plate
  > #WARNING: the joint6 must be set to 0° of rotation#
  - get_camera_pos(pos, ori): takes the position and the quaternio of the end effector and calculates the camera position
  - ik_service_client(pos, ori, tip_name): impöements an ik solver client in order the do inverse kinematics calculations for the provided position and quaternio orientation

Robot_Math:
  - quaternion2matrix(x,y,z,w): takes all four parts of a quaternion and retruns the coresponding roational matrix
  - add_quaternion(quat1, quat2): returns the dot product of the two porvided quaternions
  - matrix2quaternion(r): takes a rotational matrix and returns the coresponding quaternion
  > #WARNING: function not tested#
  - angles2quaternion(roll, pitch, yaw): taks rotational angels and returns quaternion
  - multiplay_quaternions(quat1, quat2): retruns the product of the two provided quaternions
  - game_piece_picker(object, center_x, center_y): calculates the center of mass (com) for all peces and returns the piece that rebalences the board

Object_Detection:
  > #WARNING: not testes#
  - detection_circle_and_blocks(img): takes the image and finds the radius and center of the board plus the center of all playing pieces
  > #WARNING: the images are resized wiht imgsz=640#
  - detect_objects(img): takes an cv2 image and retruns a list of all detected playing pieces
  - calc_object_center(x_min, y_min, x_max, y_max): takes two corner of a YOLO boundary box and calculates the center
  - calc_offset(x, y, hight, width): calculates the offset of an point from the center of the picture(height, width)
  - get_pixel_size(img): takes the image and calculates the radius of the board in order to calculate the pixel size in mm/pixel

Bamboleo:
  - main(): the function implements the game logic for clearing the bamboleo board
    > - the robot will move to a fixed postion to take the picture, in order the change the postion you can move the robot to the desired postion and use the get_joint_angels function
    > - the images are resized to 640x640 using padding in order to allow a translation in to real world coordinates
    > - the camera is calibrated by changeing the exposior and the gain in order to impove the image quality
    > - camera_cneterd parameter enables self calibarion by centering the camera center in the board center of the robot if set to false
    > - the robot will take a picture and aquirer the object list by using the YOLO model
    > - the bast playing piece to remove will be coosen
    > - the coordinates of the piece will be determind and translated to real world coordinates
    > - the robot will change the gripper orientation and the postion
    > - then the gripper will move down and close the gripper
    > - then the robot will move back up and place the objec to the side 
