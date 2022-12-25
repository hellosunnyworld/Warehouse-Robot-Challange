#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from copy import deepcopy
from object_color_detector.srv import *
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import pow
def cartesian(arm,poses):
# Encapsulates the settings for cartesian planning
    fraction = 0.0   # coverage rate of path planning
    maxtries = 100   # maximum planning times
    attempts = 0     # history planning times

    # get the name of the end effector link
    end_effector_link = "wx250s/ee_arm_link"
    # set the current state of the end effector as the start state
    start_pose = arm.get_current_pose(end_effector_link).pose

    #print( start_pose )
    hpose = deepcopy(start_pose)        # initialize scene

    # Initialize the list of waypoints
    waypoints = []

    # set path points and add them to the path point list
    for wpose in poses:
        waypoints.append(deepcopy(wpose))
    # set the current state of the robot arm as the start state
    arm.set_start_state_to_current_state()
 
    # try to plan a path in Cartesian space, passing all path points in order
    while fraction < 1.0 and attempts < maxtries:
        (plan, fraction) = arm.compute_cartesian_path (
                                waypoints,   # waypoint poses path
                                0.01,        # eef_step，终端步进值
                                0.0,         # jump_threshold，跳跃阈值
                                True)        # avoid_collisions，避障规划
            
        # increment the trial times
        attempts += 1
            
        # print motion planning progress
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
    # If the path planning succeeds（coverage = 100%）, start to move the robot arm
    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the arm.")
        arm.execute(plan)
        rospy.loginfo("Path execution complete.")
    # If failing in path planning. print the error message
    else:
        rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

    rospy.sleep(2)

def worldX(yc):
# from camera frame to world frame x
    return -0.00035*yc + (0.31499)
def worldY(xc):
    return -0.00042*xc + (0.10371)
def detectObj(arm, wpose, i, scene):
# get cube positions and moves to one cube
    poses = []
    # detect objects
    rospy.wait_for_service("/object_detect")
    detector = rospy.ServiceProxy("/object_detect", DetectObjectSrv)
    pos = detector(0)
    # if it is the first detection, add all cubes as obstacles into the scene
    if(i == 0):
        addBlockObstacle(scene, pos)

    # update the current pose
    arm.set_start_state_to_current_state()

    try:
        if (i == 0):
            wpose.position.x = worldX(pos.redObjList[0].position.y) 
            wpose.position.y = worldY(pos.redObjList[0].position.x)
            wpose.position.z = 0.09
            
        elif (i == 1):    
            wpose.position.x = worldX(pos.blueObjList[0].position.y)
            wpose.position.y = worldY(pos.blueObjList[0].position.x)
            wpose.position.z = 0.09

        elif (i == 2):    
            wpose.position.x = worldX(pos.greenObjList[0].position.y)
            wpose.position.y = worldY(pos.greenObjList[0].position.x)
            wpose.position.z = 0.09

        print(wpose)
        poses.append(deepcopy(wpose))
        # carry out cartesian plan to move the gripper to the target object
        cartesian(arm, poses)

        # remove the cube after it is grasped
        if (i == 0):
            scene.remove_world_object('RedBlock') 
        elif (i == 1):
            scene.remove_world_object('BlueBlock') 
        else:
            scene.remove_world_object('GreenBlock') 
    except:
        pass

def addObstacle(scene):
# add table and trash bins as obstacles
    # set the size of the table and tool
    table_size = [10,10,0.00001]
    bins_size = [0.29, 0.12, 0.13]

    # add the table to the scene
    table_pose = PoseStamped()
    table_pose.header.frame_id = 'world'

    table_pose.pose.position.x = 0.0
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = 0.01

    table_pose.pose.orientation.w = 1
    scene.add_box('table', table_pose, table_size)

    # add the bins to the scene
    bins_pose = PoseStamped()
    bins_pose.header.frame_id = 'world'

    bins_pose.pose.position.x = -0.005
    bins_pose.pose.position.y = 0.19
    bins_pose.pose.position.z = 0.08

    bins_pose.pose.orientation.w = 1
    scene.add_box('bins', bins_pose, bins_size)

def addBlockObstacle(scene, pos):
# add cubes as obstacles
    block_size = [0.02, 0.02, 0.02]
    block_pose = PoseStamped()
    block_pose.header.frame_id = 'world'
    block_pose.pose.position.z = 0.01
    block_pose.pose.orientation.w = 1

    try:
        block_pose.pose.position.x = worldX(pos.redObjList[0].position.y)
        block_pose.pose.position.y = worldY(pos.redObjList[0].position.x)
        scene.add_box('RedBlock', block_pose, block_size)
    except:
        print("Didn't find the red block")

    try:
        block_pose.pose.position.x = worldX(pos.blueObjList[0].position.y) 
        block_pose.pose.position.y = worldY(pos.blueObjList[0].position.x)
        scene.add_box('BlueBlock', block_pose, block_size)
    except:
        print("Didn't find the blue block")

    try:
        block_pose.pose.position.x = worldX(pos.greenObjList[0].position.y)
        block_pose.pose.position.y = worldY(pos.greenObjList[0].position.x)
        scene.add_box('GreenBlock', block_pose, block_size)
    except:
        print("Didn't find the green block")
    rospy.sleep(1)

def move_to_bin(i,arm):
# moves to the position above the corresponding bin with 6-axis target states
    if (i == 0): #red
        joint_positions = [1.5738643407821655, -0.34821364283561707, -0.3666214048862457, -0.0015339808305725455, -1.5508545637130737, 0.0015339808305725455]
    elif (i == 1): #blue
        joint_positions = [2.0033788681030273, -0.2822524607181549, -0.2991262674331665, -0.012271846644580364, -1.543184757232666, 0.4218447208404541]
    else: #green
        joint_positions = [1.1857671737670898, -0.2991262674331665, -0.3344078063964844, 0.00920388475060463, -1.5447187423706055, -0.39269909262657166]        
    arm.set_joint_value_target(joint_positions)
    arm.go()
    rospy.sleep(1)

class MoveItCartesianDemo:
    def __init__(self):
        # initialize move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # initialize nodes
        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        # initialize scene
        scene = PlanningSceneInterface()
        rospy.sleep(1)

        # initialize the gripper's group which should be controlled by move_group
        gripper = moveit_commander.MoveGroupCommander("interbotix_gripper")
        
        # set the tolerance of the motion of the gripper
        gripper.set_goal_joint_tolerance(0.0001)

        # set the maximum speed and acceleration
        gripper.set_max_acceleration_scaling_factor(0.5)
        gripper.set_max_velocity_scaling_factor(0.5)
       
        #scene.remove_world_object('bins') 
        scene.remove_world_object('RedBlock') 
        scene.remove_world_object('BlueBlock') 
        scene.remove_world_object('GreenBlock') 
        scene.remove_world_object('bins') 
        # initialize the arm group which should be controlled by move_group
        arm = MoveGroupCommander('interbotix_arm')
        
        # allow replan after failing in planning
        arm.allow_replanning(True)
        
        # set the reference frame of the goal pose
        arm.set_pose_reference_frame('world')
                
        # set the tolerance of goal position(m) and orientation(rad)
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        
        # set the maximum speed and acceleration
        arm.set_max_acceleration_scaling_factor(1)
        arm.set_max_velocity_scaling_factor(1)
        rospy.set_param("/wx250s/robot_description_kinematics/interbotix_arm/kinematics_solver","lma_kinematics_plugin/LMAKinematicsPlugin")
        rospy.set_param("/wx250s/rviz_ubuntu_3085633_2002174377580385898/interbotix_arm/kinematics_solver","lma_kinematics_plugin/LMAKinematicsPlugin")           
        rospy.set_param("/wx250s/robot_description_kinematics/interbotix_arm/kinematics_solver_search_resolution", 0.006)
        rospy.set_param("/wx250s/rviz_ubuntu_3085633_2002174377580385898/interbotix_arm/kinematics_solver_search_resolution", 0.006)
        # get the name of the link of the end effector
        end_effector_link = "wx250s/ee_arm_link"

        # set the table and the trash bins as obstacles
        addObstacle(scene)
        rospy.sleep(1)

        # open the gripper
        # Set the target position of the gripper, described using the position data of both joints 
        joint_positions = [0.036, -0.036]
        gripper.set_joint_value_target(joint_positions)
        # Control the gripper to complete the movement
        gripper.go()
        rospy.sleep(1)

        for i in range(3):
            # move to the calibration pose to observe objects
            poses = []
            start_pose = arm.get_current_pose(end_effector_link).pose       
            wpose = deepcopy(start_pose)
            print("1st pose: observe objects")
            wpose.position.x = 0.15
            wpose.position.y = 0
            wpose.position.z = 0.3
            wpose.orientation.x = 0
            wpose.orientation.y = pow(2,0.5)/2
            wpose.orientation.z = 0
            wpose.orientation.w = pow(2,0.5)/2
            print(wpose)
            poses.append(deepcopy(wpose))

            # carry out cartesian plan to move the gripper above the target object
            cartesian(arm, poses)

            # update the current pose
            arm.set_start_state_to_current_state()

            # # move to the target object
            print("2nd pose: pick object")
            detectObj(arm, wpose, i, scene)

            # Set the target position of the gripper, described using the position data of both joints (in radians)
            joint_positions = [0.024, -0.024]
            gripper.set_joint_value_target(joint_positions)
            # Control the gripper to complete the movement
            gripper.go()
            rospy.sleep(2)

            print("4th pose: place object")
            # move to the target bin
            move_to_bin(i,arm)
            # open the gripper
            # Set the target position of the gripper, described using the position data of both joints
            joint_positions = [0.036, -0.036]
            gripper.set_joint_value_target(joint_positions)
            # Control the gripper to complete the movement
            gripper.go()
            rospy.sleep(2)

        arm.set_named_target('Sleep')
        arm.go()
        rospy.sleep(1)

        scene.remove_world_object('table') 
        scene.remove_world_object('bins') 
        # turn off and exit moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
