#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import tf2_ros
import geometry_msgs.msg
import moveit_msgs.msg
from sensor_msgs.msg import JointState
import tf.transformations as tt
import utils as qr
import math
from aml_math import Transform3, quaternion_mult
import numpy as np
import atexit

import tf

from grasp_service_client import GraspServiceClient

def get_target_transform(transform):


    ee_target = geometry_msgs.msg.Pose()
    ee_target.position = transform.position
    ee_target.orientation = transform.orientation
    q = [transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w]


  
    pos = qr.quaternion_rotate_vec(q, [0.0, 0.0, -0.16])#-0.16]) # link_6 to ee
    ee_target.position.x += pos[0]
    ee_target.position.y += pos[1]
    ee_target.position.z += pos[2]

    qx = tt.quaternion_about_axis(math.pi, (1, 0, 0))
    q = tt.quaternion_multiply(q, qx)

    ee_target.orientation.x = q[0]
    ee_target.orientation.y = q[1]
    ee_target.orientation.z = q[2]
    ee_target.orientation.w = q[3]

    return ee_target


def get_ee_transfrom(buf, parent_frame_id = "world", child_frame_id = "left_arm_7_link"):
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = buf.lookup_transform(parent_frame_id, child_frame_id, rospy.Time(0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("failed to lookup transform")
            rate.sleep()
            continue

    ee_target = geometry_msgs.msg.Pose()
    ee_target.position = trans.transform.translation
    ee_target.orientation = trans.transform.rotation
    # q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
    # ee_target.orientation.w = q[3]


    return ee_target


def msg2Transform3(pose_msg):
    p = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
    q = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
    
    return Transform3(p=p, q=q)

def move_cartesian_path(waypoints, arm):
    success = False
    for i in range(10):
        (plan, fraction) = arm.compute_cartesian_path(waypoints,   # waypoints to follow
                                                      0.01,        # eef_step
                                                      0.0)         # jump_threshold
        if fraction < 0.85:
            print("iter=", i, "fraction=", fraction)
        else:
            print("start to move cartesian path")
            print("iter=", i, "fraction=", fraction)
            success = arm.execute(plan, wait=True)
            arm.stop()
            
            break

    return success

def solution2pose(solution, idx=0):

    target = geometry_msgs.msg.Pose()
    target.position.x = solution['base_trajectory'][idx]['p'][0]
    target.position.y = solution['base_trajectory'][idx]['p'][1]
    target.position.z = solution['base_trajectory'][idx]['p'][2]
    # target.orientation = trans.transform.rotation
    target.orientation.x = solution['base_trajectory'][idx]['q'][0]
    target.orientation.y = solution['base_trajectory'][idx]['q'][1]
    target.orientation.z = solution['base_trajectory'][idx]['q'][2]
    target.orientation.w = solution['base_trajectory'][idx]['q'][3]

    return target

def solution2carthesian_path(solution, tf_buffer):

    waypoints = []


    tmp = msg2Transform3(get_ee_transfrom(tf_buffer, parent_frame_id='left_arm_7_link', child_frame_id='left_hand_palm_link'))
    tmp2 = msg2Transform3(get_ee_transfrom(tf_buffer, parent_frame_id='left_arm_7_link', child_frame_id='left_hand_kuka_coupler_bottom'))
    tmp2_inv = tmp2.inverse()

    c2l = tmp2_inv.to_matrix()
    l2p = tmp.to_matrix()

    def to_palm_frame(wrist, coupler2link, link2palm):

        base_to_link7 = np.dot(wrist, coupler2link)
        link7_to_palm = np.dot(base_to_link7, link2palm)
        #base_to_palm = np.dot(m1,m2)
        ee_transform = Transform3.from_matrix(link7_to_palm)

        return transform2PoseMsg(ee_transform)


    for i in range(len(solution['base_trajectory'])):
        pose_msg = solution2pose(solution,i)

        wrist_pose = msg2Transform3(pose_msg).to_matrix()
        wpt = to_palm_frame(wrist_pose, c2l, l2p)

        waypoints.append(wpt)


    return waypoints

def transform2PoseMsg(transform3):

    target = geometry_msgs.msg.Pose()
    target.position.x = transform3.p[0]
    target.position.y = transform3.p[1]
    target.position.z = transform3.p[2]
    # target.orientation = trans.transform.rotation
    target.orientation.x = transform3.q[0]
    target.orientation.y = transform3.q[1]
    target.orientation.z = transform3.q[2]
    target.orientation.w = transform3.q[3]


    return target

def main():
    grasp_service_client = GraspServiceClient()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_commander")
    atexit.register(moveit_commander.roscpp_shutdown)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    # rospy.sleep(2)

    scene.remove_world_object("table")
    scene.remove_world_object("guard")
    rospy.sleep(2)

    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.55
    p.pose.position.y = 0
    p.pose.position.z = -0.33
    p.pose.orientation.w = 1.0  
    scene.add_box("table", p, (0.87, 1.77, 0.04))
    solution = grasp_service_client.get_grasp_solution()
    print "Got solution: ", solution
    cloud_centroid = solution['cloud_centroid']
    min_pt = solution['cloud_min_pt']
    max_pt = solution['cloud_max_pt']

    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = cloud_centroid[0] 
    p.pose.position.y = cloud_centroid[1] 
    p.pose.position.z = cloud_centroid[2]
    p.pose.orientation.w = 1.0
    scene.add_box("guard", p, (max_pt[0] - min_pt[0], max_pt[1] - min_pt[1], max_pt[2] - min_pt[2]))

    
    ########################################################
    arm = moveit_commander.MoveGroupCommander("left_hand_arm")

    print "Tolerances"
    print "Goal joint tol: ", arm.get_goal_joint_tolerance()
    print "Goal pos tol: ", arm.get_goal_position_tolerance()
    print "Goal ori tol: ", arm.get_goal_orientation_tolerance()

    arm.set_goal_joint_tolerance(0.1) # default 0.0001
    arm.set_goal_position_tolerance(0.1) # default 0.0001
    arm.set_goal_orientation_tolerance(0.1) # default 0.001
    
    arm.set_max_velocity_scaling_factor(0.25)
    arm.set_max_acceleration_scaling_factor(0.25)

    arm_initial_pose = arm.get_current_pose().pose
    arm_initial_joints = arm.get_current_joint_values()

    gripper = moveit_commander.MoveGroupCommander("left_hand")
    gripper_joint_values = gripper.get_current_joint_values()
    print "Left hand: ", gripper_joint_values
    # ### Open

    open_cmd = raw_input("Open hand? (y/n)")
    if open_cmd == "y":
        gripper_joint_values[0] = 0.0
        gripper.set_joint_value_target(gripper_joint_values)
        gripper.go(wait=True)
        
    ### tf2
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    br = tf.TransformBroadcaster()

    ### get target tf

    
    waypoints = solution2carthesian_path(solution, tf_buffer)    
    ee_transform = msg2Transform3(waypoints[0])
    
    

    # print tmp.q.to_euler()
    # Go to pre-grasp pose
    target_pose = transform2PoseMsg(ee_transform)
    arm.set_pose_target(target_pose,end_effector_link="left_hand_palm_link")
    arm.go(wait=True)
    rospy.sleep(2)
    arm.clear_pose_targets()
    arm.stop()

    scene.remove_world_object("guard")
    scene.remove_world_object("table")

    # Finish grasping
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(0.1)
    success = move_cartesian_path(waypoints[:3], arm)
    if not success:
        arm.set_pose_target(waypoints[2],end_effector_link="left_hand_palm_link")
        arm.go(wait=False)
        rospy.sleep(1)
        # arm.clear_pose_targets()
        # arm.stop()

    # Close hand
    gripper_joint_values[0] = 0.78
    gripper.set_joint_value_target(gripper_joint_values)
    gripper.go(wait=True)
    rospy.sleep(2)

    # Move up
    move_cartesian_path(waypoints[3:], arm)
    


    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        br.sendTransform(ee_transform.p,
                        ee_transform.q,
                        rospy.Time.now(),
                        "grasp_goal",
                        "world")

        rate.sleep()






if __name__ == '__main__':
    try:
        main()

        # grasp_service_client = GraspServiceClient()

        # moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("moveit_commander")
        # atexit.register(moveit_commander.roscpp_shutdown)

        # robot = moveit_commander.RobotCommander()
        # scene = moveit_commander.PlanningSceneInterface()
    except rospy.ROSInterruptException, e:
        print(e)
