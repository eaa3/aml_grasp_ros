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
from aml_math import Transform3, quaternion_mult, Quaternion
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
    for i in range(10):
        (plan, fraction) = arm.compute_cartesian_path(waypoints,   # waypoints to follow
                                                      0.01,        # eef_step
                                                      0.0)         # jump_threshold
        if fraction < 0.99:
            print("iter=", i, "fraction=", fraction)
        else:
            print("start to move cartesian path")
            arm.execute(plan)
            rospy.sleep(3)
            break


def solution2pose(solution):

    target = geometry_msgs.msg.Pose()
    target.position.x = solution['base_pose']['p'][0]
    target.position.y = solution['base_pose']['p'][1]
    target.position.z = solution['base_pose']['p'][2]
    # target.orientation = trans.transform.rotation
    target.orientation.x = solution['base_pose']['q'][0]
    target.orientation.y = solution['base_pose']['q'][1]
    target.orientation.z = solution['base_pose']['q'][2]
    target.orientation.w = solution['base_pose']['q'][3]

    return target

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

    # Grasp service
    grasp_service_client = GraspServiceClient()

    # Moveit and node setup
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_commander")
    atexit.register(moveit_commander.roscpp_shutdown)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # TF setup

    ## tf2
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    br = tf.TransformBroadcaster()

    # Getting point cloud
    cloud_info = grasp_service_client.get_cloud_centroid()
    centroid = np.array(cloud_info['cloud_centroid'])
    print "Got solution: ", cloud_info

    print centroid
    target_pos = centroid + np.array([-0.095,-0.01,0.07])
    target_q = Quaternion()
    target_q.set_from_euler([0,np.pi/2,0])
    target_transform = Transform3(p=target_pos,q=target_q)

    rate = rospy.Rate(30)

    # while not rospy.is_shutdown():
    #     br.sendTransform(target_transform.p,
    #                     target_transform.q,
    #                     rospy.Time.now(),
    #                     "grasp_goal",
    #                     "world")

    #     rate.sleep()
    br.sendTransform(target_transform.p,
                        target_transform.q,
                        rospy.Time.now(),
                        "grasp_goal",
                        "world")

    ########################################################
    arm = moveit_commander.MoveGroupCommander("left_hand_arm")

    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(0.3)

    arm_initial_pose = arm.get_current_pose().pose
    arm_initial_joints = arm.get_current_joint_values()
    
    
    # Pre-grasp
    target_pose = transform2PoseMsg(target_transform)

    arm.set_pose_target(target_pose,end_effector_link="left_hand_palm_link")
    arm.go(wait=True)
    rospy.sleep(2)
    arm.clear_pose_targets()
    arm.stop()

    # Grasp
    target_pos = centroid + np.array([-0.095,-0.01,0.03])
    target_q = Quaternion()
    target_q.set_from_euler([0,np.pi/2,0])
    target_transform = Transform3(p=target_pos,q=target_q)
    target_pose = transform2PoseMsg(target_transform)

    arm.set_pose_target(target_pose,end_effector_link="left_hand_palm_link")
    arm.go(wait=True)
    rospy.sleep(2)
    arm.clear_pose_targets()
    arm.stop()



    gripper = moveit_commander.MoveGroupCommander("left_hand")
    gripper_joint_values = gripper.get_current_joint_values()
    print "Left hand: ", gripper_joint_values
    # ### Open
    gripper_joint_values[0] = 0.0
    gripper.set_joint_value_target(gripper_joint_values)
    gripper.go(wait=True)

    # ### Close
    gripper_joint_values[0] = 0.75
    gripper.set_joint_value_target(gripper_joint_values)
    gripper.go(wait=True)


    # Move up
    target_pos = centroid + np.array([-0.095,-0.01,0.12])
    target_q = Quaternion()
    target_q.set_from_euler([0,np.pi/2,0])
    target_transform = Transform3(p=target_pos,q=target_q)
    target_pose = transform2PoseMsg(target_transform)

    arm.set_pose_target(target_pose,end_effector_link="left_hand_palm_link")
    arm.go(wait=True)
    rospy.sleep(2)
    arm.clear_pose_targets()
    arm.stop()


    



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException, e:
        print(e)
