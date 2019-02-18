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
    grasp_service_client = GraspServiceClient()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_commander")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    # rospy.sleep(2)

    # scene.remove_world_object("table")
    # scene.remove_world_object("guard")
    # rospy.sleep(2)

    # p = geometry_msgs.msg.PoseStamped()
    # p.header.frame_id = robot.get_planning_frame()
    # p.pose.position.x = 0.5
    # p.pose.position.y = -0.4
    # p.pose.position.z = -0.02
    # p.pose.orientation.w = 1.0
    # scene.add_box("table", p, (1.2, 1.0, 0.04))

    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.5
    p.pose.position.y = 0.547
    p.pose.position.z = 0.0
    p.pose.orientation.w = 1.0
    scene.add_box("guard", p, (0.1, 0.1, 0.25))

    ########################################################
    arm = moveit_commander.MoveGroupCommander("left_hand_arm")

    arm.set_max_velocity_scaling_factor(0.25)
    arm.set_max_acceleration_scaling_factor(0.25)

    arm_initial_pose = arm.get_current_pose().pose
    arm_initial_joints = arm.get_current_joint_values()

    gripper = moveit_commander.MoveGroupCommander("left_hand")
    gripper_joint_values = gripper.get_current_joint_values()
    print "Left hand: ", gripper_joint_values
    # ### Open
    gripper_joint_values[0] = 0.0
    gripper.set_joint_value_target(gripper_joint_values)
    gripper.go(wait=True)
    
    ### tf2
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    ### get target tf

    solution = grasp_service_client.get_grasp_solution()
    print "Got solution: ", solution
    wrist_transform = msg2Transform3(solution2pose(solution))

    # offset_transform = msg2Transform3(get_ee_transfrom(tf_buffer, parent_frame_id='left_arm_7_link', child_frame_id='left_hand_kuka_coupler_bottom'))
    # ee_transform = msg2Transform3(get_ee_transfrom(tf_buffer, parent_frame_id='world', child_frame_id='left_hand_kuka_coupler_bottom'))
    tmp = msg2Transform3(get_ee_transfrom(tf_buffer, parent_frame_id='left_arm_7_link', child_frame_id='left_hand_palm_link'))
    # tmp2 = msg2Transform3(get_ee_transfrom(tf_buffer, parent_frame_id='left_hand_kuka_coupler_bottom', child_frame_id='left_hand_palm_link'))
    # offset_transform.inverted()
    #tmp.inverted()
    # offset_transform.multiplied(ee_transform)
    m1 = wrist_transform.to_matrix()
    m2 = tmp.to_matrix()
    ee_transform = Transform3.from_matrix(np.dot(m1,m2))

    rotation = tt.quaternion_about_axis(135.0*np.pi/180.0,(0,0,1))
    ee_transform.q = quaternion_mult(ee_transform.q, rotation)
    # tt.
    
    br = tf.TransformBroadcaster()

    print tmp.q.to_euler()
    # target_pose = transform2PoseMsg(ee_transform)
    # arm.set_pose_target(target_pose,end_effector_link="left_hand_palm_link")
    # arm.go(wait=True)
    # rospy.sleep(2)
    # arm.clear_pose_targets()
    # arm.stop()

    # target_pose = transform2PoseMsg(ee_transform)

    ### Joint space
    # arm.set_pose_target(target_pose,end_effector_link="left_arm_7_link")
    # arm.go(wait=True)
    # rospy.sleep(2)

    ### Cartesian space
    # waypoints = []
    # waypoints.append(target_pose)
    # move_cartesian_path(waypoints, arm)
    # rospy.sleep(2)

    # ### Close
    # gripper_joint_values[0] = 0.7
    # gripper.set_joint_value_target(gripper_joint_values)
    # gripper.go(wait=True)
    # rospy.sleep(2)

    ### Cartesian space
    # target_pose = get_ee_transfrom(tf_buffer, 'ee_target')
    # waypoints = []
    # waypoints.append(target_pose)
    # move_cartesian_path(waypoints, arm)
    # rospy.sleep(2)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        br.sendTransform(ee_transform.p,
                        ee_transform.q,
                        rospy.Time.now(),
                        "grasp_goal",
                        "world")

        rate.sleep()


    moveit_commander.roscpp_shutdown()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException, e:
        print(e)
