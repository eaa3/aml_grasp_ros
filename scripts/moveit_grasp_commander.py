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


def get_ee_transfrom(buf, child_frame_id):
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = buf.lookup_transform('world', child_frame_id, rospy.Time(0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("failed to lookup transform")
            rate.sleep()
            continue

    ee_target = geometry_msgs.msg.Pose()
    ee_target.position = trans.transform.translation
    ee_target.orientation = trans.transform.rotation
    q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

    if child_frame_id is 'ee_target':
        qy = tt.quaternion_about_axis(-0.5*math.pi, (0, 1, 0))
        qx = tt.quaternion_about_axis(math.pi, (1, 0, 0))
        q = tt.quaternion_multiply(q, qy)
        q = tt.quaternion_multiply(q, qx)

    if child_frame_id is 'link_6_target':
        pos = qr.quaternion_rotate_vec(q, [0.0, 0.0, -0.16]) # link_6 to ee
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


def main():
    grasp_service_client = GraspServiceClient()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_commander")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)

    scene.remove_world_object("table")
    scene.remove_world_object("guard")
    rospy.sleep(2)

    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.5
    p.pose.position.y = -0.4
    p.pose.position.z = -0.02
    p.pose.orientation.w = 1.0
    scene.add_box("table", p, (1.2, 1.0, 0.04))

    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.0
    p.pose.position.y = 0.11
    p.pose.position.z = 0.1
    p.pose.orientation.w = 1.0
    scene.add_box("guard", p, (0.1, 0.1, 0.2))

    ########################################################
    arm = moveit_commander.MoveGroupCommander("arm")

    arm_initial_pose = arm.get_current_pose().pose
    arm_initial_joints = arm.get_current_joint_values()

    gripper = moveit_commander.MoveGroupCommander("gripper")
    gripper_joint_values = gripper.get_current_joint_values()

    ### Open
    gripper_joint_values[0:3] = [0.0, 0.0, 0.0]
    gripper.set_joint_value_target(gripper_joint_values)
    gripper.go()
    rospy.sleep(2)

    ### tf2
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    ### get target tf

    solution = grasp_service_client.get_grasp_solution()
    print "Got solution: ", solution
    wirst_transform = solution2pose(solution)

    # br = tf.TransformBroadcaster()

    # for i in range(10):
    #     br.sendTransform(solution['base_pose']['p'],
    #                     solution['base_pose']['q'],
    #                     rospy.Time.now(),
    #                     "grasp_goal",
    #                     "world")

    # arm.set_pose_target(target_pose)
    # arm.go()
    # rospy.sleep(2)
    target_pose = get_target_transform(wirst_transform)

    ### Joint space
    # arm.set_pose_target(target_pose)
    # arm.go()
    # rospy.sleep(2)

    ### Cartesian space
    waypoints = []
    waypoints.append(target_pose)
    move_cartesian_path(waypoints, arm)
    rospy.sleep(2)

    ### Close
    gripper_joint_values[0:3] = [1.0, 1.0, 1.0]
    gripper.set_joint_value_target(gripper_joint_values)
    gripper.go()
    rospy.sleep(2)

    ### Cartesian space
    target_pose = get_ee_transfrom(tf_buffer, 'ee_target')
    waypoints = []
    waypoints.append(target_pose)
    move_cartesian_path(waypoints, arm)
    rospy.sleep(2)

    moveit_commander.roscpp_shutdown()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException, e:
        print(e)
