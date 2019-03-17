#!/usr/bin/env python

import sys, subprocess
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
from kbhit import KBHit




from grasp_service_client import GraspServiceClient


from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    JointTrajectory
)

from boris_tools.moveit_wrapper import MoveitWrapper
from boris_tools.trajectory_io import parse_trajectory_file, make_ros_trajectory_msg, make_cartesian_trajectory
from boris_tools.boris_robot import BorisRobot
from boris_tools.boris_kinematics import boris_kinematics
from boris_joint_trajectory_action import MinJerkTrajHelper

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

def cartesian_path2joint_path(waypoints, robot):
    trajectory = JointTrajectory()
    trajectory.joint_names = robot.joint_names('left_arm')

    time_sec = 0.5
    time_incr = 0.5
    for i in range(len(waypoints)):

        wpt = waypoints[i]
        joint_positions = robot.inverse_kinematics([wpt.position.x,wpt.position.y,wpt.position.z], 
                                                    [wpt.orientation.x,wpt.orientation.y,wpt.orientation.z, wpt.orientation.w])
        
        if joint_positions is not None:

            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.velocities = [0.0]*len(joint_positions)
            point.accelerations = [0.0]*len(joint_positions)

            point.time_from_start = rospy.Duration(time_sec)

            trajectory.points.append(point)
            
        else:
            rospy.logwarn("Failed to find ik solution for grasp waypoint %d"%(i,))

        time_sec += time_incr

    return trajectory


def solution2hand_joint_path(solution, robot):

    trajectory = JointTrajectory()
    trajectory.joint_names = robot.joint_names('left_hand')

    time_sec = 0.5
    time_incr = 8.0
    for i in range(len(solution['joint_trajectory'])):

        
        joint_positions = [np.mean(solution['joint_trajectory'][i])]
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0]*len(joint_positions)
        point.accelerations = [0.0]*len(joint_positions)

        point.time_from_start = rospy.Duration(time_sec)

        trajectory.points.append(point)
            
        time_sec += time_incr

    return trajectory



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


class GraspExecuter(object):


    def __init__(self):

        

        self._moveit_wrapper = MoveitWrapper()
        self._moveit_wrapper.init_moveit_commander()
        rospy.init_node('grasp_player')
        self._moveit_wrapper.setup()

        self._kin  = boris_kinematics(root_link="left_arm_base_link")
        self._boris = BorisRobot(moveit_wrapper = self._moveit_wrapper)

        self._scene = self._moveit_wrapper.scene()
        self._planning_frame = self._moveit_wrapper.robot().get_planning_frame()


        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)
        self._br = tf.TransformBroadcaster()

        self._scene.remove_world_object("table")
        

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self._planning_frame
        p.pose.position.x = 0.55
        p.pose.position.y = 0
        p.pose.position.z = -0.33
        p.pose.orientation.w = 1.0  
        self._scene.add_box("table", p, (0.87, 1.77, 0.04))

        self._solution = None
        self._grasp_waypoints = []
        self._pre_grasp_plan = None
        self._grasp_arm_joint_path = None
        self._grasp_hand_joint_path = None

        self._post_grasp_plan = None

        self._grasp_service_client = GraspServiceClient()


        self._boris.set_control_mode(mode="joint_impedance", limb_name="left_arm")

        self._arm_traj_generator = MinJerkTrajHelper()
        self._hand_traj_generator = MinJerkTrajHelper()
        self._pre_grasp_traj_generator = MinJerkTrajHelper()
        self._post_grasp_traj_generator = MinJerkTrajHelper()


        self._scan_waypoints = np.load("/home/earruda/Projects/boris_ws/src/boris-robot/boris_tools/scripts/scan_waypoints2.npy")


        self._kbhit = KBHit()

    def scan_object(self):
        
        command = "rosservice call /grasp_service/acquire_cloud"
        for waypoint in self._scan_waypoints:

            # goto
            self._boris.goto_with_moveit("left_arm",waypoint)
            # scan
            process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
            output, error = process.communicate()


            print output

    def remove_collision_object(self, name):

        self._scene.remove_world_object(name)
        rospy.sleep(0.5)
        
    def add_table(self):
        self.remove_collision_object("table")

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self._planning_frame
        p.pose.position.x = 0.55
        p.pose.position.y = 0
        p.pose.position.z = -0.35#-0.34
        p.pose.orientation.w = 1.0  
        self._scene.add_box("table", p, (0.87, 1.77, 0.04))
        rospy.sleep(2)

    def add_object_guard(self, grasp_solution):
        self.remove_collision_object("guard")
        
        # print "Got solution: ", grasp_solution
        cloud_centroid = grasp_solution['cloud_centroid']
        min_pt = grasp_solution['cloud_min_pt']
        max_pt = grasp_solution['cloud_max_pt']

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self._planning_frame
        p.pose.position.x = cloud_centroid[0] 
        p.pose.position.y = cloud_centroid[1] 
        p.pose.position.z = cloud_centroid[2]
        p.pose.orientation.w = 1.0
        self._scene.add_box("guard", p, (max_pt[0] - min_pt[0], max_pt[1] - min_pt[1], max_pt[2] - min_pt[2]))

        rospy.sleep(0.1)



    def get_solution(self):
        self._solution = self._grasp_service_client.get_grasp_solution()

        self.add_table()
        #self.add_object_guard(self._solution)

        
        self._grasp_waypoints = solution2carthesian_path(self._solution, self._tf_buffer)

        group = self._moveit_wrapper.get_group("left_hand_arm")
        group.set_goal_joint_tolerance(0.02) # default 0.0001
        group.set_goal_position_tolerance(0.05) # default 0.0001
        group.set_goal_orientation_tolerance(0.02) # default 0.001

        self._grasp_waypoints[0].position.z += 0.10
        self._pre_grasp_plan = self._boris.get_moveit_cartesian_plan("left_hand_arm", self._grasp_waypoints[0])  


        is_executable = len(self._pre_grasp_plan.joint_trajectory.points) > 0
        
        # if is_executable:
        #     # current_angles = self._boris.angles("left_arm")

        #     # joint_positions = self._pre_grasp_plan.joint_trajectory.points[-1].positions
        #     # self._moveit_wrapper.set_start_state("left_hand_arm", self._boris.joint_names("left_arm"), joint_positions)
        #     # rospy.sleep(1.0)
        #     self._grasp_arm_joint_path, fraction = self._boris.compute_cartesian_path_moveit("left_hand_arm",self._grasp_waypoints[:3])
        #     # Set back to current
        #     # self._moveit_wrapper.set_start_state("left_hand_arm", self._boris.joint_names("left_arm"), current_angles)
        #     is_executable = fraction >= 0.85

        if is_executable:
            
            self._pre_grasp_pose = msg2Transform3(self._grasp_waypoints[0])
            
           
            #cartesian_path2joint_path(self._grasp_waypoints, self._boris)
            self._grasp_hand_joint_path = solution2hand_joint_path(self._solution, self._boris)
            #print self._grasp_hand_joint_path

            self._pre_grasp_traj_generator.set_waypoints(self._pre_grasp_plan.joint_trajectory)
            self._hand_traj_generator.set_waypoints(self._grasp_hand_joint_path)

            rospy.loginfo("Pre grasp plan length %d"%(len(self._pre_grasp_plan.joint_trajectory.points),))
            rospy.loginfo("Pre grasp plan total time %f"%(self._pre_grasp_plan.joint_trajectory.points[-1].time_from_start.to_sec(),))
            
            rospy.loginfo("Hand path length %d"%(len(self._grasp_hand_joint_path.points),))
            rospy.loginfo("Hand path total time %f"%(self._grasp_hand_joint_path.points[-1].time_from_start.to_sec(),))
            

        
         
        # state = self._moveit_wrapper.robot().get_current_state()

        # print "Robot state"
        # print state

        #group = self._moveit_wrapper.get_group("left_hand_arm")
          
        

        return is_executable

    def update_step(self, step, time_steps, incr):
        step = step + incr

        if step >= len(time_steps):
            step = len(time_steps)-1
        elif step < 0:
            step = 0

        return step

    def goto_pregrasp(self):

        self._boris.execute_moveit_plan("left_hand_arm", self._pre_grasp_plan)

    def plan_grasp(self):

        self.remove_collision_object("guard")
        self.remove_collision_object("table")
        self._grasp_arm_joint_path, fraction = self._boris.compute_cartesian_path_moveit("left_hand_arm",self._grasp_waypoints[:3], 
                                                                                         eef_step = 0.015, jump_threshold = 10.0)
        # Set back to current
        # self._moveit_wrapper.set_start_state("left_hand_arm", self._boris.joint_names("left_arm"), current_angles)
        is_executable = fraction >= 0.85

        ## try to plan final goal in case cartesian path
        if not is_executable:
            self._grasp_arm_joint_path = self._boris.get_moveit_cartesian_plan("left_hand_arm", self._grasp_waypoints[2])  
            is_executable = len(self._grasp_arm_joint_path.joint_trajectory.points) > 0

        if not is_executable:

            rospy.logwarn("Grasp not executable, maybe try again or try new grasp")
        
        else:

            self._grasp_arm_joint_path = self._grasp_arm_joint_path.joint_trajectory

            self._arm_traj_generator.set_waypoints(self._grasp_arm_joint_path)

            rospy.loginfo("Grasp path length %d"%(len(self._grasp_arm_joint_path.points),))
            rospy.loginfo("Grasp path total time %f"%(self._grasp_arm_joint_path.points[-1].time_from_start.to_sec(),))
        
        
        return is_executable

    def plan_post_grasp(self):

        self.remove_collision_object("guard")
        self.remove_collision_object("table")
        self._grasp_waypoints[3].position.z += 0.08
        self._post_grasp_plan = self._boris.get_moveit_cartesian_plan("left_hand_arm",self._grasp_waypoints[3])
        # Set back to current
        # self._moveit_wrapper.set_start_state("left_hand_arm", self._boris.joint_names("left_arm"), current_angles)
        is_executable = len(self._post_grasp_plan.joint_trajectory.points) > 0
        if not is_executable:

            rospy.logwarn("Grasp not executable, maybe try again or try new grasp")
        
        else:

            self._post_grasp_plan = self._post_grasp_plan.joint_trajectory

            self._post_grasp_traj_generator.set_waypoints(self._post_grasp_plan)

            rospy.loginfo("Grasp path length %d"%(len(self._post_grasp_plan.points),))
            rospy.loginfo("Grasp path total time %f"%(self._post_grasp_plan.points[-1].time_from_start.to_sec(),))
        
        
        return is_executable

    def execute_pre_grasp(self):

        rospy.loginfo("Pre Grasp Execution!!")
        key = None
        time_steps = np.linspace(0.0,1.0,1000)
        step = 0



        def exec_step(step):

            joint_goal = self._pre_grasp_traj_generator.get_point_t(time_steps[step])

            print "CurrArmState:", self._boris.angles('left_arm')
            print "ArmGoal[%.2f]: "%(time_steps[step],), joint_goal.time_from_start.to_sec(), " step=", step, " pos: ",joint_goal.positions
            cmd = self._boris.cmd_joint_angles(angles=joint_goal.positions,execute=True)
            # self._boris.goto_joint_angles('left_hand',hand_goal.positions)
        
        loop_rate = rospy.Rate(30.0)
        play_forward = False
        play_backward = False
        while not rospy.is_shutdown() and key != 'q':
            
            if self._kbhit.kbhit():
                key = self._kbhit.getch()

            if key == '.':
                
                rospy.loginfo("Step %d"%(step,))
                step = self.update_step(step, time_steps,1)
                exec_step(step)
                
                
            if key == ',':

                step = self.update_step(step, time_steps,-1)
                exec_step(step)

                rospy.loginfo("Step %d"%(step,))

            if key == ']':
                play_forward = True
                play_backward = False
                rospy.loginfo("Playing forward")
            elif key == '[':
                play_forward = False
                play_backward = True
                rospy.loginfo("Playing backward")
            elif key == 'p' and (play_forward or play_backward):
                play_forward = False
                play_backward = False
                rospy.loginfo("Halt open loop playing")

            if play_forward:
                step = self.update_step(step, time_steps,1)
                exec_step(step)
                rospy.loginfo("Step %d"%(step,))
            elif play_backward:
                step = self.update_step(step, time_steps,-1)
                exec_step(step)
                rospy.loginfo("Step %d"%(step,))

            loop_rate.sleep()
        rospy.loginfo("Leaving Pre Grasp Execution!!")

    def execute_grasp(self):
        
       

        rospy.loginfo("Grasp Execution!!")
        key = None
        time_steps = np.linspace(0.0,1.0,200)
        time_steps_arm = np.linspace(0.0,1.0,100)
        step = 0
        step_arm = 0

        def step_grasp(step, arm_step):

            joint_goal = self._arm_traj_generator.get_point_t(time_steps_arm[arm_step])
            hand_goal = self._hand_traj_generator.get_point_t(time_steps[step])

            print "HandGoal: ", hand_goal.time_from_start.to_sec(), " step=", step, " pos: ",hand_goal.positions
            print "CurrArmState:", self._boris.angles('left_arm')
            print "ArmGoal: ", hand_goal.time_from_start.to_sec(), " step=", step, " pos: ",joint_goal.positions
            cmd = self._boris.cmd_joint_angles(angles=joint_goal.positions,execute=True)
            self._boris.goto_joint_angles('left_hand',hand_goal.positions)
        
        loop_rate = rospy.Rate(30.0)
        play_forward = False
        play_backward = False
        while not rospy.is_shutdown() and key != 'q':

            key = None
            if self._kbhit.kbhit():
                key = self._kbhit.getch()

            if key == '.':
                step = self.update_step(step, time_steps,1)
                step_arm = self.update_step(step_arm, time_steps_arm,1)  
                rospy.loginfo("Step %d"%(step,))

                step_grasp(step, step_arm)
                
                
            if key == ',':

                step = self.update_step(step, time_steps,-1)
                step_arm = self.update_step(step_arm, time_steps_arm,-1)    
                rospy.loginfo("Step %d"%(step,))

                step_grasp(step, step_arm)


            if key == ']':
                play_forward = True
                play_backward = False
                rospy.loginfo("Playing forward")
            elif key == '[':
                play_forward = False
                play_backward = True
                rospy.loginfo("Playing backward")
            elif key == 'p' and (play_forward or play_backward):
                play_forward = False
                play_backward = False
                rospy.loginfo("Halt open loop playing")

            if play_forward:
                step = self.update_step(step, time_steps,1)
                step_arm = self.update_step(step_arm, time_steps_arm,1)  
                step_grasp(step, step_arm)
                rospy.loginfo("Step %d"%(step,))
            elif play_backward:
                step = self.update_step(step, time_steps,-1)
                step_arm = self.update_step(step_arm, time_steps_arm,-1)  
                step_grasp(step, step_arm)
                rospy.loginfo("Step %d"%(step,))

            loop_rate.sleep()
        rospy.loginfo("Leaving Grasp Execution!!")


    def step_execution(self, step, time_steps, trajectory_generator, command_func):

        joint_goal = trajectory_generator.get_point_t(time_steps[step])
        command_func(joint_goal.positions)

    def execute_post_grasp(self):

        rospy.loginfo("Post Grasp Execution!!")
        key = None
        time_steps = np.linspace(0.0,1.0,200)
        #time_steps_arm = np.linspace(0.0,1.0,100)
        step = 0

    
        loop_rate = rospy.Rate(30.0)
        play_forward = False
        play_backward = False
        while not rospy.is_shutdown() and key != 'q':

            key = None
            if self._kbhit.kbhit():
                key = self._kbhit.getch()

            if key == '.':
                step = self.update_step(step, time_steps,1)
                rospy.loginfo("Step %d"%(step,))

                self.step_execution(step, time_steps, self._post_grasp_traj_generator, self._boris.cmd_joint_angles)
                
                
            if key == ',':

                step = self.update_step(step, time_steps,-1)
                rospy.loginfo("Step %d"%(step,))

                self.step_execution(step, time_steps, self._post_grasp_traj_generator, self._boris.cmd_joint_angles)


            if key == ']':
                play_forward = True
                play_backward = False
                rospy.loginfo("Playing forward")
            elif key == '[':
                play_forward = False
                play_backward = True
                rospy.loginfo("Playing backward")
            elif key == 'p' and (play_forward or play_backward):
                play_forward = False
                play_backward = False
                rospy.loginfo("Halt open loop playing")

            if play_forward:
                step = self.update_step(step, time_steps,1)
                self.step_execution(step, time_steps, self._post_grasp_traj_generator, self._boris.cmd_joint_angles)
                rospy.loginfo("Step %d"%(step,))
            elif play_backward:
                step = self.update_step(step, time_steps,-1)
                self.step_execution(step, time_steps, self._post_grasp_traj_generator, self._boris.cmd_joint_angles)

                rospy.loginfo("Step %d"%(step,))

            loop_rate.sleep()
            
        rospy.loginfo("Leaving Post Grasp Execution!!")

    def run(self):

        rospy.loginfo("Running!!")
        
        loop_rate = rospy.Rate(30)
        has_solution = False
        has_plan = False
        has_post_grasp_plan = False
        while not rospy.is_shutdown():

            key = self._kbhit.getch()

            if key == "0":
                
                has_solution = self.get_solution()
                rospy.loginfo("Queried solution %s"%(has_solution,))

            elif key == "1" and has_solution:
               #self.goto_pregrasp()
               self.execute_pre_grasp()
            elif key == "2" and has_solution:
                if has_plan:
                    self.execute_grasp()
                else:
                    rospy.logwarn("No grasp plan has been generated. Press 9 to attempt to generate one.")
            elif key == "3" and has_solution:
                if has_post_grasp_plan:
                    self.execute_post_grasp()
                else:
                    rospy.logwarn("No post-grasp plan has been generated. Press 8 to attempt to generate one.")
            elif key == "9":
                has_plan = self.plan_grasp()
            elif key == "8":
                has_post_grasp_plan = self.plan_post_grasp()
            elif key == "r":
                self.remove_collision_object("table")
                self.remove_collision_object("guard")
            elif key == "t":
                self.add_table()
            elif key == "g" and has_solution:
                self.add_object_guard(self._solution)
            elif key =="e":
                self.remove_collision_object("guard")
            elif key == "s":
                self.scan_object()
            elif not has_solution:

                rospy.logwarn("No grasp solution. Press 0.")
                

            

            loop_rate.sleep()

        
        # goto with pre-grasp boris.goto_with_moveit("left_arm",traj_msg.points[0].positions)
        


def main():
    
    grasp_executer = GraspExecuter()

   
    grasp_executer.run()
    ########################################################
    # arm = moveit_commander.MoveGroupCommander("left_hand_arm")

    # print "Tolerances"
    # print "Goal joint tol: ", arm.get_goal_joint_tolerance()
    # print "Goal pos tol: ", arm.get_goal_position_tolerance()
    # print "Goal ori tol: ", arm.get_goal_orientation_tolerance()

    # arm.set_goal_joint_tolerance(0.1) # default 0.0001
    # arm.set_goal_position_tolerance(0.1) # default 0.0001
    # arm.set_goal_orientation_tolerance(0.1) # default 0.001
    
    # arm.set_max_velocity_scaling_factor(0.25)
    # arm.set_max_acceleration_scaling_factor(0.25)

    # arm_initial_pose = arm.get_current_pose().pose
    # arm_initial_joints = arm.get_current_joint_values()

    # gripper = moveit_commander.MoveGroupCommander("left_hand")
    # gripper_joint_values = gripper.get_current_joint_values()
    # print "Left hand: ", gripper_joint_values
    # # ### Open

    # open_cmd = raw_input("Open hand? (y/n)")
    # if open_cmd == "y":
    #     gripper_joint_values[0] = 0.0
    #     gripper.set_joint_value_target(gripper_joint_values)
    #     gripper.go(wait=True)
        
    # ### tf2
    # tf_buffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tf_buffer)
    # br = tf.TransformBroadcaster()

    # ### get target tf

    
    # waypoints = solution2carthesian_path(solution, tf_buffer)    
    # ee_transform = msg2Transform3(waypoints[0])
    
    

    # # print tmp.q.to_euler()
    # # Go to pre-grasp pose
    # target_pose = transform2PoseMsg(ee_transform)
    # arm.set_pose_target(target_pose,end_effector_link="left_hand_palm_link")
    # arm.go(wait=True)
    # rospy.sleep(2)
    # arm.clear_pose_targets()
    # arm.stop()

    # scene.remove_world_object("guard")
    # scene.remove_world_object("table")

    # # Finish grasping
    # arm.set_max_velocity_scaling_factor(0.1)
    # arm.set_max_acceleration_scaling_factor(0.1)
    # success = move_cartesian_path(waypoints[:3], arm)
    # if not success:
    #     arm.set_pose_target(waypoints[2],end_effector_link="left_hand_palm_link")
    #     arm.go(wait=False)
    #     rospy.sleep(1)
    #     # arm.clear_pose_targets()
    #     # arm.stop()

    # # Close hand
    # gripper_joint_values[0] = 0.78
    # gripper.set_joint_value_target(gripper_joint_values)
    # gripper.go(wait=True)
    # rospy.sleep(2)

    # # Move up
    # move_cartesian_path(waypoints[3:], arm)
    


    # rate = rospy.Rate(30)
    # while not rospy.is_shutdown():
    #     br.sendTransform(ee_transform.p,
    #                     ee_transform.q,
    #                     rospy.Time.now(),
    #                     "grasp_goal",
    #                     "world")

    #     rate.sleep()






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
