#!/usr/bin/env python

from temporal_grasp_ros.srv import *
import rospy
import numpy as np

class GraspServiceClient(object):

    
    def __init__(self):


        print "Waiting for PCL service"
        rospy.wait_for_service('grasp_service/get_solution')
        print "Service found!"
        try:
            self.get_grasp_solution_srv = rospy.ServiceProxy('grasp_service/get_solution', GraspSolution)
        except rospy.ServiceException, e:
            print "Service proxy setup failed: %s"%e

    def get_grasp_solution(self):

        grasp_solution = None

        try:

            # request = PCLUtilityRequest()
            # request.function_call_id = "compute_features"
            # request.cloud_input = cloud2_msg
            response = self.get_grasp_solution_srv(GraspSolutionRequest())
            # gsp = GraspSolutionResponse()

            
            joint_trajectory = [ np.asarray(wpt.joint_angles) for wpt in response.trajectory.waypoints ]
            base_trajectory = [ {'p': np.asarray(wpt.base_pose)[:3], 'q': np.asarray(wpt.base_pose)[3:]} for wpt in response.trajectory.waypoints ]
            grasp_solution = {'joint_angles': response.joint_angles,
                              'base_pose': {'p': np.asarray(response.base_pose)[:3], 'q': np.asarray(response.base_pose)[3:]},
                              'base_trajectory': base_trajectory,
                              'joint_trajectory': joint_trajectory,
                              'cloud_min_pt': response.cloud_min_pt,
                              'cloud_max_pt': response.cloud_max_pt,
                              'cloud_centroid': response.cloud_centroid
                              }

        except rospy.ServiceException, e:

            print "Service call to compute_features_srv failed: %s"%e

        return grasp_solution


if __name__ == '__main__':
    rospy.init_node("grasp_service_client")
    rate = rospy.Rate(1)

    grasp_service_client = GraspServiceClient()

    while not rospy.is_shutdown():

        print grasp_service_client.get_grasp_solution()

        rate.sleep()