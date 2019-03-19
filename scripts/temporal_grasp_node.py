#!/usr/bin/env python
# license removed for brevity

import threading, signal
import rospy
import numpy as np

from aml_demos import GraspApp
from aml_math import *

from pcl_service_client import PCLService
from temporal_grasp_ros.srv import *
from temporal_grasp_ros.msg import *
from aml_grasp.grasp_config_test import grasp_config
from grasp_constraints import *
import functools as ft
from aml_io import *
import PyQt4.QtCore as qtc
import PyQt4.QtGui as qtg
import argparse

import tf

SAVE_DATA = False

### TODO: maybe this should be moved to somewhere else
from visualization_msgs.msg import Marker
def publish_crop_cube_maker(crop_min_pt, crop_max_pt):
    pub = rospy.Publisher('crop_marker', Marker, queue_size=1)

    msg = Marker()
    msg.header.frame_id = "world"
    msg.id = 0
    msg.type = Marker.CUBE
    msg.scale.x = crop_max_pt[0] - crop_min_pt[0]
    msg.scale.y = crop_max_pt[1] - crop_min_pt[1]
    msg.scale.z = crop_max_pt[2] - crop_min_pt[2]
    msg.color.r = 0.0
    msg.color.g = 0.0
    msg.color.b = 1.0
    msg.color.a = 0.2
    msg.pose.position.x = 0.5*(crop_max_pt[0] - crop_min_pt[0]) + crop_min_pt[0]
    msg.pose.position.y = 0.5*(crop_max_pt[1] - crop_min_pt[1]) + crop_min_pt[1]
    msg.pose.position.z = 0.5*(crop_max_pt[2] - crop_min_pt[2]) + crop_min_pt[2]
    msg.action = Marker.ADD
    msg.lifetime = rospy.Duration(0)
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0

    print "publishing"
    # HACK: seems it doesn't work with single call
    for i in range(5):
        pub.publish(msg)
        rospy.sleep(0.1) # 10Hz

class GraspAppService(GraspApp):


    def __init__(self):
        super(GraspAppService, self).__init__(grasp_config)
        # GraspApp.__init__(self, kinova_grasp_config)

        arg_fmt = argparse.ArgumentDefaultsHelpFormatter
        parser = argparse.ArgumentParser(formatter_class=arg_fmt)
        parser.add_argument('--sim', dest='sim', action='store_true',
                            help="Specify whether this is a simulated robot")
        parser.set_defaults(sim=False)
        args = parser.parse_args(rospy.myargv()[1:])

        self.robo_vis.add_key_callback(ord('A'), self.acquire_cloud)

        if args.sim:
            self.pcl_service = PCLService(point_cloud_topic="/left_camera/sim/depth_registered/points")
        else:
            self.pcl_service = PCLService(point_cloud_topic="/left_camera/depth_registered/points")

        rospy.init_node('GraspService', anonymous=True)
        self.grasp_service = None

        self.cloud_frame = Transform3()

        self.grasp_generator.add_contraint_callback(hard_kinematic_constraint, ctype='hard')

        # self.grasp_generator.add_contraint_callback(ft.partial(hard_axis_alignment_constraint,directionA=self.cloud_frame.to_matrix()[:3,2], directionB=self.cloud_frame.to_matrix()[:3,0]), ctype='hard')

        # constraint = ft.partial(axis_alignment_constraint,directionA=self.cloud_frame.to_matrix()[:3,2], directionB=self.cloud_frame.to_matrix()[:3,0])
        # self.grasp_generator.add_contraint_callback(constraint, ctype='soft')

        self.restart_grasp_service()


        self.cloud_frame = self.pcl_service.compute_cloud_frame(self.point_cloud.points())
        self.mesh_frame2.transformation = self.cloud_frame.to_matrix()


        self.cloud_name = None


        self._ros_thread = None

        self._cloud_update = None


        self._solution_avail_cb = self.publish_solutions

       
        self._solution_publisher = rospy.Publisher("/grasp/solutions", GraspSolutionSet, queue_size=1)
        self.br = tf.TransformBroadcaster()


    def setup_grasp_service(self):

        if self.grasp_service is None:
            self.grasp_service = rospy.Service('grasp_service/get_solution', GraspSolution, self.get_grasp_solution)
            self.cloud_centroid_srv = rospy.Service('grasp_service/get_cloud_centroid', GraspSolution, self.get_cloud_centroid)


            self.learn_contact_model_srv = rospy.Service('grasp_service/learn_contact_model', GraspServiceCall, self.learn_contact_model_call)
            self.learn_contact_query_srv = rospy.Service('grasp_service/learn_contact_query', GraspServiceCall, self.learn_contact_query_call)
            self.acquire_cloud_srv = rospy.Service('grasp_service/acquire_cloud', GraspServiceCall, self.acquire_cloud_call)
            self.generate_grasps_srv = rospy.Service('grasp_service/generate_grasps', GraspServiceCall, self.generate_grasps_call)
            self.next_solution_srv = rospy.Service('grasp_service/next_solution', GraspServiceCall, self.next_solution_call)
            self.prev_solution_srv = rospy.Service('grasp_service/prev_solution', GraspServiceCall, self.prev_solution_call)

            self.clear_solutions_srv = rospy.Service('grasp_service/clear_solutions', GraspServiceCall, self.clear_solutions_call)

            self.start_cloud_timer_srv = rospy.Service('grasp_service/toggle_cloud_update', GraspServiceCall, self.toggle_cloud_update_call)


            self.service_list = [self.grasp_service,
                                 self.learn_contact_model_srv,
                                 self.learn_contact_query_srv,
                                 self.acquire_cloud_srv,
                                 self.generate_grasps_srv,
                                 self.next_solution_srv,
                                 self.prev_solution_srv,
                                 self.clear_solutions_srv,
                                 self.start_cloud_timer_srv,
                                 self.cloud_centroid_srv]



    def restart_grasp_service(self):

        if self.grasp_service is not None:

            for service in self.service_list:
                service.shutdown()

            self.grasp_service = None


        self.setup_grasp_service()


    def clear_solutions_call(self, req):
        resp = GraspServiceCallResponse()

        self.grasp_generator._optimiser.solutions = []
        self.grasp_generator._solution_buffer = []

        resp.success = True

        return resp.success


    def learn_contact_model(self, req):

        GraspApp.learn_contact_model(self, None)

        self.restart_grasp_service()

    def learn_contact_query(self, req):

        GraspApp.learn_contact_query(self, None)

        self.restart_grasp_service()

# O: Learn contact model
# A: Acquire point cloud in front of robot
# U: Learn query from current cloud
# Y: Generate solutions

    def learn_contact_model_call(self, req):

        resp = GraspServiceCallResponse()
        self.learn_contact_model(None)

        resp.success = True

        self.restart_grasp_service()

        return resp

    def learn_contact_query_call(self, req):

        resp = GraspServiceCallResponse()

        self.learn_contact_query(None)

        resp.success = True

        self.restart_grasp_service()

        return resp


    def acquire_cloud_call(self, req):

        resp = GraspServiceCallResponse()

        self.acquire_cloud(self.robo_vis._vis)

        resp.success = True

        return resp



    def toggle_cloud_update_call(self, req):

        resp = GraspServiceCallResponse()

        if self._cloud_update is None:
            self._cloud_update = rospy.Timer(rospy.Duration(1), self.periodic_cloud_update)
            print "Cloud update started"
        else:
            self._cloud_update.shutdown()
            self._cloud_update = None
            print "Cloud update shudown"

        resp.success = True

        return resp

    def generate_grasps_call(self, req):

        resp = GraspServiceCallResponse()

        # def func():
        #     self.generate_grasps(self.robo_vis._vis)

        # threaded_call = threading.Thread(target=func)
        # threaded_call.start()

        # self.robo_vis.update_entities = False

        # def func():
        #     self.grasp_generator.generate(self.point_cloud, None)
        #     self.robo_vis.update_entities = True
        #     print "Finished Grasp Generation!"

        # threaded_call = threading.Thread(target=func)
        # threaded_call.start()



        self.grasp_generator.generate(self.point_cloud, None)

        resp.success = True

        self.restart_grasp_service()

        return resp

    def next_solution_call(self, req):

        resp = GraspServiceCallResponse()

        self.next_solution(self.robo_vis._vis)

        resp.success = True

        return resp

    def prev_solution_call(self, req):

        resp = GraspServiceCallResponse()

        self.prev_solution(self.robo_vis._vis)

        resp.success = True

        return resp


    def solutions2msg(self):

        solution_set_msg = GraspSolutionSet()
        if self.grasp_generator.has_solutions():


            for solution in self.grasp_generator.get_solutions():

                solution_msg = GraspSolutionMsg()

                solution_msg.joint_angles = solution.joint_angles
                solution_msg.base_pose = list(solution.base_pose[0]) + list(solution.base_pose[1])

                for i in range(len(solution.joint_trajectory)):
                    grasp_waypoint = GraspWaypoint()
                    grasp_waypoint.base_pose = solution.base_trajectory[i].p.tolist() +  solution.base_trajectory[i].q.tolist()
                    grasp_waypoint.joint_angles = solution.joint_trajectory[i].tolist()
                    solution_msg.trajectory.waypoints.append(grasp_waypoint)
                #lift waypoint
                grasp_waypoint = GraspWaypoint()
                grasp_waypoint.joint_angles = solution.joint_trajectory[-1].tolist()
                grasp_waypoint.base_pose = solution.lift_base_pose.p.tolist() +  solution.lift_base_pose.q.tolist()
                solution_msg.trajectory.waypoints.append(grasp_waypoint)

                solution_msg.cloud_min_pt = np.amin(self.point_cloud.points(), axis=0)
                solution_msg.cloud_max_pt = np.amax(self.point_cloud.points(), axis=0)
                solution_msg.cloud_centroid = self.point_cloud.compute_centroid()

                solution_set_msg.solutions.append(solution_msg)

        return solution_set_msg

    def get_grasp_solution(self, req):
        resp = GraspSolutionResponse()

        if self.grasp_generator.has_solutions():
            solution = self.grasp_generator.get_solution(self.sidx)

            resp.joint_angles = solution.joint_angles
            resp.base_pose = list(solution.base_pose[0]) + list(solution.base_pose[1])

            for i in range(len(solution.joint_trajectory)):
                grasp_waypoint = GraspWaypoint()
                grasp_waypoint.base_pose = solution.base_trajectory[i].p.tolist() +  solution.base_trajectory[i].q.tolist()
                grasp_waypoint.joint_angles = solution.joint_trajectory[i].tolist()
                resp.trajectory.waypoints.append(grasp_waypoint)
            #lift waypoint
            grasp_waypoint = GraspWaypoint()
            grasp_waypoint.joint_angles = solution.joint_trajectory[-1].tolist()
            grasp_waypoint.base_pose = solution.lift_base_pose.p.tolist() +  solution.lift_base_pose.q.tolist()
            resp.trajectory.waypoints.append(grasp_waypoint)

        resp.cloud_min_pt = np.amin(self.point_cloud.points(), axis=0)
        resp.cloud_max_pt = np.amax(self.point_cloud.points(), axis=0)
        resp.cloud_centroid = self.point_cloud.compute_centroid()

        if SAVE_DATA:
            models_path = get_aml_package_path('aml_data')
            filepath = models_path + '/exp_data/' + self.cloud_name + "_grasps.pkl"
            save_data({'solution': solution, 'sidx': self.sidx, 'solution_list': self.grasp_generator.get_solutions()}, filepath)
            print "Solution data saved to: ", filepath

        pos = resp.base_pose[0:3]
        quat = resp.base_pose[3:7]
        self.br.sendTransform(pos, quat,
                              rospy.Time.now(),
                              "/target/j2n6s300_link_6",
                              "world")
        return resp

    def get_cloud_centroid(self, req):
        resp = GraspSolutionResponse()

        resp.cloud_min_pt = np.amin(self.point_cloud.points(), axis=0)
        resp.cloud_max_pt = np.amax(self.point_cloud.points(), axis=0)
        resp.cloud_centroid = self.point_cloud.compute_centroid()


        return resp




    def generate_grasps(self, vis):

        def update_vis():
            vis.update_geometry()
            vis.poll_events()
            vis.update_renderer()


        # self.grasp_generator._constraints_cbs = []


        self.grasp_generator.generate(self.point_cloud, update_vis)


        self.restart_grasp_service()

        return False

    def periodic_cloud_update(self, event):

        crop_min_pt=[0.2, 0.3, -0.3] # table surface: z=-0.05
        crop_max_pt=[0.95, 0.9, 0.25]
        # x=0.45, y=0.6, z=0.725
        point_cloud, cloud_frame = self.pcl_service.get_processed_cloud(crop_min_pt=crop_min_pt, crop_max_pt=crop_max_pt)#(crop_min_pt=[-1,-1,-1], crop_max_pt=[1,1,1])#PointCloud(o3d.read_point_cloud("../aml_data/jug.pcd"))

        publish_crop_cube_maker(crop_min_pt, crop_max_pt)




        if point_cloud:
            point_cloud.downsample(0.005)
            diff = np.abs(point_cloud.n_points() - self.point_cloud.n_points())
            has_changed = diff > 200
            if has_changed:
                print "Cloud has changed! Diff is: ", diff
                self.cloud_frame = cloud_frame

                self.set_cloud(point_cloud._cloud, integrate_cloud=False)

                print self.point_cloud._cloud.has_normals(), self.point_cloud._cloud.has_curvatures(), self.point_cloud._cloud.has_principal_curvatures()

    def acquire_cloud(self, vis):
        crop_min_pt=[0.2, 0.3, -0.3]#[0.2, 0.3, -0.5]#[0.2, 0.3, -0.3] # table surface: z=-0.05
        crop_max_pt=[0.95, 0.9, 0.25]

        point_cloud, cloud_frame = self.pcl_service.get_processed_cloud(crop_min_pt=crop_min_pt, crop_max_pt=crop_max_pt)#(crop_min_pt=[-1,-1,-1], crop_max_pt=[1,1,1])#PointCloud(o3d.read_point_cloud("../aml_data/jug.pcd"))
        

        publish_crop_cube_maker(crop_min_pt, crop_max_pt)

        if point_cloud:
            self.cloud_frame = cloud_frame
            point_cloud.downsample(0.005)


            self.set_cloud(point_cloud._cloud)

            print self.point_cloud._cloud.has_normals(), self.point_cloud._cloud.has_curvatures(), self.point_cloud._cloud.has_principal_curvatures()

            self.cloud_frame = self.point_cloud.compute_cloud_frame()
            
            self.mesh_frame2.transformation = self.cloud_frame.to_matrix()
            # vis.add_geometry(self.point_cloud._cloud)
            # self.pcl_service.pause_sub()

            if SAVE_DATA:
                import open3d as o3d
                self.cloud_name = raw_input("Object name: ")
                models_path = get_aml_package_path('aml_data')
                filepath = models_path + '/exp_data/' + self.cloud_name + '.pcd'
                o3d.write_point_cloud(filepath, point_cloud._cloud, True)
                print "Cloud data saved to: ", filepath

        else:

            print "Cloud is empty!"

        self.restart_grasp_service()


    def publish_solutions(self):

        solution_set_msg = self.solutions2msg()

        self._solution_publisher.publish(solution_set_msg)

        pos = solution_set_msg.solutions[0].base_pose[0:3]
        quat = solution_set_msg.solutions[0].base_pose[3:7]
        self.br.sendTransform(pos, quat,
                              rospy.Time.now(),
                              "/target/j2n6s300_link_6",
                              "world")


    def start_ros_spin(self):



        self._ros_thread = threading.Thread(target=rospy.spin, verbose=False)
        self._ros_thread.daemon = True
        self._ros_thread.start()

    # def update_ros():

    #     rate = rospy.Rate(100)

    #     point_cloud = None
    #     while not rospy.is_shutdown():

    #         # if kb.kbhit():
    #         #     c = kb.getch()

    #         #     if c == 'f':
    #         #         print "Calling computing features"
    #         #         result = service.compute_features_current()
    #         #         if result is not None and point_cloud is None:
    #         #             point_cloud = service.msg2point_cloud(result)
    #         #             robo_vis.add_entity(point_cloud)

    #         # robo_vis._update()




    #         rate.sleep()


    #     robo_vis._vis.destroy_window()











if __name__ == '__main__':
    # app = qtg.QApplication(sys.argv)
    grasp_service = GraspAppService()
    # grasp_service.app = app

    grasp_service.start_ros_spin()

    grasp_service.spin()

