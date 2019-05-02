#!/usr/bin/env python
# license removed for brevity

import threading, signal, time
import rospy
import numpy as np

from aml_demos import SimpleGraspApp
from aml_math import *

from pcl_service_client import PCLService
from temporal_grasp_ros.srv import *
from temporal_grasp_ros.msg import *
from std_msgs.msg import Int32, String
from aml_grasp.grasp_config_test import grasp_config
from grasp_constraints import *
import functools as ft
from aml_io import *
import PyQt4.QtCore as qtc
import PyQt4.QtGui as qtg
import argparse

import tf
import warnings
import cPickle as pickle
import open3d as o3d
from aml_core import PointCloud
from aml_robot import SceneObject
# from pympler.tracker import SummaryTracker


SAVE_DATA = True

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

class GraspAppService(SimpleGraspApp):


    def __init__(self):
        super(GraspAppService, self).__init__(grasp_config)
        # GraspApp.__init__(self, kinova_grasp_config)

        arg_fmt = argparse.ArgumentDefaultsHelpFormatter
        parser = argparse.ArgumentParser(formatter_class=arg_fmt)
        parser.add_argument('--sim', dest='sim', action='store_true',
                            help="Specify whether this is a simulated robot")
        parser.set_defaults(sim=False)
        args = parser.parse_args(rospy.myargv()[1:])

        self.robo_vis.add_key_callback(ord('A'), self.generate_grasps)
        
        self.robo_vis.add_key_callback(ord('F'), self.recompute_features)

        self.robo_vis.add_key_callback(ord('Y'), self.run_evaluation)

        self.robo_vis.add_key_callback(ord('V'), self.load_prev_summary)
        self.robo_vis.add_key_callback(ord('B'), self.load_next_summary)

        

        # with warnings.catch_warnings():
        #     warnings.simplefilter("ignore")
        # self.tracker = SummaryTracker()
            
        # self.robo_vis.add_key_callback(ord('L'), self.show_summary)

        if args.sim:
            self.pcl_service = PCLService(point_cloud_topic="/left_camera/sim/depth_registered/points")
        else:
            self.pcl_service = PCLService(point_cloud_topic="/sim_cloud")#"/left_camera/depth_registered/points")

        rospy.init_node('GraspService', anonymous=True)
        self.grasp_service = None

        self.cloud_frame = Transform3()

        # self.grasp_generator.add_contraint_callback(hard_kinematic_constraint, ctype='hard')

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

        self._object_path_list = ['/home/earruda/Projects/pybullet_cloud/models/object_models/cylinder.urdf',
                                '/home/earruda/Projects/pybullet_cloud/models/object_models/cube_small.urdf',
                                
                                 ]

        self.random_objects = crawl('%s/object_models/random_urdfs'%(self.models_path,),'urdf')
        self._object_path_list = self._object_path_list + [ path for _, path in self.random_objects.items()]
        self._obj_idx = 0#7
        self.random_objects["cylinder.urdf"] = self._object_path_list[0]
        self.random_objects["cube_small.urdf"] = self._object_path_list[1]

        summary_path = self.models_path + '/sim_exp_data/'
        self.grasp_summaries = crawl(summary_path,'pkl')
        self.gsidx = 0

        queue_size = None
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self._solution_publisher = rospy.Publisher("/grasp/solutions", GraspSolutionSet, queue_size=queue_size)

            self._cam_selection_publisher = rospy.Publisher("/sim/next_camera", Int32, queue_size=queue_size)
            self._obj_selection_publisher = rospy.Publisher("/sim/next_object", String, queue_size=queue_size)
            # Set first object
        
        urdf_path = self._object_path_list[self._obj_idx]
        self._obj_selection_publisher.publish(urdf_path)
        self.object.load_object(urdf_path)
        self.object.set_pos_ori(self.object_position,self.object_orientation)

        #for path in self._object_path_list[self._obj_idx:self._obj_idx+52]:
            #obj = SceneObject(filename=path.split("/")[-1], phys_id = self.robot._phys_id, phys_opt='none', fixed_base=False, scale=1.0)
            #obj.set_pos_ori(self.object_position,self.object_orientation)

        self.br = tf.TransformBroadcaster()

        
        self._grasp_trial_summary = {}

    def show_summary(self,vis):

        self.tracker.print_diff()

    def load_prev_summary(self, vis):
        self.gsidx -= 1
        self.gsidx = self.gsidx if self.gsidx >= 0 else 0
        size = len(self.grasp_summaries.items())

        filename, path = self.grasp_summaries.items()[self.gsidx]
        self._grasp_trial_summary = pickle.load( open( path, "rb" ) )

        self.grasp_generator.set_solutions(self._grasp_trial_summary["solution_list"])
        
        position, orientation = self.object_position, self.object_orientation
        try:
            position, orientation = self._grasp_trial_summary["object_position"], self._grasp_trial_summary["object_orientation"]
        except:
            pass

        self.load_urdf_object(self._grasp_trial_summary["object_urdf_path"],
                            position,
                            orientation
                            )
        self.load_cloud(self._grasp_trial_summary["cloud_path"])

        self.grasp_generator.setup_execution_simulation()

        print "Grasp Summary"
        for k, v in self._grasp_trial_summary.items():
            if k != "solution_list":
                print k, ": ", v

        
        print "Gsidx: ", self.gsidx, "/", size

    def load_next_summary(self, vis):

        self.gsidx += 1
        size = len(self.grasp_summaries.items())
        self.gsidx = self.gsidx if self.gsidx < size else (size-1)
        

        filename, path = self.grasp_summaries.items()[self.gsidx]
        self._grasp_trial_summary = pickle.load( open( path, "rb" ) )

        self.grasp_generator.set_solutions(self._grasp_trial_summary["solution_list"])

        position, orientation = self.object_position, self.object_orientation
        try:
            position, orientation = self._grasp_trial_summary["object_position"], self._grasp_trial_summary["object_orientation"]
        except:
            pass

        self.load_urdf_object(self._grasp_trial_summary["object_urdf_path"],
                            position,
                            orientation
                            )
        self.load_cloud(self._grasp_trial_summary["cloud_path"])

        self.grasp_generator.setup_execution_simulation()

        print "Grasp Summary"
        for k, v in self._grasp_trial_summary.items():
            if k != "solution_list":
                print k, ": ", v


        print "Gsidx: ", self.gsidx, "/", size
        

    def load_cloud(self, cloud_path):

        cloud = o3d.read_point_cloud(cloud_path)
        pcloud = PointCloud(cloud)
        pcloud.downsample(0.005)
        self.set_cloud(cloud, integrate_cloud = False)
        
    def load_urdf_object(self, urdf_path, position, orientation):

        self._obj_selection_publisher.publish(urdf_path)
        self.object.load_object(urdf_path)
        self.object.set_pos_ori(position, orientation)

    def setup_grasp_service(self):

        if self.grasp_service is None:
            self.grasp_service = rospy.Service('grasp_service/get_solution', GraspSolution, self.get_grasp_solution)
            self.cloud_centroid_srv = rospy.Service('grasp_service/get_cloud_centroid', GraspSolution, self.get_cloud_centroid)


            self.learn_contact_model_srv = rospy.Service('grasp_service/learn_contact_model', GraspServiceCall, self.learn_contact_model_call)
            self.learn_contact_query_srv = rospy.Service('grasp_service/learn_contact_query', GraspServiceCall, self.learn_contact_query_call)
            self.acquire_cloud_srv = rospy.Service('grasp_service/acquire_cloud', GraspServiceCall, self.acquire_cloud_call)
            self.next_solution_srv = rospy.Service('grasp_service/next_solution', GraspServiceCall, self.next_solution_call)
            self.prev_solution_srv = rospy.Service('grasp_service/prev_solution', GraspServiceCall, self.prev_solution_call)

            self.clear_solutions_srv = rospy.Service('grasp_service/clear_solutions', GraspServiceCall, self.clear_solutions_call)


            self.service_list = [self.grasp_service,
                                 self.learn_contact_model_srv,
                                 self.learn_contact_query_srv,
                                 self.acquire_cloud_srv,
                                 self.next_solution_srv,
                                 self.prev_solution_srv,
                                 self.clear_solutions_srv,
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

        SimpleGraspApp.learn_contact_model(self, None)

        self.restart_grasp_service()

    def learn_contact_query(self, req):

        SimpleGraspApp.learn_contact_query(self, None)

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

    def generate_grasps(self, vis, trial_num = 0, skip_scan = False):

        # Clear grasp summary
        self._grasp_trial_summary = {}

        # Restarting scene cloud and clearing solutions
        urdf_path = self._object_path_list[self._obj_idx]
        self._obj_selection_publisher.publish(urdf_path)
        self.object.load_object(urdf_path)
        self.object.set_pos_ori(self.object_position,self.object_orientation)

        rospy.sleep(2.5)

        self.got_first_cloud = False
        self.grasp_generator._optimiser.solutions = []
        self.grasp_generator._solution_buffer = []
        
        t0_cloud = time.time()
        if not skip_scan:
            for i in range(4):
                self._cam_selection_publisher.publish(i)
                rospy.sleep(3.5)
                self.acquire_cloud(vis)

            self.recompute_features(vis)
        cloud_processing_time = time.time() - t0_cloud

        t0_query = time.time()
        self.learn_contact_query(vis)
        query_time = time.time() - t0_query

        self.grasp_generator.stop_execution_simulation()

        t0_sample = time.time()
        self.sample_solutions(vis)
        sample_time = time.time() - t0_sample

        

        t0_opt = time.time()
        self.grasp_generator.generate(self.point_cloud, None)
        opt_time = time.time() - t0_opt
        
        # self.save_solutions(vis, path = path)

        total_time = cloud_processing_time + query_time + sample_time + opt_time
        grasp_generation_time = query_time + sample_time + opt_time
  
        rospy.loginfo("Total generation time: %f"%(total_time,))
        rospy.loginfo("Grasp generation time: %f"%(grasp_generation_time,))

        self._grasp_trial_summary["skip_scan"] = skip_scan
        self._grasp_trial_summary["object_idx"] = self._obj_idx
        self._grasp_trial_summary["object_name"] = urdf_path.split('/')[-1].split('.')[0]
        self._grasp_trial_summary["object_urdf_path"] = urdf_path
        self._grasp_trial_summary["cloud_processing_time"] = cloud_processing_time
        self._grasp_trial_summary["query_time"] = query_time
        self._grasp_trial_summary["sample_time"] = sample_time
        self._grasp_trial_summary["sample_time"] = sample_time
        self._grasp_trial_summary["opt_time"] = opt_time
        self._grasp_trial_summary["grasp_generation_time"] = grasp_generation_time
        
        cloud_filename = self._grasp_trial_summary["object_name"] + "_%.4d_model"%(trial_num,)

        cloud_path = self.save_current_cloud(cloud_filename)
        self._grasp_trial_summary["cloud_path"] = cloud_path

        self._grasp_trial_summary["solution_list"] = self.grasp_generator.get_solutions()
        
        # -1 means not tried/unknown
        self._grasp_trial_summary["success_map"] = [-1]*self.grasp_generator.solution_count()
        self._grasp_trial_summary["success_count"] = 0
        self._grasp_trial_summary["trial_number"] = -1 #unset

        self._grasp_trial_summary["object_position"] = self.object_position
        self._grasp_trial_summary["object_orientation"] = self.object_orientation

        
        # self._obj_selection_publisher.publish(self._object_path_list[self._obj_idx])
        
        self._obj_idx = (self._obj_idx+1)%len(self._object_path_list)
        
        
        return False

    def save_current_cloud(self, filename):


        filepath = self.models_path + '/sim_exp_data/' + filename + '.pcd'
        o3d.write_point_cloud(filepath, self.point_cloud._cloud, True)
        print "Cloud data saved to: ", filepath
        # models_path = get_aml_package_path('aml_data')
        # filepath = models_path + '/exp_data/' + self.cloud_name + "_grasps.pkl"
        # save_data({'solution': solution, 'sidx': self.sidx, 'solution_list': self.grasp_generator.get_solutions()}, filepath)
        # print "Solution data saved to: ", filepath

        return filepath

    def run_evaluation(self, vis):
        
        n_objects = 60
        trials_per_object = 10
        n_best_to_try = 10
        summary_path = self.models_path + '/sim_exp_data/'

        for i in range(n_objects):
            
            for trial in range(trials_per_object):
                self._obj_idx = i
                self.generate_grasps(vis, trial_num=trial, skip_scan=False)
                

                self._grasp_trial_summary["trial_number"] = trial

                success = 0
                for solution_exec in range(n_best_to_try):
                    result = int(self.try_grasp2(solution_exec))
                    self._grasp_trial_summary["success_map"][solution_exec] = result

                    if solution_exec == 0 and result == 1:
                        success = 1
                
                self._grasp_trial_summary["success_count"] += success

                filename = "%.2f"%(time.time(),) + "_t%.4d_"%(trial) + self._grasp_trial_summary["object_name"] + ".pkl"
                pickle.dump( self._grasp_trial_summary, open( summary_path + filename, "wb" ) )





    def acquire_cloud_call(self, req):

        resp = GraspServiceCallResponse()

        self.acquire_cloud(self.robo_vis._vis)

        resp.success = True

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
            import open3d as o3d
            self.cloud_name = raw_input("Object name: ")
            models_path = get_aml_package_path('aml_data')
            filepath = models_path + '/exp_data/' + self.cloud_name + '.pcd'
            o3d.write_point_cloud(filepath, self.point_cloud._cloud, True)
            print "Cloud data saved to: ", filepath

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


    def acquire_cloud(self, vis):
        crop_min_pt=[0.2, 0.3, -0.32]#[0.2, 0.3, -0.5]#[0.2, 0.3, -0.3] # table surface: z=-0.05
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

        else:

            print "Cloud is empty!"

        self.restart_grasp_service()

    def recompute_features(self, vis):

        
        o3d_point_cloud = self.pcl_service.recompute_curvatures(self.point_cloud._cloud)

        self.set_cloud(o3d_point_cloud, integrate_cloud=False)

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

