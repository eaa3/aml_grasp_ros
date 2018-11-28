#!/usr/bin/env python

import rospy
from aml_core import PointCloud
from aml_graphics import Open3DVisualiser as Visualiser

import open3d as o3d
import numpy as np

import sys, math
import std_msgs.msg

import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2

if __name__ == '__main__':

    rospy.init_node('MockCloudPublisher', anonymous=True)

    print "Creating point cloud!"
    objects = ['atissuebox_video.pcd', 'btb_kde_video.pcd', 'clutter01_video.pcd','tissuebox_kde_video.pcd','bottle_video.pcd','boxtoppled_and_box_video.pcd']
    pcd = o3d.read_point_cloud("/hri/localdisk/earruda/development/libs/pd_grasp/aml_data/video_objects/%s"%(objects[5],))
    # pcd = o3d.PointCloud()
    # pcd.points = np.vstack([np.array([[0.5, 0.0, 0.0], [0.5, 0.5, 0], [0.5, 0, 0.5]]),np.random.randn(97,3)*0.1])#
    # pcd.colors = np.vstack([np.array([[0.5, 0.0, 0.0], [0.5, 0.5, 0], [0.5, 0, 0.5]]),    np.zeros((97,3))])
    points = np.asarray(pcd.points)
    point_cloud = PointCloud(pcd)

    pcl_pub = rospy.Publisher("/point_cloud_in", PointCloud2, queue_size=1)

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'world'
    #create pcl from points
    point_cloud2 = pcl2.create_cloud_xyz32(header, points)

    
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():

        pcl_pub.publish(point_cloud2)
        rate.sleep()