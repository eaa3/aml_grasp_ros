#!/usr/bin/env python
# license removed for brevity
import rospy
import tf2_ros
from std_msgs.msg import String
import geometry_msgs.msg
# import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

import sys, threading
import numpy as np

from temporal_grasp_ros.srv import PCLUtilityRequest, PCLUtility


import open3d as o3d
from aml_core import PointCloud
from aml_graphics import Open3DVisualiser as Visualiser
from aml_math import Transform3

# from __future__ import print_function

"""
Serialization of sensor_msgs.PointCloud2 messages.

Author: Tim Field
"""

import ctypes
import math
import struct

import roslib.message
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, roslib.message.Message) and cloud._type == 'sensor_msgs/PointCloud2', 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def create_cloud(header, fields, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message.

    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param fields: The point cloud fields.
    @type  fields: iterable of L{sensor_msgs.msg.PointField}
    @param points: The point cloud points.
    @type  points: list of iterables, i.e. one iterable for each point, with the
                   elements of each iterable being the values of the fields for 
                   that point (in the same order as the fields parameter)
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """

    cloud_struct = struct.Struct(_get_struct_fmt(False, fields))

    buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buff, offset, *p)
        offset += point_step

    return PointCloud2(header=header,
                       height=1,
                       width=len(points),
                       is_dense=False,
                       is_bigendian=False,
                       fields=fields,
                       point_step=cloud_struct.size,
                       row_step=cloud_struct.size * len(points),
                       data=buff.raw)

def create_cloud_xyz32(header, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message with 3 float32 fields (x, y, z).

    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param points: The point cloud points.
    @type  points: iterable
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    return create_cloud(header, fields, points)

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'
    i = 0
    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print 'Skipping unknown PointField datatype [%d]' % field.datatype
        else:
            
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            if field.name == 'rgb':
                # For some reason the colour field is having a wrong data type. It should be unsigned int 32 bits. This is a hard-coded fix
                datatype_fmt, datatype_length = _DATATYPES[PointField.UINT32]
            # print datatype_fmt, field.name
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


def get_transfrom(buf, child_frame_id):
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = buf.lookup_transform('world', child_frame_id, rospy.Time(0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("failed to lookup transform")
            rate.sleep()
            continue

    target = geometry_msgs.msg.Pose()
    target.position = trans.transform.translation
    target.orientation = trans.transform.rotation

    # qy = tt.quaternion_about_axis(-0.5*math.pi, (0, 1, 0))
    # qx = tt.quaternion_about_axis(math.pi, (1, 0, 0))
    # q0 = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
    # q = tt.quaternion_multiply(q0, qy)
    # q = tt.quaternion_multiply(q, qx)

    # target.orientation.x = q[0]
    # target.orientation.y = q[1]
    # target.orientation.z = q[2]
    # target.orientation.w = q[3]

    return target

class PCLService(object):

    def __init__(self, point_cloud_topic="/camera/depth_registered/points"): #"/output"

        self._point_cloud_topic = point_cloud_topic


        print "Waiting for PCL service"
        rospy.wait_for_service('aml_pcl_service')
        print "Service found!"
        try:
            self.compute_features_srv = rospy.ServiceProxy('aml_pcl_service', PCLUtility)
        except rospy.ServiceException, e:
            print "Service proxy setup failed: %s"%e


        # Shared variables
        self._cloud_lock = threading.Lock()
        self._transform = None
        self._current_cloud = None
        # self._feature_cloud_lock = threading.Lock()
        self._current_feature_cloud = None

        self._all_fields_order = ['x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z', 'rgb', 'curvature', 'principal_curvature_x', 'principal_curvature_y', 'principal_curvature_z', 'pc1', 'pc2']

        self._sub = rospy.Subscriber(self._point_cloud_topic, PointCloud2, self.point_cloud_cb, queue_size=1)

    def pause_sub(self):

        self._sub.unregister()

    def resume(self):

        self._sub = rospy.Subscriber(self._point_cloud_topic, PointCloud2, self.point_cloud_cb, queue_size=1)

    def msg2array(self, msg):

        all_fields = [field.name for field in msg.fields]
        point_data_generator = read_points(msg, field_names=all_fields, skip_nans=False)

        size = msg.width*msg.height
        dims = len(msg.fields)

        points = np.zeros((size,dims))
        colors = np.zeros((size,1),dtype=np.int32)
        # print "Size: ", size, " Dims: ", dims
        i = 0
        for p in point_data_generator:
            # print p
            p_dict = dict(zip(all_fields,p))
            p_row = np.array([p_dict[k] for k in self._all_fields_order])
            colors[i] = p_dict['rgb']
            points[i,:] = p_row

            # print "Color: ", int(p_dict['rgb'])
            i += 1
            # if points is None:
            #     points = p_row
            # else:
            #     points = np.vstack([points,p_row])


            # for p in list(point_data):

            #     for f, v in zip(all_fields,p):
            #         sys.stdout.write('%s: %f '%(f,v))
            #     sys.stdout.write('\n')
            #     sys.stdout.flush()


        return points

    def msg2point_cloud(self, msg):

        data = self.msg2array(msg)

        # print data
        # print "Points shape", data.shape

        def toRGBReal(uint_rgb):
            blue =  uint_rgb & 255
            green = (uint_rgb >> 8) & 255
            red =   (uint_rgb >> 16) & 255
            # print red, green, blue
            return np.array([float(red)/255.0, float(green)/255.0, float(blue)/255.0])


        
        if data.shape[0] > 0:
            # print "Sample row: ", data[0,:]


            pcd = o3d.PointCloud()
            pcd.points = data[:,:3]
            # normals = np.zeros((points.shape[0],3))
            # normals[:,:2] = points[:,11:] # using curvatures just for visualisation
            pcd.normals = data[:,3:6] # curvatures: points[:,8:11]
            
            colors = np.zeros((data.shape[0],3))
            for i in range(data.shape[0]):
                color = int(data[i,6])
                colors[i] = toRGBReal(color)
                # print toRGBReal(color)
            pcd.colors = colors
            # pcd.colors = points[:,6] convert uint32 to rgb
            pcd.curvatures = data[:,7]
            pcd.principal_curvatures = data[:,8:]

            point_cloud = PointCloud(pcd)

            cloud_frame = self.compute_cloud_frame(data)


            return point_cloud, cloud_frame
        else:
            return None, Transform3()


    def compute_cloud_frame(self, data):

        # print "COMPUTING COVARIANCE"
        centroid = np.mean(data[:,:3], axis=0)

        covariance = np.cov((data[:,:3] - centroid).T )
        # print "COMPUTING Eigen"
        evals, evecs = np.linalg.eig(covariance)

        sort_indices = np.argsort(evals)[::-1]
        vec0 = evecs[:, sort_indices[0]]  # Eigenvector with largest eigenvalue
        vec1 = evecs[:, sort_indices[1]]
        vec2 = evecs[:, sort_indices[2]]

        # print "Evals: ", evals[sort_indices], evals.shape
        # print "Vectors: ", vec0, vec1, vec2, vec0.shape, np.linalg.norm(vec0), np.linalg.norm(vec1), np.linalg.norm(vec2)

        cloud_frame = Transform3()

        cloud_frame.set_from_axis_point(vec0, vec1, vec2, centroid)
        # cloud_frame.inverted()
        return cloud_frame

    def point_cloud_cb(self, cloud2_msg):

        # Lock mutex
        self._cloud_lock.acquire()

        # Modify shared variable
        if self._current_cloud is not None:
            del self._current_cloud
            self._current_cloud = None
        self._current_cloud = cloud2_msg

        # Release mutex
        self._cloud_lock.release()


    def compute_features(self, cloud2_msg, view_point, crop_min_pt, crop_max_pt):

        cloud_output = None

        try:

            # request = PCLUtilityRequest()
            # request.function_call_id = "compute_features"
            # request.cloud_input = cloud2_msg
            response = self.compute_features_srv("compute_features",cloud2_msg, view_point, crop_min_pt, crop_max_pt)

            if response.success:
                cloud_output = response.cloud_output

                # print "Eigen values: ", response.eigen_values
                # print "Eigen Vec0: ", response.eigen1
                # print "Eigen Vec1: ", response.eigen2
                # print "Eigen Vec2: ", response.eigen3
        except rospy.ServiceException, e:

            print "Service call to compute_features_srv failed: %s"%e

        return cloud_output

    def compute_features_current(self, view_point=[1.35, -0.9, 0.32], crop_min_pt=[0.2, -0.40, -0.01], crop_max_pt=[0.6, 0.0, 0.2]):

        feature_cloud = None

        # Lock mutex
        self._cloud_lock.acquire()

        if self._current_cloud is None:
            print "Cloud is None"
            self._cloud_lock.release()
        else:

            print "Calling service"
            feature_cloud = self.compute_features(self._current_cloud, view_point, crop_min_pt, crop_max_pt)

            self._cloud_lock.release()

        return feature_cloud


    def get_processed_cloud(self, **kwargs):

        # view_point = kwargs.get('view_point', [1.35, -0.9, 0.32])
        # crop_min_pt = kwargs.get('crop_min_pt',[0.2, -0.40, -0.01])
        # crop_max_pt = kwargs.get('crop_max_pt',[0.6, 0.0, 0.2])
        
        view_point = kwargs.get('view_point', [0.469, 1.05, 0.19])
        crop_min_pt = kwargs.get('crop_min_pt',[0.0, 0.0, -0.30])#[0.2, -0.40, -0.01])
        crop_max_pt = kwargs.get('crop_max_pt',[0.848, 0.886, 0.05])

        result = self.compute_features_current(view_point, crop_min_pt, crop_max_pt)

        point_cloud = None
        cloud_frame = Transform3()
        if result is not None:
            point_cloud, cloud_frame = self.msg2point_cloud(result)


        # if point_cloud is None:
        #     raise Exception("Unable to acquire processed cloud. Is the topic %s receiving point cloud messages?"%(self._point_cloud_topic,))


        return point_cloud, cloud_frame


if __name__ == '__main__':
    rospy.init_node('writeCloudsToFile', anonymous=True)

    cloud_topic = rospy.get_param("cloud_topic", "/left_camera/depth_registered/points")

    service = PCLService(point_cloud_topic=cloud_topic)#(point_cloud_topic='/point_cloud_in')

    from kbhit import KBHit

    kb = KBHit()


    rate = rospy.Rate(50)

    robo_vis = Visualiser()
    point_cloud = None
    while not rospy.is_shutdown():

        if kb.kbhit():
            c = kb.getch()

            if c == 'f':
                print "Calling computing features"
                result = service.compute_features_current()
                if result is not None and point_cloud is not None:
                    point_cloud, cloud_frame = service.msg2point_cloud(result)
                    robo_vis.add_entity(point_cloud)

        robo_vis._update()




        rate.sleep()

    robo_vis._vis.destroy_window()