#!/usr/bin/env python
import open3d
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2


FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]


def from_ros_msg(ros_point_cloud):
    # Extract fields
    field_names = [field.name for field in ros_point_cloud.fields]
    cloud_data = list(pc2.read_points(ros_point_cloud, skip_nans=True, field_names=field_names))

    # Check empty
    open3d_point_cloud = open3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        return None

    # Fill points
    xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
    open3d_point_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    return open3d_point_cloud


def to_ros_msg(ros_msgs_header, open3d_point_cloud):
    points = np.asarray(open3d_point_cloud.points)
    return pc2.create_cloud(ros_msgs_header, FIELDS_XYZ, points)
