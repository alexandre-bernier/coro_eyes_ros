#!/usr/bin/env python
import open3d
import numpy as np

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from coro_eyes_ros.srv import *


FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]


def from_ros_msgs(ros_point_cloud):
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


def to_ros_msgs(ros_msgs_header, open3d_point_cloud):
    points = np.asarray(open3d_point_cloud.points)
    return pc2.create_cloud(ros_msgs_header, FIELDS_XYZ, points)


def ask_for_command():
    display = '**********************\n'
    display += '* Available commands *\n'
    display += '**********************\n'
    display += 's: Single scan\n'
    display += 'q: Quit\n'
    display += '--> '

    return input(display)


def post_processing(raw_point_cloud):
    # Filter outliers
    new_point_cloud, ind = raw_point_cloud.remove_radius_outlier(nb_points=14, radius=5)

    return new_point_cloud


if __name__ == '__main__':
    # Init
    rospy.init_node('post_processing', anonymous=True, disable_signals=True)
    # Scan service
    rospy.wait_for_service('/coro_eyes/Scan')
    scan_srv = rospy.ServiceProxy('/coro_eyes/Scan', Scan)
    # Publisher
    pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=1)

    rospy.loginfo("CoRo Eyes application started.\n")

    while not rospy.is_shutdown():

        cmd = ask_for_command()
        print()  # Add an empty line after the command

        # Quit
        if cmd == 'q':
            rospy.signal_shutdown("Shutting down CoRo Eyes application...")

        # Single scan
        if cmd == 's':
            # Start a scan
            raw_point_cloud = scan_srv()
            # Convert the ROS message into a Open3d point cloud
            raw_o3d_point_cloud = from_ros_msgs(raw_point_cloud.point_cloud)

            # Post-processing
            post_o3d_point_cloud = post_processing(raw_o3d_point_cloud)

            # Convert the processed point cloud into a ROS message
            post_point_cloud = to_ros_msgs(raw_point_cloud.point_cloud.header, post_o3d_point_cloud)
            # Publish the processed point cloud
            pub.publish(post_point_cloud)

        rospy.sleep(0.1)
