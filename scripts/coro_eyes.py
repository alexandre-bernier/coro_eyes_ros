#!/usr/bin/env python
import open3d

import rospy
from pc_utility import convert
from sensor_msgs.msg import PointCloud2
from coro_eyes_ros.srv import *


def ask_for_command():
    display = '**********************\n'
    display += '* Available commands *\n'
    display += '**********************\n'
    display += 's: Single scan\n'
    display += 'v: Save last scan\n'
    display += 'q: Quit\n'
    display += '--> '

    return input(display)


def post_processing(raw_point_cloud):
    # Filter outliers
    new_point_cloud, ind = raw_point_cloud.remove_radius_outlier(nb_points=100, radius=0.01)

    return new_point_cloud


if __name__ == '__main__':
    # Define variables
    raw_o3d_point_cloud = post_o3d_point_cloud = open3d.geometry.PointCloud()
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

        # Save
        if cmd == 'v':
            if post_o3d_point_cloud.is_empty():
                rospy.loginfo("Can't save point cloud. Previous scan is empty.\n")
            else:
                open3d.io.write_point_cloud("/home/alexandre/point_cloud.ply", post_o3d_point_cloud, write_ascii=True)

        # Single scan
        if cmd == 's':
            # Start a scan
            raw_point_cloud = scan_srv()
            # Convert the ROS message into a Open3d point cloud
            raw_o3d_point_cloud = convert.from_ros_msg(raw_point_cloud.point_cloud)

            # Post-processing
            post_o3d_point_cloud = post_processing(raw_o3d_point_cloud)

            # Convert the processed point cloud into a ROS message
            post_point_cloud = convert.to_ros_msg(raw_point_cloud.point_cloud.header, post_o3d_point_cloud)
            # Publish the processed point cloud
            pub.publish(post_point_cloud)

        rospy.sleep(0.1)
