#!/usr/bin/env python
import sys
import open3d
import numpy as np

import rospy
from pc_utility import convert
from sensor_msgs.msg import PointCloud2


def visualize(ros_point_cloud):
    global point_cloud, point_cloud_updated
    point_cloud = convert.from_ros_msg(ros_point_cloud)
    point_cloud_updated = True


if __name__ == '__main__':
    # Variables
    point_cloud = open3d.geometry.PointCloud()
    point_cloud_updated = False
    first_point_cloud = True

    # Check arguments
    if len(sys.argv) < 2:
        rospy.logerr("Usage: rosrun coro_eyes_ros pc_visualizer.py point_cloud_topic")

    # Init
    rospy.init_node('point_cloud_visualizer', anonymous=True)
    rospy.Subscriber(sys.argv[1], PointCloud2, visualize)

    # Start visualizer
    vis = open3d.visualization.Visualizer()
    vis.create_window()
    vis.get_view_control().set_front([-0.005, 0.153, -0.988])
    vis.get_view_control().set_lookat([0.133, -0.007, 0.744])
    vis.get_view_control().set_up([0.002, -0.988, -0.153])

    # Loop
    while not rospy.is_shutdown():
        if point_cloud_updated:
            point_cloud_updated = False
            # Save current viewpoint
            viewpoint = vis.get_view_control().convert_to_pinhole_camera_parameters()
            # Display newest point cloud
            vis.clear_geometries()
            vis.add_geometry(point_cloud)
            # Restore viewpoint
            vis.get_view_control().convert_from_pinhole_camera_parameters(viewpoint)

            # Set zoom on the first point cloud only
            if first_point_cloud:
                first_point_cloud = False
                vis.get_view_control().set_zoom(0.4)  # Somehow, set_zoom doesn't work until there are points displayed

        # Keep the window responsive
        vis.poll_events()
        vis.update_renderer()

        rospy.sleep(0.1)
