#!/usr/bin/env python

# Import modules
from pcl_helper import *
import rospy
import pcl
import numpy as np
import ctypes
import struct
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from random import randint

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    #  Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    #cloud = pcl.load_XYZRGB(pcl_cloud_ros)

    #  Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    #  PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.76
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)

    cloud_filtered = passthrough.filter()

    #  RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = .015
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    #  Extract inliers and outliers
    # Extract inliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    #  Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    #create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    #PARAMETERS for distance threshold, min and max cluster size
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(2000)
    #search for cluster 
    ec.set_SearchMethod(tree)
    #extract indices for each discovered cluster
    cluster_indices = ec.Extract()
    #print(len(cluster_indices))

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []    
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                              rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    
    #  Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects) 
    #  Publish ROS messages
    pcl_table_pub.publish(ros_cloud_table)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_cluster_cloud_pub.publish(ros_cluster_cloud)

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    #  Create Subscribers
    pcl_sub = rospy.Subscriber('/sensor_stick/point_cloud',pc2.PointCloud2, pcl_callback, queue_size=1)
    #  Create Publishers
    pcl_objects_pub = rospy.Publisher('/pcl_objects',PointCloud2,queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table',PointCloud2,queue_size=1)
    pcl_cluster_cloud_pub = rospy.Publisher('/pcl_cluster_cloud',PointCloud2,queue_size=1)
    # Initialize color_list
    get_color_list.color_list = []

    #  Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
