#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
#from pr2_robot.cd  import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

def statistical_outlier(cloud, k_mean,threshold_scale_factor):

    # Much like the previous filters, we start by creating a filter object:
    outlier_filter = cloud.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(k_mean)
    # Set threshold scale factor
    x = threshold_scale_factor
    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)
    # Finally call the filter function for magic

    return outlier_filter.filter()

def voxel_grid_downsampling(cloud, LEAF_SIZE):

    vox = cloud.make_voxel_grid_filter()
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    return vox.filter()

def passThrought_filter(cloud, filter_axis, axis_min, axis_max):

    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

def RANSAC_PLANE(cloud, max_distance):
    seg = cloud.make_segmenter()
    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    #segmenting the table
    seg.set_distance_threshold(max_distance)
    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
    # TODO: Extract inliers and outliers
    cloud_table = cloud.extract(inliers, negative=False)
    cloud_objects = cloud.extract(inliers, negative=True)
    return cloud_table, cloud_objects

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    #PARAMETERS
    scene_number = 3
    k_mean=40
    threshold_scale_factor = 0.05

    LEAF_SIZE=0.01

    filter_axis = 'z'
    axis_min = 0.60
    axis_max = 1.1

    max_distance = 0.01

    cluster_tolerance = 0.01
    min_cluster_size = 9
    max_cluster_size = 1000

    # : Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # : Voxel Grid Downsampling
    cloud = voxel_grid_downsampling(cloud,LEAF_SIZE)
    # : Statistical Outlier Filtering
    cloud_stats = statistical_outlier(cloud,k_mean,threshold_scale_factor)
    # : PassThrough Filter
    cloud = passThrought_filter(cloud_stats,filter_axis,axis_min,axis_max)
    cloud = passThrought_filter(cloud,'y',-0.5,0.5)
    # : RANSAC Plane Segmentation
    cloud_table , cloud_objects = RANSAC_PLANE(cloud, max_distance)
    # : Euclidean Clustering
    cloud_objects = statistical_outlier(cloud_objects,k_mean,threshold_scale_factor)
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    #create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    #PARAMETERS for distance threshold, min and max cluster size
    ec.set_ClusterTolerance(cluster_tolerance)
    ec.set_MinClusterSize(min_cluster_size)
    ec.set_MaxClusterSize(max_cluster_size)
    #search for cluster
    ec.set_SearchMethod(tree)
    #extract indices for each discovered cluster
    cluster_indices = ec.Extract()
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    rospy.loginfo('clusters detected {}  '.format(len(cluster_indices)))

    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                              rgb_to_float(cluster_color[j])])


    cluster_cloud = pcl.PointCloud_PointXYZRGB()

    cluster_cloud.from_list(color_cluster_point_list)
    # TODO: Convert PCL data to ROS messages
    ros_cloud_stats = pcl_to_ros(cloud_stats)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # TODO: Publish ROS messages
    pcl_stats_pub.publish(ros_cloud_stats)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_cluster_cloud_pub.publish(ros_cluster_cloud)
# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=False)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += 0.4
        object_markers_pub.publish(make_label(label,label_pos,index))
        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Publish the list of detected objects
    rospy.loginfo('Detected {} objects {}'.format(len(detected_objects_labels), detected_objects_labels))

    detected_objects_pub.publish(detected_objects)
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    for object in detected_objects:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])
    detected_objects_list = []
    detected_objects_list.append(labels)
    detected_objects_list.append(centroids)



    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects,scene_number)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list,scene_number):

    # TODO: Initialize variables
    yaml_dicts = []
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()
    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')

    dropbox_params = rospy.get_param('/dropbox')
    left_place_position = dropbox_params[0]['position']
    right_place_position = dropbox_params[1]['position']

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # : Loop through the pick list
    test_scene_num.data = scene_number
    for i in range(0,len(object_list_param)):
        for obj in object_list:
            if obj.label == object_list_param[i]['name']:
                # : Get the PointCloud for a given object and obtain it's centroid
                # : Create 'place_pose' for the object
                # : Assign the arm to be used for pick_place
                object_name.data = object_list_param[i]['name']
                points_arr = ros_to_pcl(obj.cloud).to_array()
                pos = np.mean(points_arr, axis=0)[:3]

                pick_pose.position.x = np.asscalar(pos[0])
                pick_pose.position.y = np.asscalar(pos[1])
                pick_pose.position.z = np.asscalar(pos[2])
                if object_list_param[i]['group'] == 'red':
                    arm_name.data = 'left'
                    place_pose.position.x = left_place_position[0]
                    place_pose.position.y = left_place_position[1]
                    place_pose.position.z = left_place_position[2]
                else:
                    arm_name.data = 'right'
                    place_pose.position.x = right_place_position[0]
                    place_pose.position.y = right_place_position[1]
                    place_pose.position.z = right_place_position[2]


        # : Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        yaml_dicts.append(yaml_dict)
        '''
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        '''
    # : Output your request parameters into output yaml file
    file_name = 'output_'+ str(scene_number) +'.yaml'
    send_to_yaml(file_name, yaml_dicts)

if __name__ == '__main__':

    # : ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # : Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points',pc2.PointCloud2, pcl_callback, queue_size=1)
    # : Create Publishers
    pcl_stats_pub = rospy.Publisher('/pcl_stats',PointCloud2,queue_size=1)
    pcl_objects_pub = rospy.Publisher('/pcl_objects',PointCloud2,queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table',PointCloud2,queue_size=1)
    pcl_cluster_cloud_pub = rospy.Publisher('/pcl_cluster_cloud',PointCloud2,queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size = 1)
    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)
    # : Load Model From disk
    scene_number = 3
    model_file = 'model' + str(scene_number) + '.sav'
    model = pickle.load(open(model_file,'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    # : Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
