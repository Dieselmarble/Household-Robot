#!/usr/bin/env python3

import sys
import cv2
import numpy as np
import math

import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import MarkerArray
from vision_msgs.msg import BoundingBox2D
from vision_msgs.msg import BoundingBox2DArray
import ros_numpy
import message_filters
import struct
import detection_ros_markers_python3 as dr
import detection_2d_to_3d_python3 as d2


class IdentificationNode:
    def __init__(self, identifier, default_marker_name, node_name,
                topic_base_name, fit_plane, min_box_side_m=None,
                max_box_side_m=None, modify_3d_detections=None):
        self.rgb_image = None
        self.rgb_image_timestamp = None 
        self.camera_info = None
        self.marker_array = None
        self.all_points = []
        self.publish_marker_point_clouds = True

        self.identifier = identifier
        self.marker_collection = dr.DetectionBoxMarkerCollection(default_marker_name)
        self.topic_base_name = topic_base_name
        self.node_name = node_name
        self.image_count = 0


    def image_callback(self, ros_rgb_image, bounding_boxes):
        self.rgb_image = ros_numpy.numpify(ros_rgb_image)
        self.rgb_image_timestamp = ros_rgb_image.header.stamp
        self.camera_info = rgb_camera_info
        self.marker_array = marker_array
        self.bounding_boxes = bounding_boxes
        self.image_count = self.image_count + 1

        # OpenCV expects bgr images, but numpify by default returns rgb images.
        self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)
        
        # Copy the depth image to avoid a change to the depth image
        # during the update.
        time_diff = self.rgb_image_timestamp - self.depth_image_timestamp
        time_diff = abs(time_diff.to_sec())
        if time_diff > 0.0001:
            print('WARNING: The rgb image and the depth image were not taken at the same time.')
            print('         The time difference between their timestamps =', closest_time_diff, 's')

        # Rotate the image by 90deg to account for camera
        # orientation. In the future, this may be performed at the
        # image source.
        #detection_box_image = cv2.rotate(self.rgb_image, cv2.ROTATE_90_CLOCKWISE)
        
        identification_box_image = self.rgb_image
        landmarks = self.marker_array
        boxes = self.bounding_boxes

        debug_input = False
        if debug_input: 
            print('DetectionNode.image_callback: received an image!')
            print('DetectionNode.image_callback: detection_box_image.shape =', identification_box_image.shape)
            #cv2.imwrite('./output_images/deep_learning_input_' + str(self.image_count).zfill(4) + '.png', detection_box_image)
        print('-----------------------yes---------------------------')
        debug_output = True
        _, output_image = self.identifier.apply_to_image(identification_box_image, boxes, (300,300), landmarks, draw_output=debug_output)        
        if debug_output: 
            print('DetectionNode.image_callback: processed image with faceID deep network!')
            print('DetectionNode.image_callback: output_image.shape =', output_image.shape)
            #cv2.imwrite('./output_images/deep_learning_output_' + str(self.image_count).zfill(4) + '.png', output_image)
            cv2.namedWindow("Image Window", 1)
            cv2.imshow("Image Window", output_image)
            cv2.waitKey(1)


    def main(self):
        rospy.init_node(self.node_name)
        name = rospy.get_name()
        rospy.loginfo("{0} started".format(name))
        
        self.rgb_topic_name = '/camera/color/image_raw' #'/camera/infra1/image_rect_raw'
        self.rgb_image_subscriber = message_filters.Subscriber(self.rgb_topic_name, Image)
        self.camera_info_subscriber = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
        self.bounding_box_subscriber = message_filters.Subscriber('/faces/bounding_boxes', BoundingBox2DArray)
        self.marker_array_subscriber = message_filters.Subscriber('/faces/marker_array', MarkerArray)

        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.rgb_image_subscriber, self.bounding_box_subscriber], 100, 0.1)
        self.synchronizer.registerCallback(self.image_callback)

        print('hello -------------------------------')
        #self.bounding_box_service = rospy.Service('face_bounding_box', AddTwoInts, handle_add_two_ints)
        
        