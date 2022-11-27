#!/usr/bin/env python3
# !coding=utf-8

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from robot_vision_pkg.msg import Robot3D_NEW
from robot_vision_pkg.srv import set_object,set_objectResponse
from detection_yolov5 import YOLO_model
import message_filters
import os
object = 0


def callback(data,depth_data):

    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")

    img,ret = yolo.Detection(cv_img,object) #Object'Light_switch'
    if len(ret) != 0:
        #print("find the boxes axis:",ret)
        depth_image = bridge.imgmsg_to_cv2(depth_data, '16UC1')
        #print("depth_image:", print("the object is ",oimg.shape[:2])

        # 定义需要得到真实三维信息的像素点（x, y)
        x = ret[0][0]
        y = ret[0][1]

        real_z = avg_depth(depth_image*0.001,ret[0])  #mm to m
        real_x = (x - ppx) / fx * real_z
        real_y = (y - ppy) / fy * real_z
        X,Y,Z = round(real_x,3),round(real_y,3),round(real_z,3)

        # #rospy.loginfo("find the object axis:", label)
        Object_3D = Robot3D_NEW()
        Object_3D.header.stamp= rospy.Time.now()
        Object_3D.X = X
        Object_3D.Y = Y
        Object_3D.Z = Z
        robot_info_pub.publish(Object_3D)
        print("Publish object 3d axis:",X,Y,Z)
        #cv2.imwrite(os.environ['HELLO_FLEET_PATH'] + '/deep_perception_models/' + '/yolov5/result/'+str(rospy.Time.now())+".jpg",img)
    else:
        #print("can't find any object")
        rospy.loginfo("YOLO Can't find any object")
        pass

    
    

def get_object_cb(set_object):
    
    global object
    object = set_object.OBJECT
    #rospy.loginfo("Ready set detect object,%s",object)
    return set_objectResponse("OK")

def avg_depth(img,box):
    #print("the depth_data value",img)
    center_x, center_y, w, h =box[0],box[1],box[2],box[3]
    ret = np.average(img[int(center_y-h/4):int(center_y+h/4), int(center_x-w/4):int(center_x +w/4)])
    return ret



def main():
    rospy.init_node('get_3D_object', anonymous=True)
    while not rospy.is_shutdown():
        #print("the object is ",object)
        if object: 
            image_sub = message_filters.Subscriber("/cam_2/color/image_raw", Image,queue_size =1,
                                                buff_size = 53428800)
            depth_sub = message_filters.Subscriber("/cam_2/aligned_depth_to_color/image_raw",
                                                Image,queue_size =1,buff_size = 53428800)
            #rospy.loginfo("get the realsense data!!!!")
            ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
            rospy.loginfo("!!!!TimeSynchronizer!!!!")

            ts.registerCallback(callback)
    #s = rospy.service("/show_robot3D",Robot3D,robot3DCallback)

            rospy.spin()
        else:
            pass

if __name__ == '__main__':
    global fx, fy, ppx, ppy,bridge,yolo,robot_info_pub
    # fx = 904.6849365234375
    # fy = 903.9230346679688
    # ppx = 635.1298828125
    # ppy = 379.99224853515625
    fx=910.279    #D435i 1280*720'Light_switch'
    fy=910.661
    ppx=668.346
    ppy=376.096
    # fx=606.853   #D435i 640*480
    # fy=607.108
    # ppx=338.897
    # ppy=250.731

    bridge = CvBridge()
    yolo = YOLO_model()
    # Object = rospy.get_param('robot_arm/OBJECT')
    robot_info_pub = rospy.Publisher('/Camera3D_object', Robot3D_NEW, queue_size=1)
    rospy.Service("/detect_object_set",set_object,get_object_cb)
    try:
        main()
    except rospy.ROSInterruptException:
        pass



# if __name__ == '__main__':
#     try:
#         #get_depth_point()
#         displayWebcam()
#     except rospy.ROSInterruptException:
#         pass
