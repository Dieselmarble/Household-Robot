#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from robot_arm.srv import arm_position, arm_positionResponse
import tf

Gripping_length = rospy.get_param('robot_arm/GRIPPING_LENGTH')
Z_DISTANCE= rospy.get_param('robot_arm/Z_DISTANCE')
X_DISTANCE = rospy.get_param('robot_arm/X_DISTANCE')
GRABE_WIDE = rospy.get_param('robot_arm/GRAB_WIDTH')
Raw_X_axis= rospy.get_param('robot_arm/Raw_X_AXIS')
Raw_X_Object = rospy.get_param('robot_arm/Raw_X_OBJECT')
ANGLE_DISTANCE = rospy.get_param('robot_arm/ANGLE_DISTANCE')

class move_arm_log:
    def __init__(self):
        self.position =[]
        rospy.wait_for_service('/arm_control')
        self.arm_client = rospy.ServiceProxy('/arm_control', arm_position)
        self.x_axis_distance=0
        self.z_axis_distance=0
        self.angle_distance=0
        self.grabe_distance=0
        self.tf_listener = tf.TransformListener()

    def move_position_command(self,position):
        x_movement, z_movement, grasp_angle, grabe_wide,machine_close =position

        z_flag,x_flag,angle_flag,grabe_flag = -1,-1,-1,-1

        
        print("z_axis_distance",self.z_axis_distance,self.x_axis_distance,self.angle_distance,self.grabe_distance) 
        if (self.z_axis_distance+z_movement >  Z_DISTANCE): 
            # 量程限制            
            rospy.loginfo("The Z comulative trip control is out of range!")
        else:    
            z_flag = 0

        if (self.x_axis_distance+x_movement > X_DISTANCE):            
            rospy.loginfo("The X comulative trip control is out of range!")
        else:
            x_flag = 0

        if (abs(self.angle_distance+grasp_angle) > ANGLE_DISTANCE):
            rospy.loginfo("The angel comulative trip control is out of range!")
        else:
            angle_flag =0

        if (self.grabe_distance+ grabe_wide > GRABE_WIDE):   
            rospy.loginfo("The grab expand control is out of range")
        else:
            grabe_flag =0

        self.z_axis_distance +=  (z_flag+1)*z_movement
        self.x_axis_distance +=  (x_flag+1)*x_movement
        self.angle_distance +=  (angle_flag+1)*grasp_angle
        self.grabe_distance += (grabe_flag+1)*grabe_wide
        response = self.arm_client((x_flag+1)*x_movement,(z_flag+1)*z_movement,(angle_flag+1)*grasp_angle,(grabe_flag+1)*grabe_wide, machine_close)
        return response.result
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s" % e)

    def get_arm_position(self):
        return self.x_axis_distance, self.z_axis_distance, self.angle_distance, self.grabe_distance

    def move_init_position(self):
        position = [-self.x_axis_distance, -self.z_axis_distance,\
                    -self.angle_distance, -self.grabe_distance,0]
        self.move_position_command(position)

if __name__ =="__main__":
    rospy.init_node('arm_client')
    Com = move_arm_log()
    ret = Com.move_position_command([-50,-200,-30,20])
    rospy.sleep(2)
    ret2 = Com.move_position_command([-20,100,-40,-20])