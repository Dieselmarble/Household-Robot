#!/usr/bin/env python3


import threading
import rospy

from execute_task import ExecuteTaskNode as etn
from execute_task import inverse_kinematics_base,inverse_kinematics_grasp
from robot_vision_pkg.msg import Robot3D_NEW
from robot_vision_pkg.srv import set_object,set_objectResponse

X_AXIS_LENGTH = rospy.get_param('robot_arm/X_AXIS_LENGTH')
Z_AXIS_LENGTH = rospy.get_param('robot_arm/Z_AXIS_LENGTH')
BASE_ANGLE_ERROR = rospy.get_param('robot_arm/ANGLE_ERROR')
Gripping_length = rospy.get_param('robot_arm/GRIPPING_LENGTH')

class GraspObjectNode(etn):
    def __init__(self):
        etn.__init__(self)
        etn.main(self)
        self.look_around = True
        self.finish_task = False
        self.node_name = "OpenLight"
        rospy.loginfo("Init node with name %s", self.node_name)    
    
    '''
        look around to find light switch
    '''
    def look_around_callback(self):
        rospy.loginfo("start threading look_around_callback")
        # see if moving is allowed
        while self.look_around:
            rospy.loginfo("moving_event %s", self.get_moving_event_status())
            self.turn_around_with_angle(30)
            rospy.sleep(2)
        rospy.loginfo("look around callback stopped")
        self.send_voice_message("停止寻找")


    def grasp_bottle_callback(self):
        self.send_voice_message("开始抓取")
        # do the task
        rospy.sleep(2)
        object_pose = self.get_object_pose_euler()
        # while light_switch_pose[2]<1:
        #     light_switch_pose = self.get_object_pose_euler()

        rospy.loginfo("the bottle_object pose:%s",object_pose)
        move_base_forward, Z, theta =\
            inverse_kinematics_base(object_pose)
        rospy.loginfo("moving base robot!! %s",move_base_forward)
        rospy.loginfo("moving base around!! %s",theta)
        if move_base_forward>0.05:
            self.set_moving_event()
            self.turn_around_with_angle(theta)
            self.move_forward_with_distance(move_base_forward-0.08) #预留8cm的手臂离目标距离
        #
        rospy.sleep(3)
        # else:
        #     rospy.loginfo("inverse_kinematics error!!!")
        rospy.loginfo("arm_pose after move!! %s", self.get_object_pose_euler())

        move_base_forward, Z, theta =\
            inverse_kinematics_base(self.get_object_pose_euler())
        theta += BASE_ANGLE_ERROR
        self.set_moving_event()    
        self.turn_around_with_angle(theta) 
        rospy.loginfo("moving base around  Again!! %s",theta)
        #self.send_voice_message("完成底盘运动")
        #self.unset_moving_event()
        rospy.sleep(3)
        rospy.loginfo("the bottle_object pose:%s",self.get_object_pose_euler())
        _,object_arm_pose = self.tranT_armTobase()
        rospy.loginfo("arm_pose after fine-tune !! %s", object_arm_pose)
        # if int(object_arm_pose[1] *1000) > Gripping_length:
        #     rospy.loginfo("!!!!move base again!!!")
        #     self.set_moving_event()
        #     move_base_forward, Z, theta =\
        #         inverse_kinematics_base(self.get_object_pose_euler())
        #     theta += BASE_ANGLE_ERROR    
        #     self.turn_around_with_angle(theta ) 
            #self.unset_moving_event()
        _,object_arm_pose = self.tranT_armTobase()
        arm_z, arm_x, arm_theta =inverse_kinematics_grasp(object_arm_pose)
        
        """when Y >Gripping length ,turn around again"""
        # if arm_z == 0:

        #     self.turn_around_with_angle(10)   
        #     object_pose = self.get_object_pose_euler()  
        #     move_base_forward, Z, theta =inverse_kinematics_base(object_pose)
        #     theta += BASE_ANGLE_ERROR    
        #     self.turn_around_with_angle(theta ) 
        #     rospy.loginfo("moving base around  Again!! %s",theta)
        #     #self.send_voice_message("完成底盘运动")
      
        #     rospy.sleep(2)
        #     rospy.loginfo("the bottle_object pose:%s",self.get_object_pose_euler())
        #     _,object_arm_pose = self.tranT_armTobase()
        #     arm_z, arm_x, arm_theta =inverse_kinematics_grasp(object_arm_pose)

        # # rospy.loginfo("arm_pose !! %s", self.get_object_pose_euler())
        # rospy.loginfo("move arm movement!! %s,  %s, %s", arm_z, arm_x,arm_theta)
        '''robot arm move'''
        #arm_z = 20
        if arm_x >0 and arm_x<300 and arm_z<400:
            self.arm_to_position([0,arm_z,0,0,0])
            #if arm_X_axiz-X_AXIS_LENGTH >0:
            self.arm_to_position([arm_x-20,0,0,65,0])
            #rospy.sleep(0.5)
            self.arm_to_position([0,0,0,-60,0])
            #rospy.sleep(0.5)
            self.arm_to_position([-arm_x+20, 50, 0, 0,0])
        
            rospy.sleep(1)
            self.move_forward_with_distance(-0.2)
            # self.arm_to_position([arm_x,-30, 0,0 ])
            # rospy.sleep(3)
            self.arm_to_position([0, -arm_z-50, 0, 0,0])
            self.arm_to_position([0, 0, 0, 0, 1])
        self.unset_moving_event()

        self.grasp_bottle_ready = True
        # finish the task
        self.finish_task = True

    def find_object_callback(self):
        rospy.loginfo("start threading find_light_switch_callback")
        rate = rospy.Rate(5)
        self.send_voice_message("开始寻找")
        while True:     #not rospy.is_shutdown
            object_pose = self.get_object_pose_euler()
            rospy.loginfo("received: %s", object_pose)
            if not object_pose is None:
                # stop looking around
                self.unset_moving_event()
                self.stop_the_robot()
                self.look_around = False
                # release event
                rospy.loginfo("find target, stop moving")
                self.send_voice_message("发现目标")
                break
            rate.sleep()
        rospy.loginfo("find_light_switch_callback stopped")

    def move_to_predefined_location(self, param):
        trans, rot = self.get_object_location_from_map_server(param)
        # trans.insert(0, 3.199)
        # trans.insert(1, 1.739)
        # trans.insert(2, 0.0)
        # rot.insert(0, 0.0)
        # rot.insert(1, 0.0)
        # rot.insert(2, 138.493)
        rospy.loginfo("going to location %s, %s", trans, rot)
        ret = self.move_to_position(trans, rot)
        if ret:
            rospy.loginfo("goal reached from opl node")
            return True
        return False

    def main(self, param):
        rospy.wait_for_service('/detect_object_set', timeout=3)

      
        # self.send_voice_message("现在就去")
        trans_ori, rot_ori = self.get_position_in_map_xyza()

        # navigate to position by move_base
        ret = self.move_to_predefined_location(param)
        if ret:
            #self.move_forward_until_contact(0.5)
            self.send_voice_message("到达预订位置")
        else:
            self.send_voice_message("行动失败，请检查路径")
            return False
        # set global event in etn
        self.set_moving_event()
        rospy.loginfo("start look and find")
        # multi-threading
        self.object_client = rospy.ServiceProxy('/detect_object_set', set_object)
        response = self.object_client(param)
        self.look_around_thread = threading.Thread(target=self.look_around_callback)
        self.find_object_thread = threading.Thread(target=self.find_object_callback)
        #self.trigger_switch_light_thread = threading.Thread(target=self.trigger_switch_light_callback)
        self.grasp_bottle_thread = threading.Thread(target=self.grasp_bottle_callback)
        # start looking around thread
        self.find_object_thread.start()
        rospy.sleep(2)
        self.look_around_thread.start()
        # main thread wait for sub-thread to finish
        self.find_object_thread.join()
        self.look_around_thread.join()
        self.grasp_bottle_thread.start()
        self.grasp_bottle_thread.join()
        # self.send_voice_message("完成抓取")
        # # return to started position
        # self.set_moving_event()
        # rospy.sleep(2)
        # self.move_forward_with_distance(-0.3)
        # self.unset_moving_event()
        rospy.sleep(1)
        rospy.loginfo("returning back to inital position")
        self.send_voice_message("开始返回")
        ret = self.move_to_position(trans_ori, rot_ori)
        # return with result
        if self.finish_task:
            rospy.loginfo("finfished all task, current thread is %s", threading.current_thread().name)
            return True
        
    

