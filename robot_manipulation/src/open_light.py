#!/usr/bin/env python3


import threading
import rospy
from execute_task import ExecuteTaskNode as etn
from execute_task import inverse_kinematics_base
from robot_vision_pkg.msg import Robot3D_NEW
from robot_vision_pkg.srv import set_object,set_objectResponse
from SimplePID import simplePID
import numpy as np
from geometry_msgs.msg import Twist, Point, Vector3


X_AXIS_LENGTH = rospy.get_param('robot_arm/X_AXIS_LENGTH')
Z_AXIS_LENGTH = rospy.get_param('robot_arm/Z_AXIS_LENGTH')
BASE_ANGLE_ERROR = 2 #rospy.get_param('robot_arm/ANGLE_ERROR')

class OpenLightNode(etn):
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
        #self.send_voice_message("停止寻找")

    def trigger_calibrate_base_callback(self):
        #self.send_voice_message("开始执行")
        self.open_light_switch_ready = True
        # do the task
        rospy.sleep(2)
        light_switch_pose = self.get_object_pose_euler()
        while light_switch_pose[2]<1:
            light_switch_pose = self.get_object_pose_euler()

        rospy.loginfo("the switch pose:%60s",light_switch_pose)
        move_base_forward, Z, theta =\
            inverse_kinematics_base(light_switch_pose)
        rospy.loginfo("moving base robot!! %s",move_base_forward)
        rospy.loginfo("moving base around!! %s",theta)
        if abs(theta)< 30 :
            self.set_moving_event()
            self.turn_around_with_angle(theta)
            self.move_forward_with_distance(move_base_forward)
            #self.unset_moving_event()
            rospy.sleep(2)
        else:
            rospy.loginfo("inverse_kinematics error!!!")
        rospy.loginfo("arm_pose !! %s", self.get_object_pose_euler())
        move_base_forward, Z, new_theta =\
            inverse_kinematics_base(self.get_object_pose_euler())
        print("the new_theta is ",new_theta)
        
        self.turn_around_with_angle(new_theta + BASE_ANGLE_ERROR)  #+
        rospy.loginfo("moving base around again !! %s",new_theta)
        self.unset_moving_event()
        rospy.sleep(2)
    def trigger_calibrate_arm_callback(self):   
        light_swith_arm_pose_matrix,light_swith_arm_pose = self.tranT_armTobase()
        rospy.loginfo("arm_pose !! %s", light_swith_arm_pose)
        arm_X_axis = int(light_swith_arm_pose[0]*1000)
        arm_Z_axis = int(light_swith_arm_pose[2]*1000)
        #arm_Z_axis = 1040
        rospy.loginfo("arm_ move position !! %s %s", arm_X_axis,arm_Z_axis)
        if arm_Z_axis-Z_AXIS_LENGTH <650 and arm_X_axis - X_AXIS_LENGTH >0 and arm_X_axis - X_AXIS_LENGTH < 100:
            self.arm_to_position([0,arm_Z_axis-Z_AXIS_LENGTH,0,0,0])
            self.arm_to_position([arm_X_axis-X_AXIS_LENGTH-20,0,0,0,0])
            rospy.sleep(0.5)
            self.arm_to_position([-(arm_X_axis-X_AXIS_LENGTH-20),0,0,0,0])
            self.arm_to_init_position()
            self.arm_to_position([0,0,0,0,1])                                              # close the machine
        else:
            rospy.sleep(1)                                                                 # cal the pose again
            light_swith_arm_pose_matrix,light_swith_arm_pose = self.tranT_armTobase()
            rospy.loginfo("arm_pose !! %s", light_swith_arm_pose)
            arm_X_axis = int(light_swith_arm_pose[0]*1000)
            arm_Z_axis = int(light_swith_arm_pose[2]*1000)
            #arm_Z_axis = 1040
            if  arm_X_axis - X_AXIS_LENGTH >0:           
                self.arm_to_position([0,arm_Z_axis-Z_AXIS_LENGTH,0,0,0])
                self.arm_to_position([arm_X_axis-X_AXIS_LENGTH,0,0,0,0])
                rospy.sleep(1)
                self.arm_to_position([-(arm_X_axis-X_AXIS_LENGTH),0,0,0,0])
                self.arm_to_init_position()
                self.arm_to_position([0,0,0,0,1])      
            else:
                self.send_voice_message("关灯任务失败")    
        self.finish_task = True

    def find_object_callback(self):
        rospy.loginfo("start threading find_light_switch_callback")
        rate = rospy.Rate(5)
        #self.send_voice_message("开始寻找")
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
        # rot.insert(0, 0.0)roslaunch robot_vision_pkg object_vision_detect.launch 
        rospy.loginfo("going to location %s, %s", trans, rot)
        ret = self.move_to_position(trans, rot)
        if ret:
            rospy.loginfo("goal reached from opl node")
            return True
        return False

    def PID_base_calibrate(self):
        while not self.finish_task:
            # gets called whenever we receive a new position. It will then update the motorcomand
            #dist, Z, angle = inverse_kinematics_base(self.get_object_pose_euler())  
            # call the PID controller to update it and get new speeds
            #rospy.loginfo('dist: {}, angle: {}'.format(dist, angle))
            trans, euler_d = self.get_position_in_map_xyza()
            [uncliped_ang_speed] = self.PID_controller.update([euler_d[2]])
            #[uncliped_lin_speed, uncliped_ang_speed] = self.PID_controller.update([dist, angle])
            # clip these speeds to be less then the maximal speed specified above
            angularSpeed = np.clip(uncliped_ang_speed, -0.6, 0.6)
            #linearSpeed  = np.clip(-uncliped_lin_speed, -self.move_speed, self.move_speed)
            # create the Twist message to send to the cmd_vel topic
            velocity = Twist()	
            velocity.linear = Vector3(0.,0.,0.)
            velocity.angular= Vector3(0., 0.,angularSpeed)
            rospy.loginfo('angularSpeed: {}'.format(angularSpeed))
            #rospy.loginfo('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))
            self.cmd_vel_publisher.publish(velocity)
            rospy.sleep(0.1)

    def init_stationary_PID(self):
        trans, euler_d = self.get_position_in_map_xyza()
        target_state = np.array(euler_d[2])
        self.PID_controller = simplePID(target_state, [0.03] , [0.001],  [0.0])
        #self.PID_controller = simplePID(target_state, [0.01, 0.008] , [0.01,0.001], [0.0, 0.0])

    def main(self, param):
        rospy.wait_for_service('/detect_object_set', timeout=2)
        
        self.send_voice_message("现在就去")
        trans_ori, rot_ori = self.get_position_in_map_xyza()
        # navigate to position by move_base
        
        #ret = self.move_to_predefined_location(param)
        
        # if ret:
        #     #self.move_forward_until_contact(0.4)
        #     #self.send_voice_message("到达预订位置")
        # else:
        #     self.send_voice_message("行动失败，请检查路径")
        #     return False
        # set global event in etn
        self.set_moving_event()
        rospy.loginfo("start look and find")
        self.object_client = rospy.ServiceProxy('/detect_object_set', set_object)
        response = self.object_client(param)
        # multi-threading
        rospy.sleep(0.5)
        self.look_around_thread = threading.Thread(target=self.look_around_callback)
        self.find_object_thread = threading.Thread(target=self.find_object_callback)
        self.trigger_switch_light_base_thread = threading.Thread(target=self.trigger_calibrate_base_callback)
        # start looking around thread
        # self.find_object_thread.start()
        # rospy.sleep(1)
        # self.look_around_thread.start()
        # # main thread wait for sub-thread to finish
        # self.find_object_thread.join()
        # self.look_around_thread.join()
        # self.trigger_switch_light_base_thread.start()
        # self.trigger_switch_light_base_thread.join()
        # self.trigger_switch_light_arm_thread = threading.Thread(target=self.trigger_calibrate_arm_callback)
        self.trigger_switch_light_PID_thread = threading.Thread(target=self.PID_base_calibrate)
        self.init_stationary_PID()

        self.trigger_switch_light_PID_thread.start()
        # self.trigger_switch_light_arm_thread.start()

        self.trigger_switch_light_PID_thread.join()
        # self.trigger_switch_light_arm_thread.join()

        self.send_voice_message("完成任务")
        # return to started position
        self.set_moving_event()
        rospy.sleep(0.5)
        self.move_forward_with_distance(-0.3)
        self.unset_moving_event()
        rospy.sleep(0.5)
        rospy.loginfo("returning back to inital position")
        self.send_voice_message("开始返回")
        ret = self.move_to_position(trans_ori, rot_ori)
        # return with result
        if self.finish_task:
            rospy.loginfo("finfished all task, current thread is %s", threading.current_thread().name)
            return True
        
if __name__ == '__main__':
    task = OpenLightNode()
    task.main()
