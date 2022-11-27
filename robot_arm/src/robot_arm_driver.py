#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import math
import time
import rospy
from robot_arm.msg import blue_control
from robot_arm.srv import arm_position, arm_positionResponse
import tf


Gripping_length = rospy.get_param('robot_arm/GRIPPING_LENGTH')
Z_DISTANCE= rospy.get_param('robot_arm/Z_DISTANCE')
X_DISTANCE = rospy.get_param('robot_arm/X_DISTANCE')
GRAB_WIDTH = rospy.get_param('robot_arm/GRAB_WIDTH')
Raw_X_axis= rospy.get_param('robot_arm/Raw_X_AXIS')
Raw_X_Object = rospy.get_param('robot_arm/Raw_X_OBJECT')
ANGLE_DISTANCE = rospy.get_param('robot_arm/ANGLE_DISTANCE')
Z_AXIS_LENGTH = rospy.get_param('robot_arm/Z_AXIS_LENGTH')

# 定义回调函数，在服务端收到一个新目标时被执行。
class Arm_interface_class:
    def __init__(self):
        try:
            self.portx ="/dev/STM32"
            self.bps = 115200  #默认值，接受512字节
            self.timeX =0
            self.ser =serial.Serial(self.portx,self.bps,timeout= self.timeX)
            #打开串口，获得串口对象
            self.x_axis=0
            self.z_axis =0
            self.angle=0
            self.grabe=0
            self.x_axis_distance=0
            self.z_axis_distance =0
            self.angle_distance=0
            self.grabe_distance=0
            self.br = tf.TransformBroadcaster()
            rospy.loginfo("串口详情参数:",self.ser)
        except Exception as e:
            rospy.loginfo("Serial communication ERROR!!!")

    def turn_on_machine(self):   #继电器上电
        line = str([0, 0, 0, 0, 1, 0, 0, 0, 0, 0])
        try:
            result = self.ser.write(line.encode("gbk"))
            time.sleep(0.5)
            rospy.loginfo("Open the delay switch：")
        except Exception as e:
            rospy.loginfo("Opening machine case Error!!!")

    def turn_off_machine(self):   #继电器下电
        line = str([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        try:
            result = self.ser.write(line.encode("gbk"))
            time.sleep(0.01)
            rospy.loginfo("Close the delay switch：")
        except Exception as e:
            rospy.loginfo("Closing machine case Error!!!")

    def move_init_point(self):
        self.move_arm_position(0, 0, -self.angle_distance, -self.grabe_distance)
        self.move_arm_position(-self.x_axis_distance, -self.z_axis_distance, 0, 0)
        #记录运动的距离，并回到起止位置,
        # self.turn_off_machine()
        # rospy.sleep(1)

    def disconneted_control(self):  #终端串口通信
        self.ser.close()

    def bluetool_control(self,x_movement,z_movement,grasp_angle,grabe_wide,close_machine):  #x轴，z轴，夹爪旋转角度，展开角度
        if close_machine==0:
            self.list1 = []
            line = str([0,0,0,0,1,1,z_movement,x_movement,grasp_angle,grabe_wide]) #first 1 继电器，second 1 蓝牙
            self.z_axis =  z_movement
            self.x_axis = x_movement
            self.angle = grasp_angle
            self.grabe = grabe_wide

            if line:
                result = self.ser.write(line.encode("gbk"))
                time.sleep(0.01)
                #print("写入stm32控制指令：",line)

            if self.ser.in_waiting:
                str1=self.ser.readline(self.ser.in_waiting).decode("gbk")
                rospy.loginfo("收到的数据：")
                self.list1.append(str1)
        else:
            self.turn_off_machine()

    def linear_control(self,msg):  #bluetool control linear
        if msg.x_control !=self.x_axis or msg.z_control !=self.z_axis \
                or msg.angle_control !=self.angle or msg.grabe_control !=self.grabe or msg.close_machine !=0 :
            self.bluetool_control(msg.x_control,msg.z_control,msg.angle_control,msg.grabe_control,msg.close_machine)

    def move_arm_position(self,x_movement,z_movement,grasp_angle,grabe_wide):  #多轴行程运动（XYZ坐标模式）
        self.list1 = []
        line = str([z_movement,x_movement,grasp_angle,grabe_wide,1,0,0,0,0,0]) #1为继电器开关
        if (z_movement > Z_DISTANCE):                                          #single  axis control is out of range
            z_movement = 0
        if (x_movement > X_DISTANCE):
            x_movement = 0
        if (abs(grasp_angle) > 90):
            grasp_angle = 0
            
        self.z_axis_distance +=  z_movement
        self.x_axis_distance +=  x_movement
        self.angle_distance +=  grasp_angle
        self.grabe_distance += grabe_wide
        logic_list =[z_movement,x_movement,grasp_angle,grabe_wide]
        #print("the control axis number:",4-logic_list.count(0))
        tmp=0
        if x_movement*2 == z_movement and z_movement != 0:   #z axis speed is 2 time of x axis
            tmp = 1
        if line:
            result = self.ser.write(line.encode("gbk"))
            time.sleep(0.01)
            #print("写入stm32控制指令：",result)

        while True:

            if (self.z_axis_distance > Z_DISTANCE):  # 量程限制
                rospy.loginfo("The Z comulative trip control is out of range!")
                break
            if (self.x_axis_distance > X_DISTANCE):
                rospy.loginfo("The X comulative trip control is out of range!")
                break
            if (abs(self.angle_distance) > 90):
                rospy.loginfo("The angel comulative trip control is out of range!")
                break
            # if (abs(self.grabe_distance) > 80):
            #     rospy.loginfo("The grab expand control is out of range")
            #     break
            if self.ser.in_waiting:
                str1=self.ser.readline(self.ser.in_waiting).decode("gbk")
                if str1=="exit":
                    break
                else:
                    print("收到的数据：",str1)
                    self.list1.append(str1)
                if  len(self.list1) == 5-logic_list.count(0)-tmp or len(self.list1)== 4 :
                    break

    def auto_move_cb(self, srv):
        if srv.machine_close == 0:
            self.turn_on_machine()
            rospy.sleep(1)
            self.move_arm_position(srv.x_movement,srv.z_movement,srv.grasp_angle,srv.grabe_wide)
            self.br.sendTransform((srv.x_movement/1000, 0, srv.z_movement/1000),
                                         tf.transformations.quaternion_from_euler(0, 0, srv.grasp_angle/180 *math.pi),
                                         rospy.Time.now(),
                                         "base_link",
                                         "hand_link")
        else:
            self.turn_off_machine()
            rospy.sleep(0.5)
        
        return arm_positionResponse("OK")

    def main(self):
        rospy.init_node("robot_arm_driver",anonymous=True)
        rospy.Subscriber("robot_control", blue_control, self.linear_control, queue_size=1)
        rospy.Service("/arm_control", arm_position, self.auto_move_cb)

    def shutdown(self):
        rospy.loginfo("shutdown robot arm service")
        rospy.sleep(1)
        self.move_init_point()
        self.turn_off_machine()
        rospy.sleep(0.5)
        self.disconneted_control()
        rospy.sleep(1)

if __name__ =="__main__":
    Robot = Arm_interface_class()
    Robot.main()
    # Robot.turn_on_machine()
    rospy.on_shutdown(Robot.shutdown)
    rospy.spin()

    
