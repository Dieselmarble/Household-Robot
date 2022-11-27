#!/usr/bin/env python3


import serial
import time

class serial_port_communication:
    def __init__(self):
        try:
            self.portx ="/dev/STM32"
            self.bps = 115200
            self.timeX =0
            self.ser =serial.Serial(self.portx,self.bps,timeout= self.timeX)
            #打开串口，获得串口对象
            self.z_axis_distance = 0
            self.x_axis_distance = 0
            self.angle_distance = 0
            self.grabe_distance =0
            self.angle = 0
            print("串口详情参数:",self.ser)
        except Exception as e:
            print("Serial communication ERROR!!!",e)

    def turn_on_machine(self):   #继电器上电
        line = str([0, 0, 0, 0, 1, 0, 0, 0, 0, 0])
        try:
            result = self.ser.write(line.encode("gbk"))
            time.sleep(1)
            print("Open the delay switch：")
        except Exception as e:
            print("Opening machine case Error!!!")

    def turn_off_machine(self):   #继电器下电
        line = str([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        try:
            result = self.ser.write(line.encode("gbk"))
            time.sleep(0.01)
            print("Close the delay switch：")
        except Exception as e:
            print("Closing machine case Error!!!")

    def normal_contorl(self,x_movement,z_movement,grasp_angle,grabe_wide):  #x轴，z轴，夹爪展开角度，速度
        self.list1 = []
        line = str([z_movement, x_movement, grasp_angle, grabe_wide, 1, 0, 0, 0, 0, 0])  # 1为继电器开关

        logic_list = [z_movement, x_movement, grasp_angle, grabe_wide]
        # print("the control axis number:",4-logic_list.count(0))
        tmp =0
        if x_movement*2 ==z_movement and x_movement !=0:
            tmp =1
        if line:
            result = self.ser.write(line.encode("gbk"))
            time.sleep(0.01)
            # print("写入stm32控制指令：",result)

        while True:
            if (self.z_axis_distance > 1150):  # 量程限制
                #rospy.loginfo("The Z comulative trip control is out of range!")
                break
            if (self.x_axis_distance > 300):
                #rospy.loginfo("The X comulative trip control is out of range!")
                break
            # if (abs(self.angle_distance) > 90):
            #     #rospy.loginfo("The angel comulative trip control is out of range!")
            #     break
            if (abs(self.grabe_distance) > 60):
                #rospy.loginfo("The grab expand control is out of range")
                break
            if self.ser.in_waiting:
                str1 = self.ser.readline(self.ser.in_waiting).decode("gbk")
                if str1 == "exit":
                    break
                else:
                    print("收到的数据：", str1)
                    self.list1.append(str1)
                if len(self.list1) == 5 - logic_list.count(0)-tmp :#or len(self.list1) == 4
                    break



    def go_init_point(self):
        self.normal_contorl(0,0,-self.angle_distance,0)
        self.normal_contorl(-self.x_axis_distance, 0,0, 0)
        self.normal_contorl(0, -self.z_axis_distance, 0, 0)
        #记录运动的距离，并回到起止位置,

        self.ser.close()     #关闭串口


import math

def inverse_kinematics(X,Y,Z):
    theta = 180* (math.asin(abs(Y)/140))/math.pi
    z = Z+30
    x = abs(X-140*math.cos(math.pi*theta/180)-375)
    print("The inverse_kinemateics resule Z axis ",z,"x axis",x,"angle",theta)
    return z,x,theta


def grasp_control_log(z,x,theta):#object_point,place_point,grasp_width

    com = serial_port_communication()
    if (z>1150):
        print("The Z axis control is out of range!")
        return
    if(x >350):
        print("The X axis control is out of range!")
        return-810
    if (abs(theta)>90):
        print("The angel control is out of range!")
        return
    print("z axis control axis!!")
    com.normal_contorl(0,z,theta,50)
    print(" Angle control axis!!!")
    # com.normal_contorl(0,0, theta, 0)
    print("grabe control logic!!")
    # com.normal_contorl(0,0,0,50)
    com.normal_contorl(x,0,0,0)
    com.normal_contorl(0,0,0,-50)
    com.normal_contorl(0, 50, 0, 0)
    com.go_init_point()

import time

def open_switch_log(z,x,theta):#object_point,place_point,grasp_width

    com = serial_port_communication()
    if (z>1150):
        print("The Z axis control is out of range!")
        return
    if(x >350):
        print("The X axis control is out of range!")
        return
    if (abs(theta)>90):
        print("The angel control is out of range!")
        return
    print("z axis control axis!!")
    com.normal_contorl(x-30,z,0,0)
    print(" Angle control axis!!!")
    # com.normal_contorl(0,0, theta, 0)
    print("move the x axis!!")
    com.normal_contorl(30,0,0,0)
    #com.normal_contorl(x,0,0,0)

    time.sleep(1)  #休息5s
    com.normal_contorl(-40,0,0,0)
    # com.normal_contorl(-175,-980, 0, 0)
    time.sleep(2)
    com.go_init_point()
    # com.normal_contorl(-195,-970,0,0)

def test_log(x,z,theta,wide):#object_point,place_point,grasp_width

    com = serial_port_communication()
    com.turn_on_machine()
    time.sleep(1)

    com.normal_contorl(x,z,theta,wide)

    com.turn_off_machine()
    time.sleep(2)

if __name__ =="__main__":
    try:
        # z,x,theta = inverse_kinematics(770,0,990)
        # open_switch_log(z,x,theta)
        test_log(0,10,0,0)
    except Exception as e:
        print("ERROR case:",e)