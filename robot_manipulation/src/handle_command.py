#!/usr/bin/env python3

from __future__ import print_function
from xf_mic_asr_offline.srv import VoiceCommand, VoiceCommandResponse
import argparse as ap
from std_msgs.msg import String, Bool
import rospy
from handover_object import HandoverObjectNode as hdo
from greetings_interaction import GreetingsNode as gn
from open_light import OpenLightNode as oln
from grasp_object import GraspObjectNode as gon

class HandleCommandNode():
    def __init__(self):
        self.node_name = 'HandleCommandNode'
        self.rate = 2.0
        self.task_id = None
        rospy.init_node(self.node_name)
        self.node_name = rospy.get_name()
        rospy.loginfo("{0} started".format(self.node_name))

    def handle_voice_command(self, req):
        #voice_command_map = {"关一下灯": 0, '开一下灯': 1, '拿一下水': 2, '我是谁':3}
        if (req.task_id == 0):
            result = self.handle_switch_light("Light_switch")
        elif (req.task_id == 1):
            result= self.handle_switch_light("Light_switch")
        elif (req.task_id == 2):
            result = self.handle_grasp_bottle("Bottle")
        elif (req.task_id == 3):
            result = self.handle_greetings("who are you")
        # return result to Cpp voice interface
        if result == True:
            ans = VoiceCommandResponse()
            ans.successful = True
            ans.redo_required = False
            #ans.message = "任务完成"
        else:
            ans = VoiceCommandResponse()
            ans.successful = False
            ans.redo_required = True
            ans.message = "任务失败"
        return ans

    def handle_switch_light(self, param):
        self.switch_light_node = oln()
        result = self.switch_light_node.main(param) 
        return result

    def handle_grasp_bottle(self,parm):
        self.grasp_bottle_node =gon()
        result = self.grasp_bottle_node.main(parm)
        return result 

    def handle_greetings(self, param):
        self.greeting_node = gn()
        result = self.greeting_node.main() 
        return result

    def main(self):
        # self.handle_switch_light("Light_switch")
        # rosservice for receivng command from voice control and give feedback
        self.command_handler_service = rospy.Service('/voice_command', VoiceCommand, self.handle_voice_command)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        parser = ap.ArgumentParser(description='Handle and analyse vocal commands.')
        args, unknown = parser.parse_known_args()
        node = HandleCommandNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
