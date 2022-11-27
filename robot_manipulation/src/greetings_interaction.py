#!/usr/bin/env python3

import threading
import numpy as np
import threading
import rospy
import ros_numpy
from execute_task import ExecuteTaskNode as etn

class GreetingsNode(etn):
    def __init__(self):
        etn.__init__(self)
        self.rate = 10.0
        self.joint_states = None
        self.wrist_position = None
        self.lift_position = None
        self.joint_states_lock = threading.Lock()
        etn.main(self)

    def main(self):
        name = self.recognize_face_multiple_times()      
        self.send_voice_message("你好! " + name )
        rospy.sleep(1)
        return True

if __name__ == '__main__':
    greeting = GreetingsNode()
    greeting.main()


        