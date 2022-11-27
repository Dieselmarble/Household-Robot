#!/usr/bin/env python3

from cv2 import split
import rospy
from map_builder import SemanticMap
import os
from semantic_map_server.srv import SemanticMapQuery, SemanticMapQueryResponse


class SenmanticMapServer():
    def __init__(self):
        self.map = SemanticMap()
        self.file_path = os.environ['HELLO_FLEET_PATH'] + '/maps/'
        self.file_name = 'semantic.xml'
        self.node_name = 'SemanticMapServer'
    
    def extact_position_argument(self, dic, name_str):
        translation = dic[name_str]['attrib']['translation']
        translation_float = str(translation).split(',')
        ret1 = []
        for item in translation_float:
            item = float(item)
            ret1.append(item)

        rotation = dic[name_str]['attrib']['rotation']
        rotation_float = str(rotation).split(',')
        ret2 = []
        for item in rotation_float:
            item = float(item)
            ret2.append(item)
        return ret1, ret2
    
    def find_object_location(self, name):
        name_str = str(name.query)
        dic = self.map.find_node(name_str)
        # {'light_switch': {'attrib': {'position': '(100, 0, 0)'}}}
        trans, rot = self.extact_position_argument(dic, name_str)
        rospy.loginfo("map server returning location %s, %s", trans, rot)
        resp = SemanticMapQueryResponse()
        resp.translation = trans
        resp.rotation = rot
        return resp

    def main(self):
        self.map.load_senmatic_map(self.file_path, self.file_name)
        rospy.init_node(self.node_name)
        s = rospy.Service('SemanticServer', SemanticMapQuery, self.find_object_location)
        rospy.spin()

if __name__ == '__main__':
    server = SenmanticMapServer()
    server.main()