#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import os

from pkg_resources import yield_lines

class SemanticMap:
    def __init__(self):
        self.map_name = None
        self.map_path = None
        self.tree = None
        self.root = None
    
    def create_map_file(self, map_path, map_name):
        '''
            create a xml tree with only a root node
        '''
        # create root node
        root = ET.Element('root')
        tree = ET.ElementTree(root)
        self.map_name = map_name
        self.map_path = map_path
        self.tree = tree
        self.tree.write(self.map_path+self.map_name, encoding='utf-8', xml_declaration=True)
        self.root = root

    def load_senmatic_map(self, map_path, map_name):
        '''
            load map file to RAM
        '''
        self.map_name = map_name
        self.map_path = map_path
        map = map_path+map_name
        tree = ET.ElementTree()
        tree.parse(map)
        self.tree = tree
        self.root = self.tree.getroot()


    def create_sub_node(self, child_node_name, attribute):
        '''
            add a new node to map
        '''
        root = self.root
        element = ET.SubElement(root, child_node_name)
        for idx, (key, el) in enumerate(attribute.items()):
            element.set(key, el)
        root.append(element)

    def save_file(self):
        self.tree.write(self.map_path+self.map_name, encoding='utf-8', xml_declaration=True)

    def delete_node(self, tag):
        '''
            delete an node from map
        '''
        name = self.find_node(tag)
        self.root.remove(name)

    def find_node(self, name):
        '''
            find a node from map
        '''
        #name = self.tree.find(tag)
        nodes = self.root.iter(name)
        return self.get_node(nodes)

    def get_node(self, node, start=None):
        '''
            return node as a dictionary
        '''
        dict = {}
        if not start:
            start = self.root
        for node in start:
            if list(node):
                # have a child node
                dict[node.tag] = self.get_node(node)
            else:
                dict[node.tag] = {}
                if node.text:
                    dict[node.tag]['text'] = node.text
                if node.attrib:
                    dict[node.tag]['attrib'] = node.attrib
        return dict

    def insert_node(self, tag):
        return

    def find_rec(self, node, tag, ans):
        for item in node.findall(tag):
            ans.append(item)
            self.find_rec(item, tag, ans)
        return ans

if __name__ == '__main__':
    file_path = os.environ['HELLO_FLEET_PATH'] + '/maps/'
    map = SemanticMap()
    map.create_map_file(file_path, 'semantic.xml')
    map.create_sub_node('Light_switch', {'translation': '2.227828,1.44677,0.000', 'rotation': '0.000,0.000,161.233'}) 
    map.create_sub_node('Bottle', {'translation': '3.367173,1.580501,0.000', 'rotation': '0.000,0.000,-72.63'}) 
    map.save_file()