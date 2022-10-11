#!/usr/bin/env python3
#! -*- coding: utf-8 -*-

import numpy as np
import math
import threading
import yaml
import sys
import datetime
from subprocess import Popen, PIPE

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion


np.set_printoptions(linewidth=200)

class CleaningPathEditor:
    def __init__(self) -> None:
        rospy.init_node("cleaning_path_editor")

        if rospy.has_param("~MAP_PATH"):
            self.map_file = rospy.get_param("~MAP_PATH")
        else:
            rospy.logwarn("No path file specified. Shutting down...")
            rospy.signal_shutdown("No path file specified")
            sys.exit(1)
        self.backup_file = rospy.get_param("~BACKUP_FILE", self.map_file + ".bak")
        self.pose_topic = rospy.get_param("~POSE_TOPIC", "/move_base_simple/goal") # rviz 2DNavGoal

        self.has_saved = False
        self.update_pose = True
        self.pose = PoseStamped()
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)

        self.lock = threading.Lock()
        self.map = self.load_map_from_yaml()
        self.take_backup()
        rospy.loginfo("=== map editor ===")

    def take_backup(self) -> None:
        try:
            with open(self.backup_file, "a") as f:
                f.write("timestamp: {}\n".format(datetime.datetime.now()))
                yaml.safe_dump(self.map, f, sort_keys=False)
                self.has_saved = True
        except Exception as e:
            print(str(e))
            rospy.signal_shutdown("Failed to take backup")
            sys.exit(2)

    def load_map_from_yaml(self) -> dict:
        try:
            with open(self.map_file, "r") as f:
                _map = yaml.safe_load(f)
                for edge in _map["EDGE"]:
                    edge["node_id"] = [int(edge["node_id"][0]), int(edge["node_id"][1])]
                return _map

        except Exception as e:
            print(str(e))
            rospy.signal_shutdown("Failed to load map from file")
            sys.exit(2)

    def has_navigation_manager_node(self) -> bool:
        node_list_cmd = "rosnode list"
        ros_node_process = Popen(node_list_cmd.split(), stdout=PIPE)
        ros_node_process.wait()
        nodetuple = ros_node_process.communicate()
        nodelist = nodetuple[0]
        nodelist = nodelist.decode()
        nodelist = nodelist.split()
        for nd in nodelist:
            if not nd.find('node_edge_map_manager') == -1:
                return True
        return False

    def node_id_existing(self, _id: int) -> bool:
        for node in self.map["NODE"]:
            if node["id"] == _id:
                return True
        return False

    def ask_continue(self, cmd: str) -> bool:
        yn = input("continue {}? [Y/n]: ".format(cmd))
        if yn == "Y" or yn == "y" or yn == "":
            return True
        else:
            return False

    def pose_callback(self, msg: PoseStamped) -> None:
        if not self.update_pose:
            self.pose = msg
            self.update_pose = True

    def get_direction_from_quaternion(self, q: Quaternion) -> float:
        _roll, _pitch, _yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return round(_yaw, 4)
        # _yaw = _yaw - self.map["MAP_DIRECTION"]
        # while _yaw < -math.pi:
        #     _yaw += 2 * math.pi
        # while _yaw > math.pi:
        #     _yaw -= 2 * math.pi
        #
        # if _yaw < -math.pi * 3/4.0:
        #     return "down"
        # elif _yaw < -math.pi * 1/4.0:
        #     return "right"
        # elif _yaw < math.pi * 1/4.0:
        #     return "up"
        # elif _yaw < math.pi * 3/4.0:
        #     return "left"
        # else:
        #     return "down"

    def get_pose_from_rviz(self) -> tuple:
        self.update_pose = False
        print("Please use tool '2DNavGoal' in rviz to get the pose")
        _loop = True
        counter = 0
        while _loop:
            if self.update_pose:
                _x = round(self.pose.pose.position.x, 3)
                _y = round(self.pose.pose.position.y, 3)
                _direction = self.get_direction_from_quaternion(self.pose.pose.orientation)
                return _direction, _x, _y
            else:
                if counter == 0:
                    rospy.loginfo("Waiting for pose...")
                counter += 1
                rospy.sleep(1.0)
                if counter > 30:
                    _loop = self.ask_continue("wait for pose")
        return None, None, None

    def get_index_from_id(self, id: int) -> int:
        index = 0
        for node in self.map["NODE"]:
            if node["id"] == id:
                return index
            index += 1
        return -1

    def get_distance_from_point_to_line(self, _x0: float, _y0: float, _x1: float, _y1: float, _x2: float, _y2: float) -> float:
        dx21 = _x2 - _x1
        dy21 = _y2 - _y1
        dx10 = _x1 - _x0
        dy10 = _y1 - _y0
        dx20 = _x2 - _x0
        dy20 = _y2 - _y0
        t = -( dx21*dx10 + dy21*dy10)

        if t <= 0:
            return math.sqrt(dx10*dx10 + dy10*dy10)
        elif t >= dx21*dx21 + dy21*dy21:
            return math.sqrt(dx20*dx20 + dy20*dy20)
        else:
            return math.sqrt((dx21*dy10 - dy21*dx10)**2 / (dx21*dx21 + dy21*dy21))

    def get_edge_id_from_pose(self, _x0: float, _y0: float) -> int:
        nearest_edge_id = -1
        nearest_edge_distance = float(100)
        _count = 0
        for edge in self.map["EDGE"]:
            _x1 = self.map["NODE"][self.get_index_from_id(edge["node_id"][0])]["point"]["x"]
            _y1 = self.map["NODE"][self.get_index_from_id(edge["node_id"][0])]["point"]["y"]
            _x2 = self.map["NODE"][self.get_index_from_id(edge["node_id"][1])]["point"]["x"]
            _y2 = self.map["NODE"][self.get_index_from_id(edge["node_id"][1])]["point"]["y"]
            _distance = self.get_distance_from_point_to_line(_x0, _y0, _x1, _y1, _x2, _y2)
            if _distance < nearest_edge_distance:
                nearest_edge_distance = _distance
                nearest_edge_id = _count

            _count += 1

        return nearest_edge_id

    def set_node(self, _id: int, _type: str, _label: str, _x: float, _y: float) -> dict:
        _point = {"x": _x, "y": _y}
        _node = {"id": _id, "type": _type, "point": _point, "label": _label}
        return _node

    def ask_edge_settings(self) -> tuple:
        _start_node_id = int(input("Please enter the new start_node_id: "))
        if not self.node_id_existing(_start_node_id):
            print("Entered id is not exists. Please enter another.")
            return None
        _end_node_id = int(input("Please enter the new end_node_id: "))
        if not self.node_id_existing(_end_node_id):
            print("Entered id is not exists. Please enter another.")
            return None
        return _start_node_id, _end_node_id

    def set_edge(self, _start_node_id: int, _end_node_id: int) -> dict:
        _node_id = [_start_node_id, _end_node_id]
        _edge = {"node_id": _node_id}
        # _edge = {"command": _command, "start_node_id": _start_node_id, "end_node_id": _end_node_id, "skippable": _skippable}
        return _edge


    def add_node(self) -> None:
        print("=== Add Node ===")
        _loop = True
        while _loop:
            _id = int(input("Please enter the new node id: "))
            if self.node_id_existing(_id):
                print("Entered id is already exists. Please enter another.")
            else:
                _type = input("Please enter the new node type(Default is intersection). : " )
                _label = input("Please enter the new node label(If you don't wanna label, just press enter).: " )
                _direction, _x, _y = self.get_pose_from_rviz()
                if _type == '':
                   _type = "intersection"
                if _label == None or _label == '':
                    _label = ""
                if _x == None:
                    print("Cannot get pose.")
                    continue
                _node = self.set_node(_id, _type, _label, _x, _y)
                print("New node :\n" + str(_node))
                self.map["NODE"].append(_node)
                self.has_saved = False

            _loop = self.ask_continue("adding nodes")

    def move_node(self) -> None:
        print("=== Move Node ===")
        _loop = True
        while _loop:
            _id = int(input("Please enter the node id: "))
            if not self.node_id_existing(_id):
                print("Entered id is not exists. Please enter another.")
            else:
                _direction, _x, _y = self.get_pose_from_rviz()
                _type = input("Please enter the new node type(If you don't wanna change, just press enter). : " )
                _label = input("Please enter the new node label(If you don't wanna change, just press enter).: " )
                for node in self.map["NODE"]:
                    if node["id"] == _id:
                        if _type == '' or _type == None:
                            node["type"] = node["type"]
                        else:
                            node["type"] = _type
                        if _label == '' or _label == None:
                            node["label"] = node["label"]
                        else:
                            node["label"] = _label
                        node["point"]["x"] = _x
                        node["point"]["y"] = _y
                        print("Node {} has been moved to:\n".format(_id) + str(node))
                        self.has_saved = False
                        break
            _loop = self.ask_continue("moving nodes")

    def delete_node(self) -> None:
        print("=== Delete Node ===")
        _loop = True
        while _loop:
            _id = int(input("Please enter the node id: "))
            if not self.node_id_existing(_id):
                print("Entered id is not exists. Please enter another.")
            else:
                _index = self.get_index_from_id(_id)
                print("Node {} has been deleted:".format(_id) + str(self.map["NODE"].pop(_index)))
                self.has_saved = False

            _loop = self.ask_continue("deleting nodes")

    def add_edge(self) -> None:
        print("=== Add Edge ===")
        _loop = True
        while _loop:
            self.visualize_edge()
            _id = int(input("Where do want to instert the new edge? To insert it last, enter -1: "))
            if (_id >= len(self.map["EDGE"])) or (_id < -1):
                print("Entered id is not valid. Please enter another.")
            else:
                _start_node_id, _end_node_id = self.ask_edge_settings()
                if _start_node_id == None:
                    continue
                _edge = self.set_edge(_start_node_id, _end_node_id)
                if _id == -1:
                    self.map["EDGE"].append(_edge)
                else:
                    self.map["EDGE"].insert(_id, _edge)
                print("New edge :\n" + str(_edge))
                self.has_saved = False

            _loop = self.ask_continue("adding edges")

    def move_edge(self) -> None:
        print("=== Move Edge ===")
        _loop = True
        while _loop:
            print("Which edge do you want to move?")
            _direction, _x, _y = self.get_pose_from_rviz()
            _id = self.get_edge_id_from_pose(_x, _y)
            if _id == -1:
                print("Cannot get edge id")
                continue
            else:
                print("Edge {} is selected".format(_id))
                print("Current settings are: {}".format(self.map["EDGE"][_id]))
                _start_node_id, _end_node_id = self.ask_edge_settings()
                if _start_node_id == None:
                    continue
                _edge = self.set_edge(_start_node_id, _end_node_id)
                self.map["EDGE"][_id] = _edge
                print("Edge {} has been moved to: ".format(_id) + str(_edge))
                self.has_saved = False

            _loop = self.ask_continue("moving edges")

    def delete_edge(self) -> None:
        print("=== Delete Edge ===")
        _loop = True
        while _loop:
            _direction, _x, _y = self.get_pose_from_rviz()
            _id = self.get_edge_id_from_pose(_x, _y)
            if _id == -1:
                print("Cannot find the edge")
            else:
                print("Edge {} has been deleted:\n".format(_id) + str(self.map["EDGE"].pop(_id)))
                self.has_saved = False

            _loop = self.ask_continue("deleting edges")

    def overwrite_map_file(self) -> None:
        try:
            with open(self.map_file, "w") as f:
                yaml.safe_dump(self.map, f, sort_keys=False)
                # yaml.safe_dump(self.map, f, sort_keys=False, default_style=False)
                print("saved and updated map file")
                self.has_saved = True
        except Exception as e:
            print(str(e))
            rospy.logwarn("Failed to overwrite map file")

    def visualize_node(self) -> None:
        print("Nodes:")
        for node in self.map["NODE"]:
            print(str(node))

    def visualize_edge(self) -> None:
        _count = 0
        print("Edges:")
        for edge in self.map["EDGE"]:
            print("No: {}: ".format(_count) + str(edge))
            _count += 1

    def visualize_map(self) -> None:
        print("=== Visualize Map ===")
        self.visualize_node()
        self.visualize_edge()

    def reload_map(self) -> None:
        print("=== Reload Math ===")
        if self.has_saved:
            self.map = self.load_map_from_yaml()
            print("reloaded map")
            self.visualize_map()
        else:
            yn = input("You have not saved the path. Are you sure to reload the map? (y/n): ")
            if yn == "y" or yn == "Y":
                self.map = self.load_map_from_yaml()
                print("reloaded map \n")
                self.visualize_map()

    def print_help(self) -> None:
        print("=== Help ===")
        print("an: add node")
        print("dn: delete node")
        print("mn: move node")
        print("ae: add edge")
        print("de: delete edge")
        print("me: move edge")
        print("s: save the map")
        print("v: visualize the path")
        print("r: reload the map")
        print("q: quit\n")

    def process(self) -> None:
        rate = rospy.Rate(1)
        self.visualize_map()
        while not rospy.is_shutdown():
            if  self.has_navigation_manager_node():
                rospy.loginfo_once("Found navigation manager")
                cmd = input("Please enter the command.(h for help): ")
                if cmd == "an":
                    self.add_node()
                elif cmd == "dn":
                    self.delete_node()
                elif cmd == "mn":
                    self.move_node()
                elif cmd == "ae":
                    self.add_edge()
                elif cmd == "de":
                    self.delete_edge()
                elif cmd == "me":
                    self.move_edge()
                elif cmd == "s":
                    self.overwrite_map_file()
                    self.reload_map()
                elif cmd == "v":
                    self.visualize_map()
                elif cmd == "r":
                    self.reload_map()
                elif cmd == "h":
                    self.print_help()
                elif cmd == "q":
                    if self.has_saved:
                        rospy.signal_shutdown("quit")
                        sys.exit(0)
                    else:
                        yn = input("You have not saved the path. Are you sure to quit? [y/n]")
                        if yn == "y" or yn == "Y":
                            rospy.signal_shutdown("quit")
                            sys.exit(0)
            else:
                rospy.logwarn("Waiting for node_edge_map_manager node to start...")
                rate.sleep()
                continue

        rate.sleep()

if __name__ == "__main__":
    editor = CleaningPathEditor()
    editor.process()
