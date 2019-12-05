#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
import tf
import sys
import math
import itertools
import numpy as np

import pdb

class RvizMarker():
    id = 0
    config = {
        "red_object" : {"color":{"r":1.0,"g":0.0,"b":0.0,"a":1},"scale":{"x":0.2,"y":0.2,"z":0.3},"type":1},
        "vector"    : {"color":{"r":0.0,"g":0.0,"b":1.0, "a":1},"scale":{"x":0.07,"y":0.1,"z":0},"type":0},
    }

    def __init__(self, name, item):
        self.pub = rospy.Publisher(name, Marker, queue_size = 10)
        self.marker_data = Marker()
        self.marker_data.header.frame_id = "world"
        self.marker_data.header.stamp = rospy.Time.now()
        self.marker_data.color.r = RvizMarker.config[item]["color"]["r"]
        self.marker_data.color.g = RvizMarker.config[item]["color"]["g"]
        self.marker_data.color.b = RvizMarker.config[item]["color"]["b"]
        self.marker_data.color.a = RvizMarker.config[item]["color"]["a"]
        self.marker_data.scale.x = RvizMarker.config[item]["scale"]["x"]
        self.marker_data.scale.y = RvizMarker.config[item]["scale"]["y"]
        self.marker_data.scale.z = RvizMarker.config[item]["scale"]["z"]
        self.marker_data.ns = "basic_shapes"
        self.marker_data.id = RvizMarker.id
        RvizMarker.id += 1
        self.marker_data.type = RvizMarker.config[item]["type"]

    def display(self, position, rpy, text = None, lifetime = None):
        self.marker_data.id = RvizMarker.id
        self.marker_data.action = Marker.ADD

        if self.marker_data.type == 0: # if arrow
            start, end = Point(), Point()
            start.x = position[0]
            start.y = position[1]
            end.x = rpy[0]
            end.y = rpy[1]
            try:
                start.z = position[2]
                end.z = rpy[2]
            except:
                start.z = 0
                end.z = 0
            self.marker_data.points = []
            self.marker_data.points.append(start)
            self.marker_data.points.append(end)
        else:
            self.marker_data.pose.position.x = position[0]
            self.marker_data.pose.position.y = position[1]
            self.marker_data.pose.position.z = position[2]

            orientation = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
            self.marker_data.pose.orientation.x = orientation[0]
            self.marker_data.pose.orientation.y = orientation[1]
            self.marker_data.pose.orientation.z = orientation[2]
            self.marker_data.pose.orientation.w = orientation[3]

        self.marker_data.lifetime = rospy.Duration() if lifetime == None else lifetime

        self.pub.publish(self.marker_data)

class GetObjectData():
    def __init__(self, name=""):
        if name != "":
            rospy.init_node(name, anonymous=True)

        # rospram
        detected_line_name = rospy.get_param('~detected_line_name', '/line_detector/debug/line_marker')
        self.camera_frame_name = rospy.get_param('~camera_frame_name', 'rs_d435_color_optical_frame')

        # subscriber
        self.get_object_data_sub = rospy.Subscriber(detected_line_name, Marker, self.getobjectdataCallback)

        # tf
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

        # marker
        self.target_object = RvizMarker("target_red_object", "red_object")
        self.target_angle = RvizMarker("target_red_object_posture", "vector")
        time.sleep(0.5)

    def line_intersection(self, line1, line2):
        x_diff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        y_diff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        line1_vec = np.array([-x_diff[0], -y_diff[0]])
        line2_vec = np.array([-x_diff[1], -y_diff[1]])
        line1_norm = np.linalg.norm(line1_vec)
        line2_norm = np.linalg.norm(line2_vec)
        cos = np.dot(line1_vec,line2_vec) / (line1_norm * line2_norm)

        if abs(cos) > 0.17:
            rospy.logwarn("lines do not intersect at right angle")
            return None

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(x_diff, y_diff)
        d = (det(*line1), det(*line2))
        x = det(d, x_diff) / div
        y = det(d, y_diff) / div

        a = np.array([x,y])
        forward = [np.array(line1[0][0:2]), np.array(line2[0][0:2])]
        back = [np.array(line1[1][0:2]), np.array(line2[1][0:2])]
        for i in range(2):
            if np.linalg.norm(a-forward[i]) > np.linalg.norm(a-back[i]):
                rospy.logwarn("lines intersect in the back")
                return None
            elif np.linalg.norm(a-forward[i]) > 0.15:
                rospy.logwarn("wrong lines intersect")
                return None
        return x, y

    def getobjectdataCallback(self, msg):
        if (len(msg.points) < 3):
            rospy.logwarn("number of detected points: %d",len(msg.points))
            return

        # 1. convert two points into one line
        point_coords_camera = []
        lines_coords_camera = []

        point_st = PoseStamped()
        point_st.header.frame_id = self.camera_frame_name
        orientation = Quaternion()
        point_st.pose.orientation = orientation
        point_coords_world = []
        lines_coords_world = []

        for i in range(len(msg.points)):
            point_coords_camera.append([msg.points[i].x, msg.points[i].y, msg.points[i].z])
            if (i % 2) == 1:
                if point_coords_camera[i-1][0] > point_coords_camera[i][0]:
                    point_coords_camera[i-1], point_coords_camera[i] = point_coords_camera[i], point_coords_camera[i-1]
                lines_coords_camera.append([point_coords_camera[i-1], point_coords_camera[i]])

                # 1-1. calculate points coords in worldframe
                for k in range(2):
                    point_st.pose.position.x = point_coords_camera[k+i-1][0]
                    point_st.pose.position.y = point_coords_camera[k+i-1][1]
                    point_st.pose.position.z = point_coords_camera[k+i-1][2]

                    trans = self.tfBuffer.transform(point_st, 'world')
                    point_coords_world.append([trans.pose.position.x,trans.pose.position.y, trans.pose.position.z])
                lines_coords_world.append([point_coords_world[i-1], point_coords_world[i]])

        # 2. chose the nearest point and compare the next to points
        while(1):
            #pdb.set_trace()
            try:
                index = lines_coords_camera.index(min(lines_coords_camera, key=lambda arg: arg[0][0]))
                rospy.loginfo(index)
                lines_coords_camera.pop(index)
                line1 = lines_coords_world.pop(index)
            except:
                rospy.logwarn("no suitable lines for object")
                return

            target_object_vertex = np.array(None)
            line2 = []
            for i in range(len(lines_coords_world)):
                if i == index:
                    continue
                else:
                    line2 = lines_coords_world[i]
                target_object_vertex = np.array(self.line_intersection(line1, line2))
                if target_object_vertex.all():
                    break

            if not target_object_vertex.all():
                pass
            else:
                np.append(target_object_vertex, [0.5])
                line1_vec = np.array([line1[1][0] - line1[0][0], line1[1][1] - line1[0][1]])
                line2_vec = np.array([line2[1][0] - line2[0][0], line2[1][1] - line2[0][1]])
                line1_norm_vec = line1_vec / np.linalg.norm(line1_vec)
                line2_norm_vec = line2_vec / np.linalg.norm(line2_vec)
                target_angle_vector = line1_norm_vec + line2_norm_vec
                np.append(target_angle_vector, [0.5])
                self.target_angle.display(list(target_object_vertex)+[0.5], list(target_object_vertex + target_angle_vector * 0.5)+[0.5], lifetime = rospy.Duration(1))

                cos = np.dot(target_angle_vector,np.array([0.5,0.5]))
                sgn = np.sign(float(np.cross(np.array([0.5,0.5]), target_angle_vector)))
                theta = sgn * np.arccos(np.clip(cos,-1.0,1.0))
                #rospy.loginfo("theta: %f", theta)
                self.target_object.display(list(target_object_vertex + target_angle_vector * 0.1)+[0.15], [0,0,theta], lifetime = rospy.Duration(1))
                break

if __name__ == '__main__':
    try:
        args = rospy.myargv(argv=sys.argv)
        get_object_data = GetObjectData("get_horizontal_object_info_test")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
