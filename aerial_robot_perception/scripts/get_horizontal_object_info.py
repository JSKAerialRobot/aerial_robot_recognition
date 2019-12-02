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

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(x_diff, y_diff)
        if div == 0:
            rospy.logwarn("lines do not intersect")
            return None
        # if div > 0:
        #     rospy.logwarn("lines intersect in the back")
        #     return None

        d = (det(*line1), det(*line2))
        x = det(d, x_diff) / div
        y = det(d, y_diff) / div
        return x, y

    def getobjectdataCallback(self, msg):
        #rospy.loginfo("number of points: %d",len(msg.points))
        if (len(msg.points) < 3):
            rospy.logwarn("number of detected points: %d",len(msg.points))
            return

        point_coords_camera = []
        lines_coords_camera = []
        for i in range(len(msg.points)):
            point_coords_camera.append([msg.points[i].x, msg.points[i].y, msg.points[i].z])
            if (i % 2) == 1:
                if point_coords_camera[i-1][0] > point_coords_camera[i][0]:
                    point_coords_camera[i-1], point_coords_camera[i] = point_coords_camera[i], point_coords_camera[i-1]
            lines_coords_camera.append([point_coords_camera[i-1], point_coords_camera[i]])
        #lines_coords_camera.sort(key=lambda arg: arg[0][1])
        lines_coords_camera.sort(key=lambda arg: arg[0][0])
        lines_coords_camera = list(itertools.chain.from_iterable(lines_coords_camera))
        point_st = PoseStamped()
        point_st.header.frame_id = self.camera_frame_name
        orientation = Quaternion()
        point_st.pose.orientation = orientation
        point_coords_world = []
        for i in range(4):
            point_st.pose.position.x = lines_coords_camera[i][0]
            point_st.pose.position.y = lines_coords_camera[i][1]
            point_st.pose.position.z = lines_coords_camera[i][2]

            trans = self.tfBuffer.transform(point_st, 'world')
            point_coords_world.append([trans.pose.position.x,trans.pose.position.y, trans.pose.position.z])
            #rospy.loginfo("points%d: (%f, %f, %f)", i, point_coords_world[i][0], point_coords_world[i][1], point_coords_world[i][2])

        length_list = []
        lines = []
        # for i in range(len(msg.points)-1):
        #     lines.append([[point_coords_world[i][0],point_coords_world[i][1]],[point_coords_world[i+1][0],point_coords_world[i+1][1]]])
        #     if i >= 1:
        #         target_object_vertex = np.array(self.line_intersection(lines[i-1], lines[i]))
        #         if not target_object_vertex.all():
        #             continue
        #         else:
        #             #lines[0] = lines[i-1]
        #             #lines[1] = lines[i]
        #             break

            #length_list.append(math.sqrt((point_coords_world[i][0] - point_coords_world[i+1][0])**2 + (point_coords_world[i][1] - point_coords_world[i+1][1])**2 + (point_coords_world[i][2] - point_coords_world[i+1][2])**2))

        for i in range(2):
           lines.append([[point_coords_world[i*2][0],point_coords_world[i*2][1]],[point_coords_world[i*2+1][0],point_coords_world[i*2+1][1]]])
           length_list.append(math.sqrt((point_coords_world[i*2][0] - point_coords_world[i*2+1][0])**2 + (point_coords_world[i*2][1] - point_coords_world[i*2+1][1])**2 + (point_coords_world[i*2][2] - point_coords_world[i*2+1][2])**2))
           #rospy.loginfo("length%d: %f", i, length_list[i])

        target_object_vertex = np.array(self.line_intersection(lines[0], lines[1]))
        if not target_object_vertex.all():
            '''no intersection point'''
            return
        np.append(target_object_vertex, [0.5])
        line1_vec = np.array([lines[0][1][0] - lines[0][0][0], lines[0][1][1] - lines[0][0][1]])
        line2_vec = np.array([lines[1][1][0] - lines[1][0][0], lines[1][1][1] - lines[1][0][1]])
        line1_norm_vec = line1_vec / np.linalg.norm(line1_vec)
        line2_norm_vec = line2_vec / np.linalg.norm(line2_vec)
        #if abs(np.dot(line1_norm_vec, line2_norm_vec)) >= 0.7: # 1.0/math.sqrt(2)
        #    rospy.logwarn("detected lines do not meet at right angles")
        #    return
        target_angle_vector = line1_norm_vec + line2_norm_vec
        np.append(target_angle_vector, [0.5])
        #rospy.loginfo(target_angle_vector)
        #rospy.logwarn(list(target_object_vertex + target_angle_vector * 0.2 * math.sqrt(2)))
        self.target_angle.display(list(target_object_vertex)+[0.5], list(target_object_vertex + target_angle_vector * 0.5)+[0.5], lifetime = rospy.Duration(1))

        cos = np.dot(target_angle_vector,np.array([0.5,0.5]))
        sgn = np.sign(float(np.cross(np.array([0.5,0.5]), target_angle_vector)))
        theta = sgn * np.arccos(np.clip(cos,-1.0,1.0))
        #rospy.loginfo("theta: %f", theta)
        self.target_object.display(list(target_object_vertex + target_angle_vector * 0.1)+[0.15], [0,0,theta], lifetime = rospy.Duration(1))

if __name__ == '__main__':
    try:
        args = rospy.myargv(argv=sys.argv)
        get_object_data = GetObjectData("get_horizontal_object_info_test")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
