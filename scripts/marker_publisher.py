#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
import rospy
import math

topic_marker = 'turtlebot3_marker'
publisher_marker = rospy.Publisher(topic_marker, Marker, queue_size=5)

topic_facing = 'turtlebot3_facing'
publisher_facing = rospy.Publisher(topic_facing, Marker, queue_size=5)

rospy.init_node('register')

count = 0
MARKERS_MAX = 100

while not rospy.is_shutdown():

   marker = Marker()
   marker.header.frame_id = "base_footprint"
   marker.type = marker.CYLINDER
   marker.action = marker.ADD
   marker.scale.x = 0.5
   marker.scale.y = 0.5
   marker.scale.z = 0.1
   marker.color.a = 1.0
   marker.color.r = 0.9
   marker.color.g = 0.9
   marker.color.b = 0.9

   vector = Marker()
   vector.header.frame_id = "base_footprint"
   vector.type = vector.ARROW
   vector.action = vector.ADD
   vector.scale.x = 1
   vector.scale.y = 0.1
   vector.scale.z = 0.1
   vector.color.a = 1.0
   vector.color.r = 0.9
   vector.color.g = 0.9
   vector.color.b = 0.9
   '''
   vector.pose.orientation.w = 1.0
   vector.pose.position.x = math.cos(count / 50.0)
   vector.pose.position.y = math.cos(count / 40.0)
   vector.pose.position.z = math.cos(count / 30.0)
   '''
   # We add the new marker to the MarkerArray, removing the oldest
   # marker from it when necessary

   # Publish the MarkerArray
   publisher_marker.publish(marker)
   publisher_facing.publish(vector)
   r = rospy.Rate(10)
   r.sleep()
