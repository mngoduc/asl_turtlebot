#!/usr/bin/env python

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
import tf
import math
from enum import Enum
from visualization_msgs.msg import Marker, MarkerArray

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 10

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .5

# time taken to cross an intersection
CROSSING_TIME = 3

# state machine modes, not all implemented
class Mode(Enum):
    WAITING = 1
    EXPLORING = 2
    GATHERING = 3
    DELIVERING = 4

class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor_nav', anonymous=True)
        # initialize variables
        self.found_objects = {
            "banana": [0, 0, 0],
            "broccoli": [0, 0, 0],
            "hot_dog": [0, 0, 0],
            "bottle": [0, 0, 0]
        }
        self.marker_colors = {
            "banana": [1.0, 0.95, 0],
            "broccoli": [0, 1.0, 0],
            "hot_dog": [1.0, 0, 0],
            "bottle": [0, 0, 1.0]
        }
        self.x = 0
        self.y = 0
        self.theta = 0
        self.mode = Mode.WAITING
        self.last_mode_printed = None
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # nav pose for controller
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        # command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # rviz marker publisher
        self.marker_publisher = rospy.Publisher('/markers', MarkerArray)
        # marker array
        self.marker_array = MarkerArray()

        # subscribers

        # high-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        # we can subscribe to nav goal click
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        # subscriber for image detection
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.detector_callback)

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if mapping else "/odom"
        print("rviz command received!")

        try:
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (
                    nav_pose_origin.pose.orientation.x,
                    nav_pose_origin.pose.orientation.y,
                    nav_pose_origin.pose.orientation.z,
                    nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.mode = Mode.EXPLORING

    # Sets robot move commands
    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.NAV

    #detector callback
    def detector_callback(self, msg):
        objects_str = msg.objects
        #rospy.loginfo(objects_str)
        if "banana" in objects_str:
            obj = msg.ob_msgs[msg.objects.index("banana")]
            self.found_objects["banana"] = [self.x, self.y, self.theta]
            dist = obj.distance
            rospy.loginfo("FOUND BANANANANANANA")
            self.add_marker("banana")
            #rospy.loginfo(obj.distance)
        if "broccoli" in objects_str:
            obj = msg.ob_msgs[msg.objects.index("broccoli")]
            self.found_objects["broccoli"] = [self.x, self.y, self.theta]
            dist = obj.distance
            rospy.loginfo("FOUND BROCCOLILILI")
            self.add_marker("broccoli")
            #rospy.loginfo(obj.distance)
        if "hot_dog" in objects_str:
            obj = msg.ob_msgs[msg.objects.index("hot_dog")]
            self.found_objects["hot_dog"] = [self.x, self.y, self.theta]
            dist = obj.distance
            rospy.loginfo("HOT DIGGITY DOG")
            self.add_marker("hot_dog")
            #rospy.loginfo(obj.distance)
        if "bottle" in objects_str:
            obj = msg.ob_msgs[msg.objects.index("bottle")]
            self.found_objects["bottle"] = [self.x, self.y, self.theta]
            dist = obj.distance
            self.add_marker("bottle")
            rospy.loginfo("ROBOTS DRUNK")
            #rospy.loginfo(obj.distance)
        #rospy.loginfo(self.found_objects)
        #rospy.loginfo(msg.objects)
        #rospy.loginfo(msg.ob_msgs[0].distance)

    def state_exploring(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

        #

    def state_waiting(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def add_marker(self, object_type):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = self.marker_colors[object_type][0]
        marker.color.g = self.marker_colors[object_type][1]
        marker.color.b = self.marker_colors[object_type][2]
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.found_objects[object_type][0]
        marker.pose.position.y = self.found_objects[object_type][1]
        marker.pose.position.z = 0 #self.found_objects[object_type][2]

        #renumber ids
        id = 0
        for m in self.marker_array.markers:
            m.id = id
            id += 1

        self.marker_array.markers.append(marker)
        self.marker_publisher.publish(self.marker_array)
        return marker

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS \
            and abs(theta-self.theta)<THETA_EPS)

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        try:
            origin_frame = "/map" if mapping else "/odom"
            (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
            self.x = translation[0]
            self.y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.theta = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.WAITING:
            # send zero velocity
            self.state_waiting()

        elif self.mode == Mode.EXPLORING:
            # moving towards a desired pose through Rviz - exploring phase
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.WAITING
            else:
                self.state_exploring()
            #
        #elif self.mode == Mode.GATHERING:
            # Something

        #elif self.mode == Mode.DELIVERING:
            # Something

        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
