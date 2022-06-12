#!/usr/bin/env python3
from decimal import DivisionByZero
import rospy
from math import *
import time
import sys
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import Header

nv = Odometry()
nv.pose.pose.position


class CircleNode:
    def __init__(self, namespace):
        rospy.init_node("Drone_Circle")
        print(namespace)
        rospy.loginfo("Starting CircleNode as Drone_Circle.")
        self.connec_ = False
        self.armed_ = False
        self.guided_ = False
        self.takeoff_ = False
        self.currPos = Point()
        self.reqPos = PoseStamped()
        self.reqVel = TwistStamped()
        self.reqPos.header.frame_id = "map"
        self.reqVel.header.frame_id = "map"
        self.reqHeader = Header()
        self.PosePub = rospy.Publisher(namespace + "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.VelPub = rospy.Publisher(namespace + "/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        # self.GeoPose = rospy.Publisher("drone1/mavros/setpoint_position/global")
        rospy.Subscriber(namespace + "/mavros/state", State, queue_size=10, callback=self.state_cb)
        rospy.Subscriber(namespace + "/mavros/gloabal_position/local", Odometry, self.pose_cb, queue_size=10)
        self.arming_client = rospy.ServiceProxy(namespace + "/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy(namespace + "/mavros/set_mode", SetMode)
        self.takeoff_client = rospy.ServiceProxy(namespace + "/mavros/cmd/takeoff", CommandTOL)
        self.theta = 90
        

    def state_cb(self, data):
        self.connec_ = data.connected
        self.armed_ = data.armed
        self.guided_ = data.guided

    def pose_cb(self, data):
        self.currPos.x = data.pose.pose.position.x
        self.currPos.y = data.pose.pose.position.y
        self.currPos.z = data.pose.pose.position.z
        if self.currPos.z>1:
            self.takeoff_= True

    def arm_drone(self):
        arm_req = CommandBoolRequest()
        arm_req.value = True
        arm_res = self.arming_client(arm_req)
        if arm_res.success:
            self.armed_ = True
            rospy.loginfo("Arming Succesfull")
        else:
            rospy.logerr("Arming failed")

    def setmode_drone(self, mode):
        mode_req = SetModeRequest()
        mode_req.base_mode = 0
        mode_req.custom_mode = mode
        mode_res = self.set_mode_client(mode_req)
        if mode_res:
            self.guided_ = True
            rospy.loginfo("mode set to guided")
        else:
            rospy.logerr("Error in setting mode")
    
    def takeoff_drone(self, height):
        takeoff_req = CommandTOLRequest()
        takeoff_req.altitude = height
        takeoff_res = self.takeoff_client(takeoff_req)
        if takeoff_res.success:
            self.takeoff_ = True
            rospy.loginfo("Takeoff Success")
        else:
            rospy.logerr("Takeoff Failed")
    
    def update(self):
        
        self.theta += 0.00001
        self.reqVel.header.stamp = rospy.Time.now()
        self.reqVel.twist.linear.x = cos(self.theta)
        self.reqVel.twist.linear.y = sin(self.theta)
        self.reqVel.twist.linear.z = 0
        self.VelPub.publish(self.reqVel)
        # print(self.reqPos)

if __name__ == "__main__":
    Drone_Circle = CircleNode(sys.argv[1])
    while not rospy.is_shutdown():
        if not (Drone_Circle.connec_):
            rospy.loginfo("Waiting For Connection")
            continue
        if not Drone_Circle.guided_ :
            Drone_Circle.setmode_drone("GUIDED")
            print("mode is bieng set to guided")
            time.sleep(35)
            continue
        if not Drone_Circle.armed_ :
            Drone_Circle.arm_drone()
            time.sleep(5)
            continue
        if Drone_Circle.guided_ and Drone_Circle.armed_ and not Drone_Circle.takeoff_:
            Drone_Circle.takeoff_drone(3)
            time.sleep(10)
            continue
        Drone_Circle.update()
    rospy.spin()