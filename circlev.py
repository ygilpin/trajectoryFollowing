#!/usr/bin/env python

##
#
# Quick script to control the UAV autonomously using mavros commands. 
# It attempts to implement a feed forward trajectory follower by 
# writing a series of calculus determined commanded velocities. 
# 
# Before running, check that:
#   - An instance of mavros is running and the connection is healthy
#   - The UAV's location (/mavros/local_position/pose) is accurate
#
##

import rospy
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose, PoseStamped, Twist
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL

class MavController:
    """
    A simple object to help interface with mavros 
    """
    def __init__(self):

        rospy.init_node("mav_control_node")
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)

        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        self.rc = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()

    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose

    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work. 
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz(self, x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        self.goto(pose)

        dist=math.sqrt((x-self.pose.position.x)**2 + (y-self.pose.position.y)**2 + (z-self.pose.position.z)**2)
        print(dist)

    def goto_xyz_w(self, x, y, z, arrival):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        self.goto(pose)

        dist=1;

        print("Waiting for arrival at")
        while dist > arrival:
            dist=math.sqrt((x-self.pose.position.x)**2 + (y-self.pose.position.y)**2 + (z-self.pose.position.z)**2)
        #print(dist)
        rospy.sleep(0.5)

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default. 
        """
        cmd_vel = Twist()
        
        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy 
        cmd_vel.linear.z = vz 
        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)
    
    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=1.0):
        """
        Arm the throttle, takeoff to a few feet, and set to guided mode
        """
        # Set to stabilize mode for arming
        mode_resp = self.mode_service(custom_mode="0")
        self.arm()

        # Set to guided mode 
        mode_resp = self.mode_service(custom_mode="4")

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        return takeoff_resp

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly, 
        land, and disarm. 
        """
        resp = self.mode_service(custom_mode="9")
        self.disarm()

def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    c = MavController()
    rospy.sleep(1)

    print("Takeoff")
    c.takeoff()
    rospy.sleep(10)

    print("Waypoint 1: position control")
    c.goto_xyz(1,1,1)
    rospy.sleep(5)
    print("Waypoint 2: position control")
    c.goto_xyz(0,0,1)
    rospy.sleep(5)

    print("Velocity Setpoint 1")
    c.set_vel(0,1,0)
    rospy.sleep(5)
    print("Velocity Setpoint 2")
    c.set_vel(0,-1,0)
    rospy.sleep(5)
    print("Velocity Setpoint 3")
    c.set_vel(0,0,0)
    rospy.sleep(5)
   
    print("Landing")
    c.land()

if __name__=="__main__":
    c = MavController()
    rospy.sleep(1)

    print("Takeoff")
    c.takeoff()
    rospy.sleep(5)
    
    goto_xyzw(1,0,1,0.01) 
    xc=[1]
    yc=[0]
    xr=[c.pose.position.x]
    yr=[c.pose.position.y]
    tv=[0]
    vxc=[0]
    vyc=[0]
    
    count = 0
    radStp=0.1
    r=0.5 # Radius of the prescribed circle
    v=1   # Prescribed linear velocity
    theta=0 # Initial Angle
    omega=v/r
    z=1
    t=0
    dt=0.2
    
    while t < 3*2*math.pi/omega:
        x=r*math.cos(omega*t)
        y=r*math.sin(omega*t)
        vx=-r*w*math.sin(omega*t)
        vy=r*w*math.cos(omega*t)
        # Record the commanded position and the actual position
        xc.append(x)
        yc.append(y)
        vxc.append(vx)
        vyc.append(vy)
        xr.append(c.pose.position.x)
        yr.append(c.pose.position.y)
        tv.append(t)
        
        c.set_vel(x,y,z)
        rospy.sleep(dt)
        t=t+dt;

    print("Landing")
    c.land()"""
    plt.figure(1)
    plt.subplot(1,2,1)
    plt.plot(xc,yc,xr,yr)
    plt.xlabel('X-Position(m)')
    plt.ylabel('Y-Position(m)')
    plt.title('Commanded vs Actual Trajectory')
    plt.legend(('Commanded Trajectory', 'Simulation Results'), loc='upper right')

    plt.subplot(1,2,2)
    plt.plot(tv,xc,tv,xr)
    plt.xlabel('Time (s)')
    plt.ylabel('X-Position(m)')
    plt.title('Commanded vs Actual Trajectory')
    plt.legend(('Commanded Trajectory', 'Simulation Results'), loc='upper right')
    plt.show()"""
    
