#!/usr/bin/env python

##
#
# Quick script to control the UAV autonomously using mavros commands. 
# Using an LQR trajectory following control system to follow a figure 8.
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
import numpy as np

class MavController:
    """
    A simple object to help interface with mavros 
    """
    def __init__(self): 
        #print("Constructor has been called")

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
            rospy.sleep(0.2)
        print("Arrived")
        rospy.sleep(0.2)

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

    def finite_horizon_lqr(self, A, B, N, rho):
        """
        Find state feedback controllers for the linear system
        
            x(t+1) = A*x(t) + B*u(t) 
        
        to minimize the cost function

            J = sum_N(x^2) + rho*sum_N(u^2)

        for the next N timesteps
        """
        assert A.shape[0] == A.shape[1], "Nonsquare A matrix."
        assert A.shape[0] == B.shape[0], "Check A and B dimensions."

        n = A.shape[1]
        m = B.shape[1]

        # Cost function definition
        Q = np.identity(n)
        Qf = np.identity(n)
        R = rho*np.identity(m)

        # Allocate array of matrices for P_t and K_t
        P = np.full((N+1,n,n),np.inf)
        K = np.full((N,m,n),np.inf)

        # Set P and K values recursively from N
        P[N] = Qf

        R = np.matrix(R)
        Q = np.matrix(Q)
        for t in range(N,0,-1):
            #Kprime = np.matrix(K[t])
            Pprime = np.matrix(P[t])
            K[t-1] = - np.linalg.inv(R + B.T * Pprime * B) * B.T * Pprime * A
            P[t-1] = Q + A.T * Pprime * A - A * Pprime * B * (np.linalg.inv(R + B.T * Pprime * B)) * B.T * Pprime * A

            """K[t-1] = - np.matmul(np.linalg.inv(R+np.matmul(np.transpose(B), np.matmul(P[t],B))), np.matmul(np.transpose(B), np.matmul(P[t], A)))
            S= np.matmul(np.linalg.inv(R+np.matmul(np.transpose(B), np.matmul(P[t], B))), np.matmul(np.transpose(B), np.matmul(P[t],A)))
            P[t-1] = Q + np.matmul(np.transpose(A),np.matmul(P[t], A)) - np.matmul(np.matmul(np.transpose(A), np.matmul(P[t] , B)), S)"""

        return K

    def get_reference_u(self, x_ref, A, B):
        """ 
        Find a reference control input for the linear system
        
            x(t+1) = A*x(t) + B*u(t)

        Given a sequence of reference states x_ref. Assumes B
        is a square matrix."""
       
        assert A.shape[1] == x_ref.shape[1], "Incorrect Reference Trajectory Dimension."
        assert A.shape[0] == A.shape[1], "Nonsquare A matrix."
        assert B.shape[0] == B.shape[1], "Nonsquare B matrix."
        assert A.shape[0] == B.shape[0], "Check A and B dimensions."

        N = x_ref.shape[0]  # number of timesteps
        n = A.shape[1]  # dimensionality of the state space
        m = B.shape[1]  # dimension of the control space
        
        Binv = np.linalg.inv(B)
        u_ref = np.full((N-1,m),np.inf)

        for i in range(N-1):
            u_ref[i] = np.matmul(Binv , (x_ref[i+1] - np.matmul(A , x_ref[i])))

        return u_ref

    def heaviside(self,x1):
        if x1 < 0:
            y = 0
        else:
            y = 1
        return y

if __name__=="__main__":
    print("Starting Main function")
    c = MavController()
    rospy.sleep(1)

    
    # System parameters
    Ts = 0.1
    A = np.array([[1,0,0],[0,1,0],[0,0,1]])     
    B = Ts*np.array([[1,0,0],[0,1,0],[0,0,1]]) 
    T = 15
    omega = 2*math.pi*Ts/T
    N = int(2*T/Ts)                                     
    t = [i for i in range(N)]                   
    rM=1
    rm=1
    rH=1
    rho=500

    # Reference trajectory
    x1 = [(rm*np.sin(omega*i)) for i in t]
    x2 = [(rM*np.sin(omega*i)*np.cos(omega*i)) for i in t]
    x3 = [rH for i in t]

    x_ref = np.array((x1,x2,x3)).T    # Nx3 array
    u_ref = c.get_reference_u(x_ref,A,B)
    K = c.finite_horizon_lqr(A, B, N, rho)

    # Taking off
    print("Takeoff")
    c.takeoff()
    rospy.sleep(5)

    # Set the initial position and wait for arrival
    c.goto_xyz_w(x1[0], x2[0], x3[0], 0.1)
    print("Starting figure 8")

    for i in t[0:N-1]:

        x = np.array([c.pose.position.x, c.pose.position.y, c.pose.position.z])
        plt.scatter(i,x[0],color="blue")
        plt.scatter(i,x[1],color="orange")
        plt.scatter(i,x[2],color="green")

        u = - np.matmul(K[i],(x_ref[i] - x)) + u_ref[i]
        c.set_vel(u[0], u[1], u[2])
        rospy.sleep(Ts)


    # Landing
    print("Landing")
    c.land()

    # Data Analysis
    #V=np.diff(x)/Ts
    #Vmax=np.maximum(np.maximum(x))
    plt.plot(t,x_ref)
    plt.show()
    """plt.figure(1)
    plt.subplot(1,2,1)
    plt.plot(xc,yc,xr,yr)
    plt.xlabel('X-Position(m)')
    plt.ylabel('Y-Position(m)')
    plt.title('Commanded vs Actual Trajectory')
    plt.legend(('Commanded Trajectory', 'Simulation Results'), loc='upper right')

    plt.subplot(1,2,2)
    plt.plot(t,xc,t,xr)
    plt.xlabel('Time (s)')
    plt.ylabel('X-Position(m)')
    plt.title('Commanded vs Actual Trajectory')
    plt.legend(('Commanded Trajectory', 'Simulation Results'), loc='upper right')
    plt.show()"""
