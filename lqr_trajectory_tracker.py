#!/usr/bin/env python3

##
#
# A simple script to track a reference trajectory using discrete-time LQR
#
##

import numpy as np
import matplotlib.pyplot as plt

def finite_horizon_lqr(A, B, N, rho=10):
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

    for t in range(N,0,-1):
        K[t-1] = - np.linalg.inv(R+np.transpose(B) @ P[t] @ B) @ np.transpose(B) @ P[t] @ A
        P[t-1] = Q + np.transpose(A) @ P[t] @ A - np.transpose(A) @ P[t] @ B @ \
                np.linalg.inv( R + np.transpose(B) @ P[t] @ B ) @ np.transpose(B) @ P[t] @ A

    return K

def get_reference_u(x_ref,A,B):
    """
    Find a reference control input for the linear system
    
        x(t+1) = A*x(t) + B*u(t)

    Given a sequence of reference states x_ref. Assumes B
    is a square matrix.
    """
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
        u_ref[i] = Binv @ (x_ref[i+1] - A @ x_ref[i])

    return u_ref


# System parameters
N = 100
t = [i for i in range(N)]
A = np.array([[1,0,0],[0,1,0],[0,0,1]])
B = 0.1*np.array([[1,0,0],[0,1,0],[0,0,1]])

# Reference trajectory
x1 = [np.sin(0.2*i) for i in t]
x2 = [np.cos(0.2*i) for i in t]
x3 = [0.0 for i in t]

x_ref = np.array((x1,x2,x3)).T    # Nx3 array
u_ref = get_reference_u(x_ref,A,B)

# Simulation

K = finite_horizon_lqr(A, B, N, rho=1)
x = np.array([-0.2,0.75,0.1])

for i in t[0:N-1]:

    plt.scatter(i,x[0],color="blue")
    plt.scatter(i,x[1],color="orange")
    plt.scatter(i,x[2],color="green")

    u = - K[i] @ (x_ref[i] - x) + u_ref[i]
    x = A @ x + B @ u

plt.plot(t,x_ref)
plt.show()


