import numpy as np
import matplotlib.pyplot as plt
from plotter import *
from scipy.integrate import odeint
from video_plot import plot2D_to_video



m1 = 2
m2 = 2.5
g = 9.81

class PID:
    def __init__(self, Kp, Ki, Kd, SP,dt,limits):
        
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.error_prev = 0
        self.SP = SP
        self.min, self.max = limits
        self.I = 0
    
    def calc(self,PV):
        error = self.SP-PV

        self.I += error * self.dt
        D = (error - self.error_prev)/self.dt
        
        control = self.Kp * error + self.Ki*self.I  + self.Kd * D
        self.error_prev = error
        
        if control > self.max:
            control = self.max
            return control
        
        elif control < self.min:
            control = self.min
            return control
        
        else:
            return control
           


def get_p(q):
    q = np.array(q).squeeze()
    x,l,theta,_,_,_ = q
    return np.array([x+l*np.sin(theta), -l*np.cos(theta)])

def dynamics(X,t,U):
    l = X[1]
    theta = X[2]
    x_dot = X[3]
    l_dot = X[4]
    theta_dot = X[5]
    q_dot = np.array([x_dot, l_dot, theta_dot])

    J = np.array([
        [1, np.sin(theta),  l*np.cos(theta)],
        [0, -np.cos(theta), l*np.sin(theta)]
    ])

    J_dot = np.array([
        [0, theta_dot*np.cos(theta),  l_dot*np.cos(theta) - theta_dot*l*np.sin(theta)],
        [0, theta_dot*np.sin(theta),  l_dot*np.sin(theta) + theta_dot*l*np.cos(theta)]
    ])

    M_1 = np.array([
        [m1, 0, 0],
        [0,  0, 0],
        [0,  0, 0]
    ])

    A_1 = np.zeros((2,3))
    A_2 = np.array([
        [0, 0, np.cos(theta)],
        [0, 0, np.sin(theta)]
    ])
    A_3 = np.array([
        [0, np.cos(theta), -l*np.sin(theta)],
        [0, np.sin(theta),  l*np.cos(theta)]
    ])


    gamma1 = q_dot.T @ A_1.T @ J @ q_dot
    gamma2 = q_dot.T @ A_2.T @ J @ q_dot
    gamma3 = q_dot.T @ A_3.T @ J @ q_dot
    Gamma = m2 * np.array([gamma1, gamma2, gamma3])

    G = np.array([0, -m2*g*np.cos(theta), m2*g*l*np.sin(theta)])

    eps = 1e-6
    
    M = M_1 + m2 * (J.T @ J)

    Cqdot = m2 * (J_dot.T @ J + J.T @ J_dot) @ q_dot - Gamma

    q_ddot = np.linalg.solve(M+np.diag(np.ones(3)*eps), (U - Cqdot - G))
    dX = np.concatenate([q_dot, q_ddot])
    
    return dX

Kp_x = 6
Ki_x = 0.5
Kd_x = 2.5

Kp_y = 100
Ki_y = 10
Kd_y = 5

Ts = 0.01
pid_x = PID(Kp_x,Ki_x,Kd_x,2,Ts, [-70,70])
pid_y = PID(Kp_y,Ki_y,Kd_y,0.6,Ts, [-200,0])


def U(x0):
    x = x0[0]
    y = x0[1]
    
    fx = pid_x.calc(x)
    fl = pid_y.calc(y)
    
    return np.array([fx,fl,0])


t = np.linspace(0,30,3001)
N = len(t)
X_hist = np.zeros((N,6))
X_hist[0] = np.array([-2,0.6,0,0,0,0])
P_hist = np.zeros((N,2))
P_hist[0] = get_p(X_hist[0])

U_hist = np.zeros((N-1,3))

for k in range(N-1):
    Uk = U(X_hist[k])
    U_hist[k] = Uk

    tk = [t[k], t[k+1]]
    X_next = odeint(dynamics, X_hist[k], tk, args=(Uk,))
    X_hist[k+1] = X_next[-1]
    P_hist[k+1] = get_p(X_next[-1])

x_c = X_hist[:,0]
x_l = P_hist[:,0]
z_l = P_hist[:,1]
F, ul = U_hist[:,0], U_hist[:,1]

#plot2D(x_c,x_l,z_l,0.1)
#plotOscillations(X_hist[:,2],t, "PID", "oscillations_PID.png")
plotControls(t[1:],F,ul)