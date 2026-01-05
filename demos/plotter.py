import matplotlib.pyplot as plt
import numpy as np

def plot2D(xc,xl,zl,Ts):
    plt.ion()
    fig, ax = plt.subplots()

 
    ax.set_xlim(-5, 5)
    ax.set_ylim(-1.5, 0.1)
    ax.grid(True)

    cart_point, = ax.plot([], [], 'ks', markersize=20) 
    cable_line, = ax.plot([], [], '-', linewidth=2) 
    load_point, = ax.plot([], [], 'ro', markersize=15) 


    ax.plot([-5, 5], [0, 0], 'k--', linewidth=1)

    step = max(1, len(xc)//500) 

    for i in range(0, len(xc), step):
        cart_point.set_data([xc[i]], [0.0])
        cable_line.set_data([xc[i], xl[i]], [0.0, zl[i]])
        load_point.set_data([xl[i]], [zl[i]])
        fig.canvas.draw_idle()
        plt.pause(Ts)

    plt.ioff()
    plt.show()
    
def plotOscillations(theta,ts, method="PID",save=False, filename="oscillations.png"):
    fig, ax = plt.subplots()
    ax.grid(True)
    
    ax.set_title("Oscillatory Motion of Payload: " + method)
    ax.plot(ts,theta)
    ax.set_xlabel(r"$t\;(\mathrm{s})$")
    ax.set_ylabel(r"$\theta\;(\mathrm{rad})$")

    if save:
        fig.savefig(filename, dpi=300, bbox_inches="tight")

    plt.show()

def plotControls(ts,F,ul, save=False, filename="controls.png"):
    fig, ax = plt.subplots(2, 1, sharex=True)

    ax[0].step(ts, F, where="post")
    ax[0].grid(True)
    ax[0].set_ylabel("F [N]")
    ax[0].set_title("Control inputs (stairs)")

    ax[1].step(ts, ul, where="post")
    ax[1].grid(True)
    ax[1].set_ylabel("ul")
    ax[1].set_xlabel("time [s]")

    
    plt.show()