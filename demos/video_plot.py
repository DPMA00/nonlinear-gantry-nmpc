import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, FFMpegWriter

def plot2D_to_video(xc, xl, zl, Ts, out_path="simulation1.mp4", fps=None):
    # If Ts is your simulation timestep (seconds), fps = 1/Ts
    if fps is None:
        fps = int(round(1.0 / Ts)) if Ts > 0 else 30

    fig, ax = plt.subplots()
    ax.set_xlim(-5, 5)
    ax.set_ylim(-1.5, 0.1)
    ax.grid(True)

    cart_point, = ax.plot([], [], 'ks', markersize=20)   # cart
    cable_line, = ax.plot([], [], '-', linewidth=2)      # cable
    load_point, = ax.plot([], [], 'ro', markersize=15)   # payload
    ax.plot([-5, 5], [0, 0], 'k--', linewidth=1)         # ground line

    # Downsample to keep video length/size reasonable
    step = max(1, len(xc) // 500)
    idx = np.arange(0, len(xc), step)
    idx = np.arange(len(xc))
    
    def init():
        cart_point.set_data([], [])
        cable_line.set_data([], [])
        load_point.set_data([], [])
        return cart_point, cable_line, load_point

    def update(frame_i):
        i = idx[frame_i]
        cart_point.set_data([xc[i]], [0.0])
        cable_line.set_data([xc[i], xl[i]], [0.0, zl[i]])
        load_point.set_data([xl[i]], [zl[i]])
        return cart_point, cable_line, load_point

    ani = FuncAnimation(
        fig,
        update,
        frames=len(idx),
        init_func=init,
        blit=True,
        interval=1000 / fps,  # ms between frames for preview timing
    )

    writer = FFMpegWriter(fps=fps, bitrate=2000)
    ani.save(out_path, writer=writer)
    plt.close(fig)
    return out_path