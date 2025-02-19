import os
from enum import Enum
import matplotlib.patches as patches
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.axes import Axes
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from ismpc import State
from simulation.mpc.gait import Gait
from simulation.utils import config, robot_config

class StatePlotter:
    def __init__(self):
        self.fig, self.ax = plt.subplots(3, 1, figsize=(10, 10))
        self.com_x_data, self.com_y_data, self.com_z_data = [], [], []
        self.zmp_x_data, self.zmp_y_data, self.zmp_z_data = [], [], []
        self.des_com_x_data, self.des_com_y_data, self.des_com_z_data = [], [], []
        self.des_zmp_x_data, self.des_zmp_y_data, self.des_zmp_z_data = [], [], []

        self.line_com_x,  = self.ax[0].plot([], [], label="COM X", color="blue")
        self.line_com_y,  = self.ax[1].plot([], [], label="COM Y", color="blue")
        self.line_com_z,  = self.ax[2].plot([], [], label="COM Z", color="blue")
        self.line_zmp_x,  = self.ax[0].plot([], [], label="ZMP X", color="red")
        self.line_zmp_y,  = self.ax[1].plot([], [], label="ZMP Y", color="red")
        self.line_zmp_z,  = self.ax[2].plot([], [], label="ZMP Z", color="red")
        self.line_des_com_x,  = self.ax[0].plot([], [], label="Desired COM X", color="green")
        self.line_des_com_y,  = self.ax[1].plot([], [], label="Desired COM Y", color="green")
        self.line_des_com_z,  = self.ax[2].plot([], [], label="Desired COM Z", color="green")
        self.line_des_zmp_x,  = self.ax[0].plot([], [], label="Desired ZMP X", color="orange")
        self.line_des_zmp_y,  = self.ax[1].plot([], [], label="Desired ZMP Y", color="orange")
        self.line_des_zmp_z,  = self.ax[2].plot([], [], label="Desired ZMP Z", color="orange")

        plt.ion()
        plt.show()


    def update_plot(self, state: State) -> None:
        self.com_x_data.append(state.lip.com_pos[0])
        self.com_y_data.append(state.lip.com_pos[1])
        self.com_z_data.append(state.lip.com_pos[2])
        self.zmp_x_data.append(state.lip.zmp_pos[0])
        self.zmp_y_data.append(state.lip.zmp_pos[1])
        self.zmp_z_data.append(state.lip.zmp_pos[2])
        self.des_com_x_data.append(state.desired_lip.com_pos[0])
        self.des_com_y_data.append(state.desired_lip.com_pos[1])
        self.des_com_z_data.append(state.desired_lip.com_pos[2])
        self.des_zmp_x_data.append(state.desired_lip.zmp_pos[0])
        self.des_zmp_y_data.append(state.desired_lip.zmp_pos[1])
        self.des_zmp_z_data.append(state.desired_lip.zmp_pos[2])
        self.line_com_x.set_data(np.arange(len(self.com_x_data)), self.com_x_data)
        self.line_com_y.set_data(np.arange(len(self.com_y_data)), self.com_y_data)
        self.line_com_z.set_data(np.arange(len(self.com_z_data)), self.com_z_data)
        self.line_zmp_x.set_data(np.arange(len(self.zmp_x_data)), self.zmp_x_data)
        self.line_zmp_y.set_data(np.arange(len(self.zmp_y_data)), self.zmp_y_data)
        self.line_zmp_z.set_data(np.arange(len(self.zmp_z_data)), self.zmp_z_data)
        self.line_des_com_x.set_data(np.arange(len(self.des_com_x_data)), self.des_com_x_data)
        self.line_des_com_y.set_data(np.arange(len(self.des_com_y_data)), self.des_com_y_data)
        self.line_des_com_z.set_data(np.arange(len(self.des_com_z_data)), self.des_com_z_data)
        self.line_des_zmp_x.set_data(np.arange(len(self.des_zmp_x_data)), self.des_zmp_x_data)
        self.line_des_zmp_y.set_data(np.arange(len(self.des_zmp_y_data)), self.des_zmp_y_data)
        self.line_des_zmp_z.set_data(np.arange(len(self.des_zmp_z_data)), self.des_zmp_z_data)

        # set limits
        for i in range(3):
            self.ax[i].relim()
            self.ax[i].autoscale_view()

        # redraw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()



def extract_feet_patches(
    f: int, gait: Gait
) -> list[Rectangle]:
    sidex = robot_config.dxz
    sidey = robot_config.dyz

    right_feet = []
    initial_right_foot_patch = patches.Rectangle(
        (robot_config.right_foot_x - sidex / 2, robot_config.right_foot_y - sidey / 2),
        angle=0,
        rotation_point="center",
        width=sidex,
        height=sidey,
        fill=False,
        color="r",
    )
    right_feet.append(initial_right_foot_patch)
    right_feet.extend(
        [
            patches.Rectangle(
                (gait.xf[j] - sidex / 2, gait.yf[j] - sidey / 2),
                angle=gait.thetaf[j] * 180 / np.pi,
                rotation_point="center",
                width=sidex,
                height=sidey,
                fill=False,
                color="r",
            )
            for j in range(0, f, 2)
        ]
    )


    left_feet = []
    initial_left_foot_patch = patches.Rectangle(
        (robot_config.left_foot_x - sidex / 2, robot_config.left_foot_y - sidey / 2),
        angle=0,
        rotation_point="center",
        width=sidex,
        height=sidey,
        fill=False,
        color="b",
    )
    left_feet.append(initial_left_foot_patch)
    left_feet.extend(
        [
            patches.Rectangle(
                (gait.xf[j] - sidex / 2, gait.yf[j] - sidey / 2),
                angle=gait.thetaf[j] * 180 / np.pi,
                rotation_point="center",
                width=sidex,
                height=sidey,
                fill=False,
                color="b",
            )
            for j in range(1, f, 2)
        ]
    )


    return right_feet + left_feet


def plot_2d(ax: Axes, k: int, f: int, gait: Gait) -> None:
    ax.plot(gait.com_traj[0, :k], gait.com_traj[1, :k], color="k")
    ax.plot(gait.zmp_traj[0, :k], gait.zmp_traj[1, :k], color="g")
    ax.plot(gait.mc_x_traj[:, max(0, k-1)], gait.mc_y_traj[:, max(0, k-1)], color="c")
    ax.legend(["CoM", "ZMP", "Moving Constraint"]) 
    for foot in extract_feet_patches(max(0, f-1), gait):
        ax.add_patch(foot)


def plot_3d(ax: Axes, k: int, f: int, gait: Gait) -> None:
    ax.plot(gait.com_traj[0, :k], gait.com_traj[1, :k], gait.com_traj[2, :k], color="k")
    ax.plot(gait.zmp_traj[0, :k], gait.zmp_traj[1, :k], gait.zmp_traj[2, :k], color="g")
    ax.plot(
        gait.left_foot_traj[0, :k],
        gait.left_foot_traj[1, :k],
        gait.left_foot_traj[2, :k],
        color="r",
    )
    ax.plot(
        gait.right_foot_traj[0, :k],
        gait.right_foot_traj[1, :k],
        gait.right_foot_traj[2, :k],
        color="b",
    )
    ax.legend(["CoM", "ZMP", "Left Foot", "Right Foot"])
    for foot in extract_feet_patches(f, gait):
        x, y = foot.get_xy()
        width = foot.get_width()
        height = foot.get_height()
        angle = foot.get_angle()
        theta = np.radians(angle)

        vertices = np.array(
            [
                [x, y, 0],
                [x + width * np.cos(theta), y + width * np.sin(theta), 0],
                [
                    x + width * np.cos(theta) - height * np.sin(theta),
                    y + width * np.sin(theta) + height * np.cos(theta),
                    0,
                ],
                [x - height * np.sin(theta), y + height * np.cos(theta), 0],
            ]
        )

        poly = Poly3DCollection([vertices], color=foot.get_edgecolor(), alpha=0.3)
        ax.add_collection3d(poly)


class PlotMode(Enum):
    TWO_D = "2d"
    THREE_D = "3d"

def animate(
    gait: Gait,
    save: bool = True,
    plot_mode: PlotMode = PlotMode.TWO_D,
) -> None:
    fig = plt.figure()
    max_x = max(gait.zmp_traj[0, :])
    min_x = min(gait.zmp_traj[0, :])
    max_y = max(gait.zmp_traj[1, :]) + 0.2
    min_y = min(gait.zmp_traj[1, :]) - 0.2

    x_pad = 0.3 * (max_x - min_x) + 0.2
    y_pad = 0.5 * (max_y - min_y) - 0.2

    if not os.path.exists(f"videos/{plot_mode.value}"):
        os.makedirs(f"videos/{plot_mode.value}")

    def update(k):
        plt.clf()
        ax = fig.add_subplot(111)
        if plot_mode == PlotMode.THREE_D:
            ax = fig.add_subplot(111, projection="3d")
        ax.set_xlim(min_x - x_pad, max_x + x_pad)
        ax.set_ylim(min_y - y_pad, max_y + y_pad)
        ax.set_aspect("equal")

        f = 0
        time = k * config.delta
        for i in range(0, len(gait.timestamps)):
            if gait.timestamps[i] <= time:
                f += 1

        if plot_mode == PlotMode.TWO_D:
            plot_2d(ax, k, f, gait)
        elif plot_mode == PlotMode.THREE_D:
            plot_3d(ax, k, f, gait)
            
        print("==========================================================")
        print(f"ITERATION {k}, Time: {time}")
        for fs in gait.fs_plan_history[max(0, k-1)]:
            print(fs)
        print("==========================================================\n\n")
        

    ani = FuncAnimation(
        fig, update, frames=config.N + 1, repeat=False, interval=config.delta
    )

    if save:
        ani.save(f"videos/{plot_mode.value}/walking.mp4", writer="ffmpeg", fps=83)
    plt.show()
