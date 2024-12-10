import os
from enum import Enum
import matplotlib.patches as patches
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.axes import Axes
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from omegaconf import OmegaConf

from gait import Gait

def extract_feet_patches(
    f: int, gait: Gait, robot_config: OmegaConf
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


def plot_2d(ax: Axes, k: int, f: int, gait: Gait, robot_config: OmegaConf) -> None:
    ax.plot(gait.com_traj[0, :k], gait.com_traj[1, :k], color="k")
    ax.plot(gait.zmp_traj[0, :k], gait.zmp_traj[1, :k], color="g")
    ax.legend(["CoM", "ZMP"])
    for foot in extract_feet_patches(f, gait, robot_config):
        ax.add_patch(foot)


def plot_3d(ax: Axes, k: int, f: int, gait: Gait, robot_config: OmegaConf) -> None:
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
    for foot in extract_feet_patches(f, gait, robot_config):
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
    config: OmegaConf,
    robot_config: OmegaConf,
    save: bool = True,
    plot_mode: PlotMode = PlotMode.TWO_D,
) -> None:
    fig = plt.figure()

    max_x = max(gait.zmp_traj[0, :])
    min_x = min(gait.zmp_traj[0, :])
    max_y = max(gait.zmp_traj[1, :])
    min_y = min(gait.zmp_traj[1, :])

    x_pad = 0.3 * (max_x - min_x)
    y_pad = 0.5 * (max_y - min_y)

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
            plot_2d(ax, k, f, gait, robot_config)
        elif plot_mode == PlotMode.THREE_D:
            plot_3d(ax, k, f, gait, robot_config)

        if save:
            plt.savefig(f"videos/{plot_mode.value}/{k}.png")

    ani = FuncAnimation(
        fig, update, frames=config.N + 1, repeat=False, interval=config.delta
    )
    plt.show()
