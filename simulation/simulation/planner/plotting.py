import os
from enum import Enum
import matplotlib.patches as patches
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.axes import Axes
from matplotlib.patches import Rectangle
from simulation.planner.plan import Plan
from simulation.utils import config, robot_config


def extract_feet_patches(f: int, plan: Plan) -> list[Rectangle]:
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
                (plan.xf[j] - sidex / 2, plan.yf[j] - sidey / 2),
                angle=plan.thetaf[j] * 180 / np.pi,
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
                (plan.xf[j] - sidex / 2, plan.yf[j] - sidey / 2),
                angle=plan.thetaf[j] * 180 / np.pi,
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


def plot_2d(ax: Axes, k: int, f: int, plan: Plan) -> None:
    ax.plot(plan.zmp_traj[:k, 0], plan.zmp_traj[:k, 1], color="g")
    ax.legend(["ZMP"])
    for foot in extract_feet_patches(f, plan):
        ax.add_patch(foot)


class PlotMode(Enum):
    TWO_D = "2d"
    THREE_D = "3d"


def animate(
    plan: Plan,
    save: bool = True,
    plot_mode: PlotMode = PlotMode.TWO_D,
) -> None:
    fig = plt.figure()

    max_x = 3
    min_x = 0
    max_y = 0.5
    min_y = -0.5

    x_pad = 0.3 * (max_x - min_x) + 0.2
    y_pad = 0.5 * (max_y - min_y) - 0.2

    if not os.path.exists(f"videos/{plot_mode.value}"):
        os.makedirs(f"videos/{plot_mode.value}")

    def update(k):
        plt.clf()
        ax = fig.add_subplot(111)
        ax.set_xlim(min_x - x_pad, max_x + x_pad)
        ax.set_ylim(min_y - y_pad, max_y + y_pad)
        ax.set_aspect("equal")

        f = 0
        time = k * config.delta
        for i in range(0, len(plan.timestamps)):
            if plan.timestamps[i] <= time:
                f += 1

        plot_2d(ax, k, f, plan)

    ani = FuncAnimation(
        fig, update, frames=config.N + 1, repeat=False, interval=config.delta
    )

    if save:
        ani.save(f"videos/{plot_mode.value}/walking.mp4", writer="ffmpeg", fps=83)
    plt.show()
