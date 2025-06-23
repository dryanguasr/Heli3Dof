from __future__ import annotations

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3D


def plot_heli3dof(pos: dict, ax: plt.Axes | None = None) -> plt.Axes:
    """Plot the configuration of the 3-DOF helicopter."""
    if ax is None:
        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111, projection="3d")
    else:
        fig = ax.figure

    ax.cla()
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("Helicopter 3-DOF")
    ax.grid(True)

    # blue post
    ax.plot([0, 0], [0, 0], [0, pos["base"][2] / 0.75], "b-", linewidth=4)

    # green beam
    ax.plot([pos["cp"][0], pos["cm_A"][0]],
            [pos["cp"][1], pos["cm_A"][1]],
            [pos["cp"][2], pos["cm_A"][2]],
            "g-", linewidth=3)

    # red arm
    ax.plot([pos["motor_L"][0], pos["motor_R"][0]],
            [pos["motor_L"][1], pos["motor_R"][1]],
            [pos["motor_L"][2], pos["motor_R"][2]],
            "r-", linewidth=3)

    # highlight key points
    ax.scatter(pos["base"][0], pos["base"][2] / 0.75, pos["base"][2] / 0.75, color="b")
    ax.scatter(pos["cp"][0], pos["cp"][1], pos["cp"][2], color="g")
    ax.scatter(pos["cm_A"][0], pos["cm_A"][1], pos["cm_A"][2], color="g")
    ax.scatter(pos["motor_L"][0], pos["motor_L"][1], pos["motor_L"][2], color="r")
    ax.scatter(pos["motor_R"][0], pos["motor_R"][1], pos["motor_R"][2], color="r")

    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, pos["base"][2] / 0.75])

    plt.draw()
    return ax
