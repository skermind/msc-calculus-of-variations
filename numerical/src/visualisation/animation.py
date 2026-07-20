import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def animate_paths(path_history, terrain):

    fig, ax = plt.subplots(figsize=(8, 8))

    ax.imshow(
        terrain.grid,
        extent=[
            0,
            terrain.width,
            0,
            terrain.height
        ],
        origin="lower"
    )

    line, = ax.plot([], [], linewidth=2)

    start, = ax.plot([], [], "go")
    end, = ax.plot([], [], "ro")

    def update(frame):

        path = path_history[frame]

        x = [p[0] for p in path.points]
        y = [p[1] for p in path.points]

        line.set_data(x, y)

        start.set_data(
            [x[0]],
            [y[0]]
        )

        end.set_data(
            [x[-1]],
            [y[-1]]
        )

        ax.set_title(
            f"Iteration {frame}"
        )

        return line, start, end


    animation = FuncAnimation(
        fig,
        update,
        frames=len(path_history),
        interval=100,
        blit=True
    )

    return animation