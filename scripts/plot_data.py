import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.use('Agg')

# output directory
results = "py_out/"


class data():
    def __init__(self, y_vals: list, title: str, machine: str):
        self.y_vals = np.array(y_vals)
        self.y_speedup = self.y_vals[:-1] / self.y_vals[1:]
        self.title = title
        self.machine = machine


def plot_graph(data):
    # create a figure that is 6in x 6in
    fig = plt.figure(figsize=(6, 6))

    # the threads are all integers
    x_vals = [1, 2, 4, 8, 12, 16, 24, 32]  # num procs

    # the axis limits and grid lines
    plt.grid(True)
    plt.xlim(0, x_vals[-1] + 5)

    # label your graph, axes, and ticks on each axis
    plt.xlabel('Number of Processors', fontsize=16)
    plt.ylabel('Time', fontsize=16)
    plt.xticks(x_vals)
    plt.yticks()
    plt.tick_params(labelsize=15)
    plt.title('Timing: ' + data.title + ' on ' + data.machine, fontsize=18)

    # plot the data values, lines and points
    plt.plot(x_vals, data.y_vals, color='r', linewidth=1)  # red line
    plt.plot(x_vals, data.y_vals, 'bo')  # blue circle

    # plot y values at points
    for i, j in zip(x_vals, data.y_vals):
        plt.annotate(str(j) + 's', xy=(i + 0.5, j), fontsize=13)

    # complete the layout, save figure, and show the figure for you to see
    plt.tight_layout()
    if (not os.path.exists(os.path.join(os.getcwd(), results))):
        os.makedirs(results)
    fig.savefig(os.path.join(results, "Time_" + data.title.replace(" ", "") +
                             "_" + data.machine + '.png'))
    # close figure
    plt.clf()

    """PLOT SPEEDUP"""

    # create a figure that is 6in x 6in
    fig = plt.figure(figsize=(6, 6))

    # the threads are all integers
    x_vals = np.log2(x_vals[1:])  # num procs

    # the axis limits and grid lines
    plt.grid(True)
    plt.ylim(0, 2.5)  # speedup range should be in [0, 2]
    plt.xlim(0, x_vals[-1] + 0.5)  # speedup range should be in [0, 2]

    # label your graph, axes, and ticks on each axis
    plt.xlabel('Log2(Number of Processors)', fontsize=16)
    plt.ylabel('Speedup', fontsize=16)
    plt.xticks(x_vals)
    plt.yticks()
    plt.tick_params(labelsize=15)
    plt.title('Speedup: ' + data.title + ' on ' + data.machine, fontsize=18)

    # plot the data values, lines and points
    plt.plot(x_vals, data.y_speedup, color='r', linewidth=1)  # red line
    plt.plot(x_vals, data.y_speedup, 'bo')  # blue circle

    # plot y values at points
    for i, j in zip(x_vals, data.y_speedup):
        plt.annotate('%.3fx' % j, xy=(i, j + 0.1), fontsize=13)
     # complete the layout, save figure, and show the figure for you to see
    plt.tight_layout()

    if (not os.path.exists(os.path.join(os.getcwd(), results))):
        os.makedirs(results)
    fig.savefig(os.path.join(results, "Speedup_" + data.title.replace(" ", "") +
                             "_" + data.machine + '.png'))


if __name__ == '__main__':
    # laptop data
    v0 = data([24.587, 13.073, 7.481, 4.436, 3.383, 3.554, 3.283, 3.105],
              "Parallel Boids", "Global")
    v1 = data([21.525, 12.914, 7.369, 4.864, 3.842, 3.477, 4.367, 3.120],
              "Parallel Flocks", "Global")
    v2 = data([15.267, 9.063, 5.405, 3.326, 2.869, 2.699, 3.683, 3.514],
              "Parallel Boids", "Local")
    v3 = data([15.232, 9.573, 5.864, 3.570, 2.796, 2.538, 2.718, 2.308],
              "Parallel Flocks", "Local")

    views = [v0, v1, v2, v3]
    for v in views:
        plot_graph(v)
