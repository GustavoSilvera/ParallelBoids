import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.ticker import StrMethodFormatter

mpl.use('Agg')

# output directory
results = "py_out/"


class data():
    def __init__(self, y_vals: list, title: str, machine: str):
        self.y_vals = np.array(y_vals)
        self.y_speedup = self.y_vals[0] / self.y_vals
        self.title = title
        self.machine = machine


def plot_graph(data):
    # create a figure that is 6in x 6in
    fig = plt.figure(figsize=(7, 6))

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
    plt.close()

    """PLOT SPEEDUP"""

    # create a figure that is 6in x 6in
    fig = plt.figure(figsize=(7, 6))

    # the threads are all integers
    x_vals = x_vals  # num procs

    # the axis limits and grid lines
    plt.grid(True)
    plt.xlim(0, x_vals[-1] + 0.5)  # speedup range should be in [0, 2]

    # label your graph, axes, and ticks on each axis
    plt.xlabel('Number of Processors', fontsize=16)
    plt.ylabel('Speedup', fontsize=16)
    plt.xticks(x_vals)
    plt.yticks()
    plt.tick_params(labelsize=15)
    plt.title('Speedup: ' + data.title + ' on ' + data.machine, fontsize=18)

    # plot the data values, lines and points
    plt.plot(x_vals, data.y_speedup, color='r', linewidth=1)  # red line
    plt.plot(x_vals, data.y_speedup, 'bo')  # blue circle
    # 1 decimal on x
    plt.gca().xaxis.set_major_formatter(StrMethodFormatter('{x:,d}'))

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
    # sequential (no-omp) baseline
    # seq for 10k, 20k, 2k, 10knB
    seq = {}
    seq['10k'] = 13.094
    seq['20k'] = 59.15
    seq['2k'] = 1.119
    seq['10knf'] = 100

    all_data = []

    # all_data.append(data([seq['10k'], 13.6352, 7.1271, 6.291, 3.762, 3.743, 4.236, 3.231],
    #                      "10000 Parallel Boids", "Global"))
    # all_data.append(data([seq['10k'], 7.758, 4.105, 3.028, 2.541, 2.489, 3.525, 3.798],
    #                      "10000 Parallel Boids", "Local"))
    # all_data.append(data([seq['10k'], 12.102, 6.29, 5.224, 4.267, 4.097, 5.233, 4.289],
    #                      "10000 Parallel Flocks", "Global"))
    # all_data.append(data([seq['10k'], 7.791, 4.177, 3.239, 2.411, 2.388, 3.261, 2.733],
    #                      "10000 Parallel Flocks", "Local"))

    # all_data.append(data([seq['20k'], 133.358, 65.0323, 37.2944, 31.294, 40.760, 60.647, 27.500],
    #                      "20000 Parallel Boids", "Global"))
    # all_data.append(data([seq['20k'], 47.301, 30.369, 14.891, 19.957, 21.712, 23.472, 15.923],
    #                      "20000 Parallel Boids", "Local"))
    # all_data.append(data([seq['20k'], 112.315, 90.895, 41.44, 61.215, 63.254, 84.46, 42.05],
    #                      "20000 Parallel Flocks", "Global"))
    # all_data.append(data([seq['20k'], 48.185, 33.051, 15.558, 22.396, 26.282, 26.665, 16.681],
    #                      "20000 Parallel Flocks", "Local"))

    # all_data.append(data([seq['2k'], 0.807, 0.4468, 0.2602, 0.2259, 0.1742, 0.2622, 0.309],
    #                      "2000 Parallel Boids", "Global"))
    # all_data.append(data([seq['2k'], 0.6351, 0.375, 0.3074, 0.3559, 0.4089, 0.9834, 1.0691],
    #                      "2000 Parallel Boids", "Local"))
    # all_data.append(data([seq['2k'], 0.789, 0.4334, 0.259, 0.2503, 0.2003, 0.476, 0.307],
    #                      "2000 Parallel Flocks", "Global"))
    # all_data.append(data([seq['2k'], 0.673, 0.369, 0.2259, 0.2101, 0.1614, 0.517, 0.2468],
    #                      "2000 Parallel Flocks", "Local"))

    # all_data.append(data([seq['10knf'], 64.214, 32.084, 16.0895, 11.143, 8.607, 8.907, 8.127],
    #                      "10000 Parallel Boids (No Flocks)", "Global"))
    # all_data.append(data([seq['10knf'], 59.896, 31.892, 18.057, 13.969, 14.798, 18.368, 23.332],
    #                      "10000 Parallel Boids (No Flocks)", "Local"))
    # all_data.append(data([seq['10knf'], 64.305, 32.518, 16.57, 11.231, 8.521, 8.576, 8.179],
    #                      "10000 Parallel Flocks (No Flocks)", "Global"))
    # all_data.append(data([seq['10knf'], 59.563, 30.391, 15.907, 10.4889, 8.286, 7.505, 7.943],
    #                      "10000 Parallel Flocks (No Flocks)", "Local"))

    # all_data.append(data([seq['10k'], 12.4488, 7.058, 5.298, 4.426, 3.778, 5.68, 3.58],
    #                      "10000 Parallel Flocks (max10)", "Global"))

    # all_data.append(data([seq['10k'], 12.039, 7.014, 4.023, 3.372, 2.822, 3.118, 2.92],
    #                      "10000 Parallel Boids (dynamic)", "Global"))
    # all_data.append(data([seq['10k'], 7.544, 4.431, 2.625, 2.251, 2.193, 4.150, 3.292],
    #                      "10000 Parallel Boids (dynamic)", "Local"))
    # all_data.append(data([seq['10k'], 10.234, 6.058, 3.915, 3.806, 3.139, 4.380, 3.407],
    #                      "10000 Parallel Flocks (dynamic)", "Global"))
    # all_data.append(data([seq['10k'], 7.158, 4.005, 2.309, 2.005, 1.789, 2.233, 1.970],
    #                      "10000 Parallel Flocks (dynamic)", "Local"))

    all_data.append(data([seq['20k'], 65.49, 38.71, 19.35, 14.47, 12.38, 12.93, 11.64],
                         "20000 Parallel Boids", "Global"))
    all_data.append(data([seq['20k'], 32.16, 18.32, 9.88, 7.52, 6.32, 7.54, 7.13],
                         "20000 Parallel Boids", "Local"))
    all_data.append(data([seq['20k'], 54.46, 34.55, 17.37, 14.48, 11.88, 15.78, 11.74],
                         "20000 Parallel Flocks", "Global"))
    all_data.append(data([seq['20k'], 30.86, 17.78, 9.546, 7.500, 5.83, 6.678, 5.47],
                         "20000 Parallel Flocks", "Local"))

    all_data.append(data([seq['20k'], 70.270, 38.454, 21.613, 20.483, 16.857, 14.953, 13.911],
                         "20000 (static) Parallel Boids", "Global"))
    all_data.append(data([seq['20k'], 34.220, 19.684, 12.232, 10.352, 8.7766, 9.965, 9.139],
                         "20000 (static) Parallel Boids", "Local"))
    all_data.append(data([seq['20k'], 55.312, 36.703, 20.03, 22.773, 17.015, 14.585, 15.745],
                         "20000 (static) Parallel Flocks", "Global"))
    all_data.append(data([seq['20k'], 34.944, 20.067, 12.721, 10.627, 8.70, 8.856, 8.144],
                         "20000 (static) Parallel Flocks", "Local"))

    for v in all_data:
        plot_graph(v)
