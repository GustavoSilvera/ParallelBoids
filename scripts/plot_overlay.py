from __future__ import annotations
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.patches as mpatches

mpl.use('Agg')

# output directory
results = "py_out/"


class plot():
    def __init__(self, x_vals, y_vals, title, x_label, y_label, axes, annotations, labels):
        self.x_vals = x_vals
        self.y_vals = y_vals
        self.title = title
        self.x_label = x_label
        self.y_label = y_label
        self.labels = labels
        self.axes = axes
        self.annotations = annotations

    def plot_graph(self):
        # create a figure that is 6in x 6in
        fig = plt.figure(figsize=(6, 6))

        # the axis limits and grid lines
        plt.grid(True)
        plt.xlim(0, self.x_vals[-1])

        # label your graph, axes, and ticks on each axis
        plt.xlabel(self.x_label, fontsize=16)
        plt.ylabel(self.y_label, fontsize=16)
        if (self.axes):
            plt.xticks(self.x_vals)
            plt.yticks()
        plt.tick_params(labelsize=15)
        plt.title(self.title, fontsize=18)

        # plot the data values, lines and points
        colours = ['r', 'b', 'g', 'y']
        patches = []
        for i, y in enumerate(self.y_vals):
            plt.plot(self.x_vals, y,
                     color=colours[i], linewidth=1)  # red line
            plt.plot(self.x_vals, y, colours[i] + 'o')  # blue circle
            patches.append(mpatches.Patch(
                color=colours[i], label=self.labels[i]))
        plt.rcParams["legend.fontsize"] = 16
        plt.legend(handles=patches)

        if self.annotations:
            # plot y values at points
            for i, j in zip(self.x_vals, self.y_vals):
                plt.annotate(str(j) + 's', xy=(i + 0.5, j), fontsize=13)

        # complete the layout, save figure, and show the figure for you to see
        plt.tight_layout()
        if (not os.path.exists(os.path.join(os.getcwd(), results))):
            os.makedirs(results)
        fig.savefig(os.path.join(results,
                                 self.title.replace(" ", "") + '.png'))
        # close figure
        plt.clf()


if __name__ == '__main__':
    seq = {}
    seq['10k'] = 13.094
    seq['20k'] = 59.15
    seq['2k'] = 1.119
    seq['10knf'] = 100

    # all_data.append(data([seq['20k'], 65.49, 38.71, 19.35, 14.47, 12.38, 12.93, 11.64],
    #                      "20000 Parallel Boids", "Global"))
    # all_data.append(data([seq['20k'], 32.16, 18.32, 9.88, 7.52, 6.32, 7.54, 7.13],
    #                      "20000 Parallel Boids", "Local"))
    # all_data.append(data([seq['20k'], 54.46, 34.55, 17.37, 14.48, 11.88, 15.78, 11.74],
    #                      "20000 Parallel Flocks", "Global"))
    # all_data.append(data([seq['20k'], 30.86, 17.78, 9.546, 7.500, 5.83, 6.678, 5.47],
    #                      "20000 Parallel Flocks", "Local"))

    # all_data.append(data([seq['20k'], 70.270, 38.454, 21.613, 20.483, 16.857, 14.953, 13.911],
    #                      "20000 (static) Parallel Boids", "Global"))
    # all_data.append(data([seq['20k'], 34.220, 19.684, 12.232, 10.352, 8.7766, 9.965, 9.139],
    #                      "20000 (static) Parallel Boids", "Local"))
    # all_data.append(data([seq['20k'], 55.312, 36.703, 20.03, 22.773, 17.015, 14.585, 15.745],
    #                      "20000 (static) Parallel Flocks", "Global"))
    # all_data.append(data([seq['20k'], 34.944, 20.067, 12.721, 10.627, 8.70, 8.856, 8.144],
    #  "20000 (static) Parallel Flocks", "Local"))

    # B_G = np.array([seq['20k'], 65.49, 38.71, 19.35,
    #                 14.47, 12.38, 12.93, 11.64])
    # B_G_S = np.array([seq['20k'], 70.270, 38.454, 21.613,
    #                   20.483, 16.857, 14.953, 13.911])

    # v0 = plot(y_vals=B_G,
    #           x_vals=[1, 2, 4, 8, 12, 16, 24, 32],
    #           y_vals2=B_G_S,
    #           title="Speedup Parallel Boids on Global",
    #           y_label="Time (s)",
    #           x_label="Num Procs",
    #           axes=False,
    #           annotations=False,
    #           y1_label='dynamic',
    #           y2_label='static')

    # B_L = np.array([seq['20k'], 32.16, 18.32, 9.88, 7.52, 6.32, 7.54, 7.13])
    # B_L_S = np.array([seq['20k'], 34.220, 19.684, 12.232,
    #                   10.352, 8.7766, 9.965, 9.139])
    # v1 = plot(y_vals=B_L,
    #           x_vals=[1, 2, 4, 8, 12, 16, 24, 32],
    #           y_vals2=B_L_S,
    #           title="Speedup Parallel Boids on Local",
    #           y_label="Time (s)",
    #           x_label="Num Procs",
    #           axes=False,
    #           annotations=False,
    #           y1_label='dynamic',
    #           y2_label='static')

    # F_G = np.array([seq['20k'], 54.46, 34.55, 17.37,
    #                 14.48, 11.88, 15.78, 11.74])
    # F_G_S = np.array([seq['20k'], 55.312, 36.703, 20.03,
    #                   22.773, 17.015, 14.585, 15.745])
    # v2 = plot(y_vals=F_G,
    #           x_vals=[1, 2, 4, 8, 12, 16, 24, 32],
    #           y_vals2=F_G_S,
    #           title="Speedup Parallel Flocks on Global",
    #           y_label="Time (s)",
    #           x_label="Num Procs",
    #           axes=False,
    #           annotations=False,
    #           y1_label='dynamic',
    #           y2_label='static')

    # F_L = np.array([seq['20k'], 30.86, 17.78, 9.546, 7.500, 5.83, 6.678, 5.47])
    # F_L_S = np.array([seq['20k'], 34.944, 20.067,
    #                   12.721, 10.627, 8.70, 8.856, 8.144])
    v0 = plot(y_vals=[[7.67, 6.23, 5.75, 5.74, 5.57, 5.48, 5.54, 5.56, 5.81, 6.00],
                      [4.13, 3.51, 3.20, 3.13, 3.07, 3.02, 3.04, 3.099, 3.14, 3.31],
                      [2.59, 2.24, 2.07, 2.03, 1.95, 1.96, 1.949, 2.05, 2.08, 2.15]],
              x_vals=[10, 20, 30, 40, 50, 60, 70, 80, 90, 100],
              title="10000 Boids Granular Flock Sizes",
              y_label="Time (s)",
              x_label="Maximum Flock Size",
              axes=False,
              annotations=False,
              labels=['4 Threads', '8 Threads', '16 Threads'])

    views = [v0]
    for v in views:
        v.plot_graph()
