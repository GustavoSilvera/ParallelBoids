import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.ticker import StrMethodFormatter

mpl.use('Agg')

# output directory
results = "py_out/"


class plot():
    def __init__(self, x_vals, y_vals, title, x_label, y_label, axes, annotations):
        self.x_vals = np.array(x_vals)
        self.y_vals = np.array(y_vals)
        self.title = title
        self.x_label = x_label
        self.y_label = y_label
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
        plt.plot(self.x_vals, self.y_vals,
                 color='r', linewidth=1)  # red line
        plt.plot(self.x_vals, self.y_vals, 'bo')  # blue circle

        if self.annotations:
            # plot y values at points
            for i, j in zip(self.x_vals, self.y_vals):
                plt.annotate(str(j) + 's', xy=(i + 0.5, j+1), fontsize=13)

        # complete the layout, save figure, and show the figure for you to see
        plt.tight_layout()
        if (not os.path.exists(os.path.join(os.getcwd(), results))):
            os.makedirs(results)
        fig.savefig(os.path.join(results,
                                 self.title.replace(" ", "") + '.png'))
        # close figure
        plt.clf()


if __name__ == '__main__':

    v11 = plot(x_vals=[10, 20, 30, 40, 50, 60, 70, 80, 90, 100],
               y_vals=[7.67, 6.23, 5.75, 5.74, 5.57,
                       5.48, 5.54, 5.56, 5.81, 6.00],
               title="10000 Boids (4t) Granular Max Flocks",
               x_label="Maximum Flock Size",
               y_label="Time(s)",
               axes=False,
               annotations=False)

    v12 = plot(x_vals=[10, 20, 30, 40, 50, 60, 70, 80, 90, 100],
               y_vals=[2.589, 2.24, 2.07, 2.033,
                       1.95, 1.96, 1.949, 2.05, 2.08, 2.148],
               title="10000 Boids (16t) Granular Max Flocks",
               x_label="Maximum Flock Size",
               y_label="Time(s)",
               axes=False,
               annotations=False)

    v13 = plot(x_vals=[1000, 2000, 5000, 10000, 20000, 30000],
               y_vals=[0.0987, 0.1943, 0.999, 2.124, 8.776, 23.914],
               title="Effect of NumBoids",
               x_label="Number of Boids",
               y_label="Time (s)",
               axes=False,
               annotations=False)

    views = [v11, v12, v13]
    for v in views:
        v.plot_graph()
