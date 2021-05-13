import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.ticker import StrMethodFormatter

mpl.use('Agg')

# output directory
results = "py_out/"


class plot():
    def __init__(self, x_vals, y_vals, title, x_label, y_label):
        self.x_vals = np.array(x_vals)
        self.y_vals = np.array(y_vals)
        self.title = title
        self.x_label = x_label
        self.y_label = y_label

        def plot_graph():
            # create a figure that is 6in x 6in
            fig = plt.figure(figsize=(6, 6))

            # the axis limits and grid lines
            plt.grid(True)
            plt.xlim(0, self.x_vals[-1])

            # label your graph, axes, and ticks on each axis
            plt.xlabel(self.x_label, fontsize=16)
            plt.ylabel(self.y_label, fontsize=16)
            plt.xticks(self.x_vals)
            plt.yticks()
            plt.tick_params(labelsize=15)
            plt.title(self.title, fontsize=18)

            # plot the data values, lines and points
            plt.plot(self.x_vals, self.y_vals,
                     color='r', linewidth=1)  # red line
            plt.plot(self.x_vals, self.y_vals, 'bo')  # blue circle

            # plot y values at points
            for i, j in zip(x_vals, self.y_vals):
                plt.annotate(str(j) + 's', xy=(i + 0.5, j), fontsize=13)

            # complete the layout, save figure, and show the figure for you to see
            plt.tight_layout()
            if (not os.path.exists(os.path.join(os.getcwd(), results))):
                os.makedirs(results)
            fig.savefig(os.path.join(results,
                                     self.title.replace(" ", "") + '.png'))
            # close figure
            plt.clear()
            plt.clf()


if __name__ == '__main__':
    # laptop data
    v0 = plot(x_vals=np.arange(100),
              y_vals=[],
              title="Tick Timings",
              x_label="Tick Index",
              y_label="Time (s)")

    views = [v0]
    for v in views:
        v.plot()
