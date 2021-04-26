import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import math

class data():
    def __init__(self, y_vals:list , title:str):
        self.y_vals = np.array(y_vals)
        self.y_diffs = np.diff(self.y_vals)
        self.title = title

def plot_graph(data):
    # create a figure that is 6in x 6in
    fig = plt.figure(figsize=(6, 6))

    # the threads are all integers
    x_vals = [1, 2, 4, 8, 12] # num procs
    
    # the axis limits and grid lines
    plt.grid(True)
    plt.xlim(0, x_vals[-1] + 5)

    # label your graph, axes, and ticks on each axis
    plt.xlabel('Number of Processors', fontsize=16)
    plt.ylabel('Time', fontsize=16)
    plt.xticks(x_vals)
    plt.yticks()
    plt.tick_params(labelsize=15)
    plt.title('Simulating ' + data.title, fontsize=18)

    # plot the data values, lines and points
    plt.plot(x_vals, data.y_vals, color='r', linewidth=1) # red line
    plt.plot(x_vals, data.y_vals, 'bo') # blue circle

    # plot y values at points
    for i,j in zip(x_vals, data.y_vals):
        plt.annotate(str(j) + 's', xy=(i + 0.5, j), fontsize=13)

    # complete the layout, save figure, and show the figure for you to see
    plt.tight_layout()
    fig.savefig('boids)' + data.title.replace(" ", "") + '.png')
    # close figure
    plt.clf()

    # create a figure that is 6in x 6in
    fig = plt.figure(figsize=(6, 6))

    # the threads are all integers
    x_vals = x_vals[:-1] # num procs
    
    # the axis limits and grid lines
    plt.grid(True)
    plt.xlim(0, x_vals[-1] + 5)

    # label your graph, axes, and ticks on each axis
    plt.xlabel('Number of Processors', fontsize=16)
    plt.ylabel('Time', fontsize=16)
    plt.xticks(x_vals)
    plt.yticks()
    plt.tick_params(labelsize=15)
    plt.title('Simulating ' + data.title, fontsize=18)

    # plot the data values, lines and points
    plt.plot(x_vals, data.y_diffs, color='r', linewidth=1) # red line
    plt.plot(x_vals, data.y_diffs, 'bo') # blue circle

    # plot y values at points
    for i,j in zip(x_vals, data.y_diffs):
        plt.annotate(str(j) + 's', xy=(i + 0.5, j), fontsize=13)
     # complete the layout, save figure, and show the figure for you to see
    plt.tight_layout()
    fig.savefig('boids)' + data.title.replace(" ", "") + '.png')


if __name__ == '__main__':
    # laptop data
    v0 = data([9.14, 5.51, 3.441, 4.517, 6.76], "ParallelBoids", "goosinator2")
    
    views = [v0]
    for v in views:
        plot_graph(v)