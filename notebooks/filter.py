# The DBSCAN (Density-Based Spatial Clustering of Applications with Noise) algorithm is a popular clustering algorithm in machine learning. 

import numpy as np
import matplotlib.pyplot as plt
import os


# define a class of points (x, y, label)
class Point:
    def __init__(self, x, y, core=False, cluster_id=-1, neighbor_pts=0):
        self.x = x
        self.y = y
        self.core = core # core point (True) or non-core point (False)
        self.cluster_id = cluster_id # cluster ID  
        self.neighbor_pts = neighbor_pts # neighbor points



# read the data
def read_data(filename = 'notebooks/data/test01.csv'):
    # print("Working dir:", os.getcwd()) ## print output to the console
    # read the data
    data = np.loadtxt(filename, delimiter=',', skiprows=1)
    points = []
    for i in range(len(data)):
        points.append(Point(data[i][0], data[i][1]))
        points[i].cluster_id = -1
    return points

# plot the data
def plot_data(points, plot_neighbor_and_core=False):
    # filter with a list comprehension if -25 < p.x < +2 in p.x
    x = [p.x for p in points if -25 < p.x < +2 and -30 < p.y < +16]
    y = [p.y for p in points if -25 < p.x < +2 and -30 < p.y < +16]
    plt.scatter(x, y, s=10, alpha=0.8) ## plot the clusters 
    plt.colorbar(cax=None, ax=None, shrink=0.4)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.axis('equal')
    plt.grid()
    plt.show()
    for i in range(len(x)):
        print("%4.1f,%4.1f" % (x[i], y[i]))
    print(len(x))

# main function
if __name__ == '__main__':
    points = read_data('notebooks/data/test02.csv')
    plot_data(points)




