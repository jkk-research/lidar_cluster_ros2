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



# define a function to calculate the distance between two points
def distance(p1, p2):
    return np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

# define a function to find the neighbors and label it as a core or non-core point
def find_neighbors(points, eps):
    for p in points:
        for q in points:
            if distance(p, q) <= eps:
                p.neighbor_pts += 1
        if p.neighbor_pts >= 3: # if the number of neighbors is greater than 3, it is a core point
            p.core = True
        else:
            p.core = False
    return points


# define a function to find the clusters
def find_clusters(points, eps):
    actual_cluster_id = 0
    # grow core points
    for i in range(10): # number of clusters
        # select first core point
        for p in points:
            if p.core == True and p.cluster_id == -1:
                p.cluster_id = actual_cluster_id    
                break
        for p in points: 
            if p.cluster_id == actual_cluster_id: # every point in the actual cluster
                for q in points:
                    if q.core == True and q.cluster_id == -1:
                        if distance(p, q) <= eps:
                            q.cluster_id = actual_cluster_id
                            # print("new cluster point in cluster %d [%.1f, %.1f]" % (actual_cluster_id, q.x, q.y))
        actual_cluster_id += 1
    # grow non-core points    
    for p in points:
        if p.core == False:
            for q in points:
                if q.core == True and q.cluster_id != -1:
                    if distance(p, q) <= eps:
                        p.cluster_id = q.cluster_id
                        # print("new cluster point (non-core) cluster ID: %d [%.1f, %.1f]" % (q.cluster_id, p.x, p.y))
    return points



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
    x = [p.x for p in points]
    y = [p.y for p in points]
    core = [p.core for p in points]
    cluster_id = [p.cluster_id for p in points]
    neighbor_pts = [p.neighbor_pts ** 2 * 20 for p in points]
    cmap1 = plt.get_cmap('summer', np.max(core) + 1)
    cmap2 = plt.get_cmap('turbo', lut=(np.max(cluster_id) + 2)) # 'CMRmap_r' 'viridis'
    if plot_neighbor_and_core == True:
        plt.scatter(x, y, s=neighbor_pts, c=core, alpha=0.5, cmap=cmap1) ## plot the neighbor points and core points
        plt.colorbar(shrink=0.4)
    plt.scatter(x, y, s=50, c=cluster_id, alpha=0.8, cmap=cmap2, vmin=-1, vmax=(np.max(cluster_id)+1)) ## plot the clusters 
    plt.colorbar(cax=None, ax=None, shrink=0.4)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('DBSCAN Demo')
    plt.axis('equal')
    plt.grid()
    plt.show()

# print the data
def print_data(points):
    print("--------------------")
    for p in points:
        print("[%4.1f,%4.1f] %r %d %d" % (p.x, p.y, p.core, p.cluster_id, p.neighbor_pts))

# main function
if __name__ == '__main__':
    points = read_data('notebooks/data/test03.csv')
    points = find_neighbors(points, 3.0)
    points = find_clusters(points, 3.0)
    print_data(points)
    plot_data(points)




