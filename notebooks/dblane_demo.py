# The DBlane - A Lane variant of the Density-Based Spatial Clustering of Applications with Noise clustering algorithm

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

class Cluster:
    def __init__(self, points, candidate_points, cluster_id):
        self.points = []
        self.candidate_points = candidate_points
        self.cluster_id = cluster_id
        self.a_head = 0.0
        self.a_tail = 0.0
    def add_back(self, point, cluster_id):
        self.points.append(point)
        self.cluster_id = cluster_id
        self.calculate_head_tail_angles()
    def add_front(self, point, cluster_id):
        self.points.insert(0, point)
        self.cluster_id = cluster_id
        self.calculate_head_tail_angles()
    def calculate_head_tail_angles(self):
        # if at least two points in the cluster, calculate the slope of the first and last points
        if len(self.points) >= 2:
            first1 = self.points[1]
            first2 = self.points[0]
            last1 = self.points[-2]
            last2 = self.points[-1]
            # calculate the angle of the first and last points
            self.a_head = self.calculate_angle(first1, first2)
            self.a_tail = self.calculate_angle(last1, last2) 
        # if zero or one point in the cluster, set the slope to 0
        else:
            self.a_head = 0.0
            self.a_tail = 0.0

    def set_cluster_points(self, points, cluster_id):
        self.points = points
        self.cluster_id = cluster_id
        self.calculate_head_tail_angles()

    def get_tail(self):
        if len(self.points) < 2:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        x_end_t = 1.0 * np.cos(self.a_tail) + self.points[-1].x 
        y_end_t = 1.0 * np.sin(self.a_tail) + self.points[-1].y        
        return self.points[-1].x, self.points[-1].y, x_end_t, y_end_t, self.a_tail
    def get_head(self): 
        if len(self.points) < 2:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        x_end_h = 1.0 * np.cos(self.a_head) + self.points[0].x
        y_end_h = 1.0 * np.sin(self.a_head) + self.points[0].y
        return self.points[0].x, self.points[0].y, x_end_h, y_end_h, self.a_head
    def calculate_angle(self, point1, point2):
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        rad = np.arctan2(dy, dx)
        return rad
    def print(self):
        print("Cluster %d:" % self.cluster_id)
        for p in self.points:
            print("[%4.1f,%4.1f] %r %d %d" % (p.x, p.y, p.core, p.cluster_id, p.neighbor_pts))
        print("a_head: %f" % self.a_head)
        print("a_tail: %f" % self.a_tail)
    def get_size(self):
        return len(self.points)
    def next_tail(self, eps_min, eps_max):
        print("next", self.points[-1].x, self.points[-1].y)
        tail_x = self.points[-1].x
        tail_y = self.points[-1].y
        p = Point(tail_x, tail_y)
        for q in self.candidate_points: 
            if q.core == True and q.cluster_id == -1:
                if eps_min <= distance(p, q) <= eps_max:
                    q.cluster_id = self.cluster_id
                    print("Found", q.x, q.y)
                    # if my_cluster.get_size() >= 2:
                    #     # check if orientation of the cluster tail / head is close to the new point candidate
                    #     head_start_x, head_start_y, _, _, head_angle = my_cluster.get_head()
                    #     tail_start_x, tail_start_y, _, _, tail_angle = my_cluster.get_tail()

                    #     # calculate the angle between the new point and the head of the cluster
                    #     actual_head_angle = my_cluster.calculate_angle(my_cluster.points[0], q)
                    #     actual_tail_angle = my_cluster.calculate_angle(my_cluster.points[-1], q)

                    #     # calculate the difference between the actual angle and the cluster angle
                    #     diff_head = angle_diff(actual_head_angle - head_angle)
                    #     diff_tail = angle_diff(actual_tail_angle - tail_angle)


                    self.add_back(q, self.cluster_id)
                    # print("new cluster point in cluster %d [%.1f, %.1f]" % (actual_cluster_id, q.x, q.y))


# Global variables for animation TODO: better solution
fig, ax = plt.subplots()
cluster_plot = ax.plot(0.0, 0.0, 'r.-', label='cluster_plot', alpha=0.4)[0]
cluster_plot.set_linewidth(4) # increase width of the line
tail_plot = ax.plot(0.0, 0.0, 'y-', label='tail_plot', alpha=0.4)[0]
tail_plot.set_linewidth(4) # increase width of the line
head_plot = ax.plot(0.0, 0.0, 'b-', label='head_plot', alpha=0.4)[0]
head_plot.set_linewidth(4) # increase width of the line
my_cluster = Cluster(points=[], candidate_points=[], cluster_id=0) ## TODO: better solution candidate_points=[] will not be empty usually
points = []

# define a function to calculate the distance between two points
def distance(p1, p2):
    return np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

# calculate the difference between two angles
def angle_diff(res):
  res = res % np.pi
  if res >= np.pi/2:
    return np.abs(np.pi-res)
  else:
    return np.abs(res)

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
def find_clusters(points, eps_min, eps_max):
    global my_cluster
    actual_cluster_id = 0
    for i in range(2): # number of clusters
        # select first core point
        for p in points:
            if p.core == True and p.cluster_id == -1:
                p.cluster_id = actual_cluster_id
                break
        for p in points: 
            if p.cluster_id == actual_cluster_id: # every point in the actual cluster
                for q in points:
                    if q.core == True and q.cluster_id == -1:
                        if eps_min <= distance(p, q) <= eps_max:
                            q.cluster_id = actual_cluster_id
                            if my_cluster.get_size() >= 2:
                                # check if orientation of the cluster tail / head is close to the new point candidate
                                head_start_x, head_start_y, _, _, head_angle = my_cluster.get_head()
                                tail_start_x, tail_start_y, _, _, tail_angle = my_cluster.get_tail()

                                # calculate the angle between the new point and the head of the cluster
                                actual_head_angle = my_cluster.calculate_angle(my_cluster.points[0], q)
                                actual_tail_angle = my_cluster.calculate_angle(my_cluster.points[-1], q)

                                # calculate the difference between the actual angle and the cluster angle
                                diff_head = angle_diff(actual_head_angle - head_angle)
                                diff_tail = angle_diff(actual_tail_angle - tail_angle)

                            if actual_cluster_id == 0:
                                my_cluster.add_back(q, actual_cluster_id)
                            # print("new cluster point in cluster %d [%.1f, %.1f]" % (actual_cluster_id, q.x, q.y))
        actual_cluster_id += 1

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
    plt.title('DBlane Demo')
    plt.axis('equal')
    plt.grid()

# print the data
def print_data(points):
    print("--------------------")
    for p in points:
        print("[%4.1f,%4.1f] %r %d %d" % (p.x, p.y, p.core, p.cluster_id, p.neighbor_pts))

def plot_cluster():
    global my_cluster, cluster_plot, tail_plot, head_plot

    x = [p.x for p in my_cluster.points]
    y = [p.y for p in my_cluster.points]   
    cluster_plot.set_xdata(x) 
    cluster_plot.set_ydata(y)

    head_start_x, head_start_y, head_end_x, head_end_y, head_angle = my_cluster.get_head()
    tail_start_x, tail_start_y, tail_end_x, tail_end_y, tail_angle = my_cluster.get_tail()
    head_plot.set_xdata([head_start_x, head_end_x])
    head_plot.set_ydata([head_start_y, head_end_y])
    tail_plot.set_xdata([tail_start_x, tail_end_x])
    tail_plot.set_ydata([tail_start_y, tail_end_y])

    # print("head: [%.1f, %.1f] [%.1f, %1.f]" % (head_start_x, head_start_y, head_end_x, head_end_y))
    # print("tail: [%.1f, %.1f] [%.1f, %1.f]" % (tail_start_x, tail_start_y, tail_end_x, tail_end_y))
    # my_cluster.print()
    return (cluster_plot, tail_plot, head_plot)

def plt_cluster(cluster):
    head_start_x, head_start_y, head_end_x, head_end_y, head_angle = cluster.get_head()
    tail_start_x, tail_start_y, tail_end_x, tail_end_y, tail_angle = cluster.get_tail()
    plt.plot([head_start_x, head_end_x], [head_start_y, head_end_y], label='head_plot')
    plt.plot([tail_start_x, tail_end_x], [tail_start_y, tail_end_y], label='tail_plot')
    x = [p.x for p in cluster.points]
    y = [p.y for p in cluster.points]   
    plt.plot(x,y, label='cluster_plot')
    plt.legend()


# main function
if __name__ == '__main__':
    points = read_data('notebooks/data/test01.csv')
    points = find_neighbors(points, 0.8)
    points = find_clusters(points, 0.1, 0.8)
    #print_data(points)
    plot_data(points)
    plot_cluster()
    plt.show()




