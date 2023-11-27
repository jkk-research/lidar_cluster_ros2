# The DBlane - A Lane variant of the Density-Based Spatial Clustering of Applications with Noise clustering algorithm

import numpy as np
import matplotlib.pyplot as plt

# define a class of points (x, y, label)
class Point:
    def __init__(self, x, y, core=False, cluster_id=-1, neighbor_pts=0):
        self.x = x
        self.y = y
        self.core = core # core point (True) or non-core point (False)
        self.cluster_id = cluster_id # cluster ID  
        self.neighbor_pts = neighbor_pts # neighbor points

# define a class of clusters
class Cluster:
    """A class to represent a cluster.
    
    Attributes
    ----------
    candidate_points : list
        list of candidate points (all points in the data set)
    cluster_points : list of lists
        actual list of points in the clusters, cluster_points[N] is a list of points in cluster N
    cluster_num : int
        number of cluster ID's
    eps_min : float
        minimum distance for the next point to be added to the cluster
    eps_max : float
        maximum distance for the next point to be added to the cluster
    ang_threshold_deg : float
        maximum angle difference (in degrees) between the current cluster direction and the next point to be added to the cluster
    """
    def __init__(self, candidate_points, cluster_num=5, eps_min=0.4, eps_max=4.0, ang_threshold_deg=25.0):
        self.candidate_points = candidate_points
        self.cluster_points = []
        self.cluster_num = cluster_num
        for _ in range(self.cluster_num): self.cluster_points.append([]) # initialize the cluster points
        self.eps_min = eps_min
        self.eps_max = eps_max
        self.ang_threshold = np.deg2rad(ang_threshold_deg)
        self.actual_cluster_num = 0
        self.head_angle = []
        self.tail_angle = []
        for _ in range(self.cluster_num): self.head_angle.append(0.0)
        for _ in range(self.cluster_num): self.tail_angle.append(0.0)
        self.recalculate_head_tail_angles()
        self.find_neighbors()

    def add_back(self, point, cluster_id):
        self.cluster_points[cluster_id].append(point)
        self.recalculate_head_tail_angles()
    def add_front(self, point, cluster_id):
        self.cluster_points[cluster_id].insert(0, point)
        self.recalculate_head_tail_angles()
    def recalculate_head_tail_angles(self):
        # if at least two points in the cluster, calculate the slope of the first and last points
        for i in range(self.cluster_num):
            if len(self.cluster_points[i]) >= 2:
                first1 = self.cluster_points[i][1]
                first2 = self.cluster_points[i][0]
                last1 = self.cluster_points[i][-2]
                last2 = self.cluster_points[i][-1]
                # calculate the angle of the first and last points
                self.head_angle[i] = self.calculate_angle(first1, first2)
                self.tail_angle[i] = self.calculate_angle(last1, last2) 
            # if zero or one point in the cluster, set the slope to 0
            else:
                self.head_angle[i] = 0.0
                self.tail_angle[i] = 0.0
    # find the neighbors and label it as a core or non-core point
    def find_neighbors(self):
        for p in self.candidate_points:
            for q in self.candidate_points:
                if distance(p, q) <= self.eps_max:
                    p.neighbor_pts += 1
            if p.neighbor_pts >= 3: # if the number of neighbors is greater than 3, it is a core point
                p.core = True
            else:
                p.core = False

    def set_cluster_points(self, points, cluster_id):
        self.cluster_points[cluster_id] = points
        self.recalculate_head_tail_angles()

    def get_tail(self, id):
        self.recalculate_head_tail_angles()
        if len(self.cluster_points[id]) < 2:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        x_end_t = 1.0 * np.cos(self.tail_angle[id]) + self.cluster_points[id][-1].x 
        y_end_t = 1.0 * np.sin(self.tail_angle[id]) + self.cluster_points[id][-1].y     
        print("tail: [%.1f, %.1f] [%.1f, %1.f]" % (self.cluster_points[id][-1].x, self.cluster_points[id][-1].y, x_end_t, y_end_t))   
        return self.cluster_points[id][-1].x, self.cluster_points[id][-1].y, x_end_t, y_end_t, self.tail_angle[id]
    def get_head(self, id): 
        self.recalculate_head_tail_angles()
        if len(self.cluster_points[id]) < 2:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        x_end_h = 1.0 * np.cos(self.head_angle[id]) + self.cluster_points[id][0].x
        y_end_h = 1.0 * np.sin(self.head_angle[id]) + self.cluster_points[id][0].y
        print("head: [%.1f, %.1f] [%.1f, %1.f]" % (self.cluster_points[id][0].x, self.cluster_points[id][0].y, x_end_h, y_end_h))   
        return self.cluster_points[id][0].x, self.cluster_points[id][0].y, x_end_h, y_end_h, self.head_angle[id]
    # calculate the difference between two angles
    def angle_diff(self, a, b):
        while a < 0.0:
            a += np.pi * 2
        while a > np.pi * 2:
            a -= np.pi * 2        
        while b < 0.0:
            b += np.pi * 2
        while b > np.pi * 2:
            b -= np.pi * 2       
        if a > b:
            greater = a
            smaller = b
        else:
            greater = b
            smaller = a
        res = greater - smaller
        return res
    def calculate_angle(self, point1, point2):
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        rad = np.arctan2(dy, dx)
        return rad
    def print_cluster(self, id):
        print("Cluster %d:" % self.cluster_num)
        for p in self.cluster_points[id]:
            print("[%4.1f,%4.1f] %r %d %d" % (p.x, p.y, p.core, p.cluster_id, p.neighbor_pts))
        print("a_head: %f" % self.head_angle[id])
        print("a_tail: %f" % self.tail_angle[id])
    def get_size(self, id):
        # for p in self.cluster_points[id]:
        #     print("[%.1f %1.f]" % (p.x, p.y), end=", ")
        return len(self.cluster_points[id])
    def next_tail(self, id):
        # print("next", self.cluster_points[-1].x, self.cluster_points[-1].y)
        tail_x = self.cluster_points[id][-1].x
        tail_y = self.cluster_points[id][-1].y
        p = Point(tail_x, tail_y)
        for q in self.candidate_points: 
            if q.core == True and q.cluster_id == -1:
                if self.eps_min <= distance(p, q) <= self.eps_max:
                    q.cluster_id = id
                    candidate_angle = self.calculate_angle(p, q)
                    angle_difference = self.angle_diff(candidate_angle, self.tail_angle[id])
                    if angle_difference < self.ang_threshold:
                        # print("Found", q.x, q.y)
                        # print("candidate_angle: %.1f deg, self.tail_angle %.1f deg" % (np.rad2deg(candidate_angle), np.rad2deg(self.tail_angle)))
                        self.add_back(q, id)
                        self.recalculate_head_tail_angles()
                        break
                    # print("new cluster point in cluster [%.1f, %.1f]" % (q.x, q.y))
                    self.recalculate_head_tail_angles()


# define a function to calculate the distance between two points
def distance(p1, p2):
    return np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)




# read the data
def read_data(filename = 'data/test01.csv'):
    # print("Working dir:", os.getcwd()) ## print output to the console
    # read the data
    data = np.loadtxt(filename, delimiter=',', skiprows=1)
    points = []
    for i in range(len(data)):
        points.append(Point(data[i][0], data[i][1]))
        points[i].cluster_id = -1
    return points

# plot the data
def plot_data(points, plot_neighbor_and_core=False, fig = None, ax = None):
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
    plt.scatter(x, y, s=50, c=cluster_id, alpha=0.2, cmap=cmap2, vmin=-1, vmax=(np.max(cluster_id)+1)) ## plot the clusters 
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
    # list comprehension
    x = [p.x for p in my_cluster.cluster_points[0]]
    y = [p.y for p in my_cluster.cluster_points[0]]   
    cluster_plot.set_xdata(x) 
    cluster_plot.set_ydata(y)

    head_start_x, head_start_y, head_end_x, head_end_y, head_angle = my_cluster.get_head(0)
    tail_start_x, tail_start_y, tail_end_x, tail_end_y, tail_angle = my_cluster.get_tail(0)
    head_plot.set_xdata([head_start_x, head_end_x])
    head_plot.set_ydata([head_start_y, head_end_y])
    tail_plot.set_xdata([tail_start_x, tail_end_x])
    tail_plot.set_ydata([tail_start_y, tail_end_y])

    # print("head: [%.1f, %.1f] [%.1f, %1.f]" % (head_start_x, head_start_y, head_end_x, head_end_y))
    # print("tail: [%.1f, %.1f] [%.1f, %1.f]" % (tail_start_x, tail_start_y, tail_end_x, tail_end_y))
    # my_cluster.print()
    return (cluster_plot, tail_plot, head_plot)

def plt_cluster(cluster, fig = None, ax = None):
    head_start_x, head_start_y, head_end_x, head_end_y, head_angle = cluster.get_head(0)
    tail_start_x, tail_start_y, tail_end_x, tail_end_y, tail_angle = cluster.get_tail(0)
    plt.plot([head_start_x, head_end_x], [head_start_y, head_end_y], 'b*-', label='head_plot', alpha=0.4, linewidth=4.2)
    plt.plot([tail_start_x, tail_end_x], [tail_start_y, tail_end_y], 'y*-', label='tail_plot', alpha=0.4, linewidth=4.2)
    x = [p.x for p in cluster.cluster_points[0]]
    y = [p.y for p in cluster.cluster_points[0]]   
    plt.plot(x,y, 'r.-', label='cluster_plot', alpha=0.4, linewidth=4.2)
    if fig == None:
        plt.legend()
    else:
        fig.legend()


# main function
if __name__ == '__main__':
    print("DBlane main function")




