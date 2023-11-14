# Generate random x, y coordinates and save them to a csv file

import numpy as np
import matplotlib.pyplot as plt
import os


# define a class of points (x, y, label)
class Point:
    def __init__(self, x, y, core=False, cluster_id=-1, neighbor_pts=0):
        self.x = x
        self.y = y

# generate random x, y coordinates
def generate_data():
    # generate random x, y coordinates
    x = np.random.uniform(-10, 15, 300)
    y = np.random.uniform(-10, 15, 300)
    points = []
    for i in range(len(x)):
        points.append(Point(x[i], y[i]))
    return points

# save the data to a csv file
def save_data(points, filename = 'notebooks/data/test03.csv'):
    print("Working dir:", os.getcwd()) ## print output to the console
    with open(filename, 'w') as f:
        f.write('x,y\n')
        for p in points:
            f.write('%f,%f\n' % (p.x, p.y))

# plot the data
def plot_data(points):
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
    points = generate_data()
    save_data(points, 'notebooks/data/test03.csv')
    plot_data(points)




