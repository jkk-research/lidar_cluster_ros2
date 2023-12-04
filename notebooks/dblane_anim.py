# The DBlane - A Lane variant of the Density-Based Spatial Clustering of Applications with Noise clustering algorithm

import matplotlib.pyplot as plt
import dblane_demo as dbl
import numpy as np
# import matplotlib.animation as animation # simple plt.pause() works better here, than animation

# main function
if __name__ == '__main__':

    fig, ax = plt.subplots()
    candidate_pts = dbl.read_data('notebooks/data/test03.csv')
    dbl.plot_data(candidate_pts, fig=fig, ax=ax)

    print(len(candidate_pts))
    cluster1 = dbl.Cluster(candidate_points=candidate_pts, eps_min=1.2, eps_max=8.2, ang_threshold_deg=30.0)
    # cluster1.add_back(candidate_pts[np.random.randint(len(candidate_pts))], 1)
    # cluster1.add_back(candidate_pts[np.random.randint(len(candidate_pts))], 1)
    cluster1.add_back(candidate_pts[14], 1)
    cluster1.add_back(candidate_pts[9], 1)
    # cluster1.print_cluster(1)

    dbl.plt_cluster(cluster1, fig=fig, ax=ax)
    
    for i in range(5):
        fig.clear()
        cluster1.next_tail(id=1)
        dbl.plot_data(candidate_pts, fig=fig, ax=ax)
        dbl.plt_cluster(cluster1, fig=fig, ax=ax)
        ax.set_title(f"frame {i}")
        # Note that using time.sleep does *not* work here!
        plt.pause(1.5)
    cluster1.calc_unassigned(id=1)
    # dbl.print_data(cluster1.candidate_points)
    # print("Size: ", cluster1.get_size(1))
    dbl.plot_data(candidate_pts, fig=fig, ax=ax)
    # To save the animation using Pillow as a gif
    # writer = animation.PillowWriter(fps=15, metadata=dict(artist='Me'), bitrate=1800)
    # ani.save('scatter.gif', writer=writer)

    plt.show()