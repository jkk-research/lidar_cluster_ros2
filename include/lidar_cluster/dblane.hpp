#ifndef DBLANE_HPP_
#define DBLANE_HPP_

#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>
#include <iostream>

// Define a class of points (x, y, label)
class Point
{
public:
    double x, y;
    bool core;
    int cluster_id, neighbor_pts;

    Point(double x, double y, bool core = false, int cluster_id = -1, int neighbor_pts = 0)
        : x(x), y(y), core(core), cluster_id(cluster_id), neighbor_pts(neighbor_pts) {}
};

// Define a class of clusters
class Cluster
{
public:
    std::vector<Point> candidate_points;
    std::vector<std::vector<Point>> cluster_points;
    int cluster_num;
    double eps_min, eps_max, ang_threshold;
    int actual_cluster_num;
    std::vector<double> head_angle, tail_angle;
    bool first_run_neighbors;

    Cluster(std::vector<Point> candidate_points, int cluster_num = 5, double eps_min = 0.4, double eps_max = 4.0, double ang_threshold_deg = 25.0)
        : candidate_points(candidate_points), cluster_num(cluster_num), eps_min(eps_min), eps_max(eps_max), ang_threshold(ang_threshold_deg * M_PI / 180.0), first_run_neighbors(true)
    {
        cluster_points.resize(cluster_num);
        head_angle.resize(cluster_num, 0.0);
        tail_angle.resize(cluster_num, 0.0);
        recalculate_head_tail_angles();
        find_neighbors();
    }
    void add_back(Point &point, int cluster_id)
    {
        cluster_points[cluster_id].push_back(point);
        point.cluster_id = cluster_id;
        recalculate_head_tail_angles();
    }

    void add_back(double x, double y, int cluster_id)
    {
        Point point(x, y);
        cluster_points[cluster_id].push_back(point);
        point.cluster_id = cluster_id;
        recalculate_head_tail_angles();
    }

    void add_front(Point &point, int cluster_id)
    {
        cluster_points[cluster_id].insert(cluster_points[cluster_id].begin(), point);
        point.cluster_id = cluster_id;
        recalculate_head_tail_angles();
    }
    void recalculate_head_tail_angles()
    {
        // if at least two points in the cluster, calculate the slope of the first and last points
        for (int i = 0; i < cluster_num; i++)
        {
            if (cluster_points[i].size() >= 2)
            {
                Point first1 = cluster_points[i][1];
                Point first2 = cluster_points[i][0];
                Point last1 = cluster_points[i][cluster_points[i].size() - 2];
                Point last2 = cluster_points[i][cluster_points[i].size() - 1];
                // calculate the angle of the first and last points
                head_angle[i] = calculate_angle(first1, first2);
                tail_angle[i] = calculate_angle(last1, last2);
            }
            // if zero or one point in the cluster, set the slope to 0
            else
            {
                head_angle[i] = 0.0;
                tail_angle[i] = 0.0;
            }
        }
    }
    void find_neighbors()
    {
        if (first_run_neighbors)
        {
            first_run_neighbors = false;
            for (Point &p : candidate_points)
            {
                for (Point &q : candidate_points)
                {
                    if (distance(p, q) <= eps_max)
                    {
                        p.neighbor_pts += 1;
                    }
                }
                if (p.neighbor_pts >= 3)
                { // if the number of neighbors is greater than 3, it is a core point
                    p.core = true;
                }
                else
                {
                    p.core = false;
                }
            }
        }
    }

    bool ray_tracing(double x, double y, std::vector<double> poly_x, std::vector<double> poly_y)
    {
        int n = poly_x.size();
        bool inside = false;
        double p2x = 0.0;
        double p2y = 0.0;
        double xints = 0.0;
        double p1x = poly_x[0];
        double p1y = poly_y[0];
        for (int i = 0; i < n + 1; i++)
        {
            p2x = poly_x[i % n];
            p2y = poly_y[i % n];
            if (y > std::min(p1y, p2y))
            {
                if (y <= std::max(p1y, p2y))
                {
                    if (x <= std::max(p1x, p2x))
                    {
                        if (p1y != p2y)
                        {
                            xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x;
                        }
                        if (p1x == p2x || x <= xints)
                        {
                            inside = !inside;
                        }
                    }
                }
            }
            p1x = p2x;
            p1y = p2y;
        }
        return inside;
    }
    std::pair<std::vector<double>, std::vector<double>> extract_polygon(std::vector<Point> points, double rectangle_width)
    {
        std::vector<double> xleft;
        std::vector<double> xright;
        std::vector<double> x;
        std::vector<double> y;

        for (Point p : points)
        {
            x.push_back(p.x);
            y.push_back(p.y);
        }

        for (double i : x)
        {
            double x1 = i - rectangle_width;
            double x2 = i + rectangle_width;
            xleft.push_back(x1);
            xright.push_back(x2);
        }

        std::vector<double> x_all(xleft);
        x_all.insert(x_all.end(), xright.rbegin(), xright.rend());

        std::vector<double> y_all(y);
        y_all.insert(y_all.end(), y.rbegin(), y.rend());

        return std::make_pair(x_all, y_all);
    }

    void set_cluster_points(std::vector<Point> points, int cluster_id)
    {
        cluster_points[cluster_id] = points;
        recalculate_head_tail_angles();
    }

    std::tuple<double, double, double, double, double> get_tail(int id)
    {
        recalculate_head_tail_angles();
        if (cluster_points[id].size() < 2)
        {
            return std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0);
        }
        double x_end_t = 1.0 * cos(tail_angle[id]) + cluster_points[id].back().x;
        double y_end_t = 1.0 * sin(tail_angle[id]) + cluster_points[id].back().y;
        // std::cout << "tail: [" << cluster_points[id].back().x << ", " << cluster_points[id].back().y << "] [" << x_end_t << ", " << y_end_t << "]\n";
        return std::make_tuple(cluster_points[id].back().x, cluster_points[id].back().y, x_end_t, y_end_t, tail_angle[id]);
    }

    std::tuple<double, double, double, double, double> get_head(int id)
    {
        recalculate_head_tail_angles();
        if (cluster_points[id].size() < 2)
        {
            return std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0);
        }
        double x_end_h = 1.0 * cos(head_angle[id]) + cluster_points[id][0].x;
        double y_end_h = 1.0 * sin(head_angle[id]) + cluster_points[id][0].y;
        // std::cout << "head: [" << cluster_points[id][0].x << ", " << cluster_points[id][0].y << "] [" << x_end_h << ", " << y_end_h << "]\n";
        return std::make_tuple(cluster_points[id][0].x, cluster_points[id][0].y, x_end_h, y_end_h, head_angle[id]);
    }

    double angle_diff(double a, double b)
    {
        while (a < 0.0)
        {
            a += M_PI * 2;
        }
        while (a > M_PI * 2)
        {
            a -= M_PI * 2;
        }
        while (b < 0.0)
        {
            b += M_PI * 2;
        }
        while (b > M_PI * 2)
        {
            b -= M_PI * 2;
        }
        double greater = (a > b) ? a : b;
        double smaller = (a > b) ? b : a;
        double res = greater - smaller;
        return res;
    }
    double calculate_angle(Point point1, Point point2)
    {
        double dx = point2.x - point1.x;
        double dy = point2.y - point1.y;
        double rad = std::atan2(dy, dx);
        return rad;
    }
    void print_cluster(int id)
    {
        std::cout << "Total " << cluster_num << " clusters\n";
        std::cout << "Cluster[" << id << "] has " << cluster_points[id].size() << " points\n";
        for (Point p : cluster_points[id])
        {
            std::cout << "[" << p.x << "," << p.y << "] " << p.core << " " << p.cluster_id << " " << p.neighbor_pts << "\n";
        }
        std::cout << "a_head(" << id << "): " << head_angle[id] << "\n";
        std::cout << "a_tail(" << id << "): " << tail_angle[id] << "\n";
    }
    // get cluster size
    int get_size(int id)
    {
        return cluster_points[id].size();
    }
    // get candidate points size    
    int get_candidate_size()
    {
        return candidate_points.size();
    }
    // get cluster points
    Point get_cluster_point(int id, int index)
    {
        return cluster_points[id][index];
    }
    // get tail angle
    double get_tail_angle(int id)
    {
        return tail_angle[id];
    }
    // get head angle
    double get_head_angle(int id)
    {
        return head_angle[id];
    }

    bool next_tail(int id)
    {
        bool extending = false;
        double tail_x = cluster_points[id].back().x;
        double tail_y = cluster_points[id].back().y;
        Point p(tail_x, tail_y);
        for (Point &q : candidate_points)
        {
            if (q.cluster_id == -1) // removed temporary  q.core == true  TODO:
            {
                if (eps_min <= distance(p, q) && distance(p, q) <= eps_max)
                {
                    double candidate_angle = calculate_angle(p, q);
                    double angle_difference = angle_diff(candidate_angle, tail_angle[id]);
                    if (angle_difference < ang_threshold)
                    {
                        add_back(q, id);
                        recalculate_head_tail_angles();
                        extending = true;
                        break;
                    }
                    recalculate_head_tail_angles();
                }
            }
        }
        return extending;
    }
    double distance(Point p1, Point p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }
    void calc_unassigned(int id)
    {
        std::pair<std::vector<double>, std::vector<double>> polygon = extract_polygon(cluster_points[id], 0.5);

        for (Point &q : candidate_points)
        {
            if (ray_tracing(q.x, q.y, polygon.first, polygon.second) && q.cluster_id == -1)
            {
                q.cluster_id = 0;
            }
        }
    }

    void init_new_cluster(int id)
    {
        for (Point &p : candidate_points)
        {
            for (Point &q : candidate_points)
            {
                if (p.core == true && q.core == true && p.cluster_id == -1 && q.cluster_id == -1)
                {
                    if (eps_min <= distance(p, q) && distance(p, q) <= eps_max)
                    {
                        add_back(p, id);
                        add_back(q, id);
                        recalculate_head_tail_angles();
                        return;
                    }
                }
            }
        }
    }

    void calculate_clusters()
    {
        int current_cluster_id = 1;
        // Step: For number of clusters
        for (current_cluster_id = 1; current_cluster_id < cluster_num; ++current_cluster_id)
        { // TODO: maybe cluster_num + 1???
            bool extending = true;
            // add initial points - Step: Init new cluster
            init_new_cluster(current_cluster_id);
            while (extending == true)
            {
                // Step: Add head / tail to cluster
                extending = next_tail(current_cluster_id);
            }
        }
        // Step: calculate unassigned points and extend the unassigned values
        calc_unassigned(current_cluster_id);
    }
};

#endif // DBLANE_HPP_