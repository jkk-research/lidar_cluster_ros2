#!/usr/bin/env python3
"""
ROS2 script: compares the full GT bója set (nonground_odom, odom frame) with the
accumulated interpolated marker map (interpolated_marker_map_odom) after one lap.

Ground truth (GT):
    Bója positions collected from the second nonground_odom PointCloud2 frame:
    intensity=1  -> bal oldali bója (inner)
    intensity=2  -> jobb oldali bója (outer)
    Since the simulator publishes the full cone map each frame, one frame already
    contains the complete GT set.

Prediction:
  Accumulated marker points from interpolated_marker_map_odom:
    ns='parallel_left_interpolated_map_points'   -> bal detekcio
    ns='parallel_right_interpolated_map_points'  -> jobb detekcio

Evaluation (bója-alapú, NEM frame-alapú):
  TP  = GT bója amire van legalabb 1 marker a match_threshold-on belul
  FN  = GT bója amire NINCS marker a match_threshold-on belul
  FP  = Marker pont amire NINCS GT bója a match_threshold-on belul
  TN  = 0 (nincs ertelme, a "negativ" osztaly nem definialt)

Metrics:
  Precision   = TP / (TP + FP)
  Sensitivity = TP / (TP + FN)   [Recall / TPR]
  F-measure   = 2*TP / (2*TP + FP + FN)
  (Specificity, NPV, Accuracy: TN=0 miatt nem ertelmezhetok -> N/A)
"""


import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import threading
import signal
import sys
import select
import tty
import termios

# Namespaces published in interpolated_marker_map_odom
# Keep both variants for compatibility across publisher versions.
NS_LEFT_NAMES = {
    "parallel_left_interpolated_map",
    "parallel_left_interpolated_map_points",
}
NS_RIGHT_NAMES = {
    "parallel_right_interpolated_map",
    "parallel_right_interpolated_map_points",
}

# Intensity labels from lidar_cone_sim.cpp
INTENSITY_NOISE = 0
INTENSITY_LEFT  = 1   # bal oldali bója (inner cone)
INTENSITY_RIGHT = 2   # jobb oldali bója (outer cone)

def unique_rows(pts: np.ndarray) -> np.ndarray:
    """Remove exact duplicate XY rows while preserving first occurrence order."""
    if len(pts) == 0:
        return pts
    _, idx = np.unique(pts, axis=0, return_index=True)
    idx = np.sort(idx)
    return pts[idx]


def evaluate(gt_pts: np.ndarray, marker_pts: np.ndarray, threshold: float) -> dict:
    """
    Bója-alapú kiértékelés.

    TP = GT bója amire van marker a threshold-on belul
    FN = GT bója amire nincs marker
    FP = Marker pont amire nincs GT bója
    """
    def safe_div(a, b):
        return float(a) / float(b) if b > 0 else float('nan')

    n_gt = len(gt_pts)
    n_mk = len(marker_pts)

    if n_gt == 0 or n_mk == 0:
        return {
            'TP': 0, 'FP': n_mk, 'FN': n_gt,
            'Precision': float('nan'), 'Sensitivity': float('nan'), 'F-measure': float('nan'),
        }

    # Distance matrix (N_gt x N_mk)
    diff = gt_pts[:, np.newaxis, :] - marker_pts[np.newaxis, :, :]
    dist = np.sqrt(np.sum(diff ** 2, axis=2))

    gt_matched = np.min(dist, axis=1) <= threshold   # (N_gt,)
    tp = int(np.sum(gt_matched))
    fn = n_gt - tp

    mk_matched = np.min(dist, axis=0) <= threshold   # (N_mk,)
    fp = int(np.sum(~mk_matched))

    precision   = safe_div(tp, tp + fp)
    sensitivity = safe_div(tp, tp + fn)
    f_measure   = safe_div(2 * tp, 2 * tp + fp + fn)

    return {
        'TP': tp, 'FP': fp, 'FN': fn,
        'Precision':   precision,
        'Sensitivity': sensitivity,
        'F-measure':   f_measure,
    }


def transform_points(pts: np.ndarray, name: str) -> np.ndarray:
    """Apply simple axis/sign transforms to probe frame-convention mismatch."""
    if len(pts) == 0:
        return pts
    x = pts[:, 0]
    y = pts[:, 1]
    if name == 'xy':
        return np.column_stack([x, y])
    if name == 'x_ny':
        return np.column_stack([x, -y])
    if name == 'nx_y':
        return np.column_stack([-x, y])
    if name == 'nx_ny':
        return np.column_stack([-x, -y])
    if name == 'yx':
        return np.column_stack([y, x])
    if name == 'y_nx':
        return np.column_stack([y, -x])
    if name == 'ny_x':
        return np.column_stack([-y, x])
    if name == 'ny_nx':
        return np.column_stack([-y, -x])
    raise ValueError(f"Unknown transform: {name}")


def transform_edges(edges: np.ndarray, name: str) -> np.ndarray:
    """Apply the same frame-convention transform to segment endpoints."""
    if len(edges) == 0:
        return edges
    start = transform_points(edges[:, :2], name)
    end = transform_points(edges[:, 2:], name)
    return np.hstack([start, end])


def choose_best_alignment(gt_l: np.ndarray, gt_r: np.ndarray,
                          mk_l: np.ndarray, mk_r: np.ndarray,
                          threshold: float):
    """Try multiple transforms and side mappings, return the best-scoring setup."""
    transform_names = ['xy', 'x_ny', 'nx_y', 'nx_ny', 'yx', 'y_nx', 'ny_x', 'ny_nx']
    best = None

    for tname in transform_names:
        mk_l_t = transform_points(mk_l, tname)
        mk_r_t = transform_points(mk_r, tname)

        # Normal mapping
        ls = evaluate(gt_l, mk_l_t, threshold)
        rs = evaluate(gt_r, mk_r_t, threshold)
        score_tp = ls['TP'] + rs['TP']
        score_fn = ls['FN'] + rs['FN']
        score_fp = ls['FP'] + rs['FP']
        cand = {
            'transform': tname,
            'mapping': 'normal',
            'left': ls,
            'right': rs,
            'score': (score_tp, -score_fn, -score_fp),
        }
        if best is None or cand['score'] > best['score']:
            best = cand

        # Swapped mapping
        ls_sw = evaluate(gt_l, mk_r_t, threshold)
        rs_sw = evaluate(gt_r, mk_l_t, threshold)
        score_tp_sw = ls_sw['TP'] + rs_sw['TP']
        score_fn_sw = ls_sw['FN'] + rs_sw['FN']
        score_fp_sw = ls_sw['FP'] + rs_sw['FP']
        cand_sw = {
            'transform': tname,
            'mapping': 'swapped',
            'left': ls_sw,
            'right': rs_sw,
            'score': (score_tp_sw, -score_fn_sw, -score_fp_sw),
        }
        if cand_sw['score'] > best['score']:
            best = cand_sw

    return best


def matched_gt_points(gt_pts: np.ndarray, marker_pts: np.ndarray, threshold: float) -> np.ndarray:
    """Return GT points that count as TP under the current threshold."""
    if len(gt_pts) == 0 or len(marker_pts) == 0:
        return np.empty((0, 2), dtype=np.float64)
    diff = gt_pts[:, np.newaxis, :] - marker_pts[np.newaxis, :, :]
    dist = np.sqrt(np.sum(diff ** 2, axis=2))
    gt_matched = np.min(dist, axis=1) <= threshold
    return gt_pts[gt_matched]


def unmatched_gt_points(gt_pts: np.ndarray, marker_pts: np.ndarray, threshold: float) -> np.ndarray:
    """Return GT points that count as FN under the current threshold."""
    if len(gt_pts) == 0:
        return np.empty((0, 2), dtype=np.float64)
    if len(marker_pts) == 0:
        return gt_pts.copy()
    diff = gt_pts[:, np.newaxis, :] - marker_pts[np.newaxis, :, :]
    dist = np.sqrt(np.sum(diff ** 2, axis=2))
    gt_unmatched = np.min(dist, axis=1) > threshold
    return gt_pts[gt_unmatched]


def unmatched_marker_points(gt_pts: np.ndarray, marker_pts: np.ndarray, threshold: float) -> np.ndarray:
    """Return marker points that count as FP under the current threshold."""
    if len(marker_pts) == 0:
        return np.empty((0, 2), dtype=np.float64)
    if len(gt_pts) == 0:
        return marker_pts.copy()
    diff = gt_pts[:, np.newaxis, :] - marker_pts[np.newaxis, :, :]
    dist = np.sqrt(np.sum(diff ** 2, axis=2))
    marker_unmatched = np.min(dist, axis=0) > threshold
    return marker_pts[marker_unmatched]


def _safe_div(a: float, b: float) -> float:
    return float(a) / float(b) if b > 0 else float('nan')


def compute_full_metrics(tp: int, fp: int, tn: int, fn: int) -> dict:
    return {
        'TP': tp,
        'FP': fp,
        'TN': tn,
        'FN': fn,
        'Precision': _safe_div(tp, tp + fp),
        'Sensitivity': _safe_div(tp, tp + fn),
        'Specificity': _safe_div(tn, tn + fp),
        'NPV': _safe_div(tn, tn + fn),
        'F-measure': _safe_div(2 * tp, 2 * tp + fp + fn),
        'Accuracy': _safe_div(tp + tn, tp + tn + fp + fn),
    }


def evaluate_connection_metrics(tp: int, fp: int, fn: int) -> dict:
    return {
        'TP': tp,
        'FP': fp,
        'FN': fn,
        'Precision': _safe_div(tp, tp + fp),
        'Sensitivity': _safe_div(tp, tp + fn),
        'F-measure': _safe_div(2 * tp, 2 * tp + fp + fn),
    }


def build_edges(points: np.ndarray, closed_loop: bool) -> np.ndarray:
    """Build consecutive edges from ordered points as (x1,y1,x2,y2)."""
    n = len(points)
    if n < 2:
        return np.empty((0, 4), dtype=np.float64)
    edges = []
    for i in range(n - 1):
        edges.append([points[i, 0], points[i, 1], points[i + 1, 0], points[i + 1, 1]])
    if closed_loop and n >= 3:
        edges.append([points[-1, 0], points[-1, 1], points[0, 0], points[0, 1]])
    return np.array(edges, dtype=np.float64)


def segment_distance(seg_a: np.ndarray, seg_b: np.ndarray) -> float:
    """Order-invariant average endpoint distance between two segments."""
    a1 = seg_a[:2]
    a2 = seg_a[2:]
    b1 = seg_b[:2]
    b2 = seg_b[2:]
    d_direct = 0.5 * (np.linalg.norm(a1 - b1) + np.linalg.norm(a2 - b2))
    d_cross = 0.5 * (np.linalg.norm(a1 - b2) + np.linalg.norm(a2 - b1))
    return float(min(d_direct, d_cross))


def evaluate_edges(gt_points: np.ndarray, marker_edges: np.ndarray,
                   edge_threshold: float,
                   gt_closed_loop: bool = True) -> dict:
    """Connection-based TP/FP/FN evaluation using GT consecutive edges vs explicit marker edges."""
    classified = classify_edges(
        gt_points,
        marker_edges,
        edge_threshold,
        gt_closed_loop=gt_closed_loop,
    )

    return {
        'stats': evaluate_connection_metrics(
            len(classified['tp_edges']),
            len(classified['fp_edges']),
            len(classified['fn_edges']),
        ),
        'n_gt_edges': classified['n_gt_edges'],
        'n_marker_edges': classified['n_marker_edges'],
    }


def classify_edges(gt_points: np.ndarray, marker_edges: np.ndarray,
                   edge_threshold: float,
                   gt_closed_loop: bool = True) -> dict:
    """Split edges into TP/FN/FP sets using GT consecutive edges and explicit marker segments."""
    gt_edges = build_edges(gt_points, gt_closed_loop)
    mk_edges = marker_edges.copy() if len(marker_edges) > 0 else np.empty((0, 4), dtype=np.float64)

    n_gt = len(gt_edges)
    n_mk = len(mk_edges)

    if n_gt == 0 and n_mk == 0:
        return {
            'tp_edges': np.empty((0, 4), dtype=np.float64),
            'fn_edges': np.empty((0, 4), dtype=np.float64),
            'fp_edges': np.empty((0, 4), dtype=np.float64),
            'n_gt_edges': 0,
            'n_marker_edges': 0,
        }
    if n_gt == 0:
        return {
            'tp_edges': np.empty((0, 4), dtype=np.float64),
            'fn_edges': np.empty((0, 4), dtype=np.float64),
            'fp_edges': mk_edges.copy(),
            'n_gt_edges': 0,
            'n_marker_edges': n_mk,
        }
    if n_mk == 0:
        return {
            'tp_edges': np.empty((0, 4), dtype=np.float64),
            'fn_edges': gt_edges.copy(),
            'fp_edges': np.empty((0, 4), dtype=np.float64),
            'n_gt_edges': n_gt,
            'n_marker_edges': 0,
        }

    dist = np.zeros((n_gt, n_mk), dtype=np.float64)
    for i in range(n_gt):
        for j in range(n_mk):
            dist[i, j] = segment_distance(gt_edges[i], mk_edges[j])

    gt_matched = np.min(dist, axis=1) <= edge_threshold
    mk_matched = np.min(dist, axis=0) <= edge_threshold

    return {
        'tp_edges': gt_edges[gt_matched],
        'fn_edges': gt_edges[~gt_matched],
        'fp_edges': mk_edges[~mk_matched],
        'n_gt_edges': n_gt,
        'n_marker_edges': n_mk,
    }


def hybrid_evaluate(cone_pts: np.ndarray, noise_pts: np.ndarray,
                    marker_pts: np.ndarray, threshold: float) -> dict:
    """Hybrid confusion matrix using cones as positives and noise as negatives."""
    n_cone = len(cone_pts)
    n_noise = len(noise_pts)

    if len(marker_pts) == 0:
        return compute_full_metrics(0, 0, n_noise, n_cone)

    if n_cone > 0:
        diff_cone = cone_pts[:, np.newaxis, :] - marker_pts[np.newaxis, :, :]
        dist_cone = np.sqrt(np.sum(diff_cone ** 2, axis=2))
        cone_matched = np.min(dist_cone, axis=1) <= threshold
        tp = int(np.sum(cone_matched))
        fn = n_cone - tp
    else:
        tp = 0
        fn = 0

    if n_noise > 0:
        diff_noise = noise_pts[:, np.newaxis, :] - marker_pts[np.newaxis, :, :]
        dist_noise = np.sqrt(np.sum(diff_noise ** 2, axis=2))
        noise_matched = np.min(dist_noise, axis=1) <= threshold
        fp = int(np.sum(noise_matched))
        tn = n_noise - fp
    else:
        fp = 0
        tn = 0

    return compute_full_metrics(tp, fp, tn, fn)


def merge_points(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    if len(a) == 0 and len(b) == 0:
        return np.empty((0, 2), dtype=np.float64)
    if len(a) == 0:
        return b.copy()
    if len(b) == 0:
        return a.copy()
    return np.vstack([a, b])


class CompareNode(Node):
    def __init__(self):
        super().__init__('nonground_vs_marker_comparator')

        self.declare_parameter('match_threshold', 1.0)
        self.declare_parameter('edge_match_threshold', 1.5)
        self.declare_parameter('log_nonground_coords', False)
        self.declare_parameter('log_nonground_max_points', 200)

        self.match_threshold = (
            self.get_parameter('match_threshold').get_parameter_value().double_value)
        self.edge_match_threshold = (
            self.get_parameter('edge_match_threshold').get_parameter_value().double_value)
        self.log_nonground_coords = (
            self.get_parameter('log_nonground_coords').get_parameter_value().bool_value)
        self.log_nonground_max_points = max(
            1,
            self.get_parameter('log_nonground_max_points').get_parameter_value().integer_value,
        )

        self._lock = threading.Lock()

        # GT: egyedi bója pozíciók (odom frame, deduped)
        self._gt_left  = np.empty((0, 2), dtype=np.float64)  # intensity=1
        self._gt_right = np.empty((0, 2), dtype=np.float64)  # intensity=2
        self._gt_noise = np.empty((0, 2), dtype=np.float64)  # intensity=0

        # Predikció: legutolsó marker snapshot
        self._marker_left  = np.empty((0, 2), dtype=np.float64)
        self._marker_right = np.empty((0, 2), dtype=np.float64)
        self._marker_left_edges = np.empty((0, 4), dtype=np.float64)
        self._marker_right_edges = np.empty((0, 4), dtype=np.float64)

        self._ng_frames = 0
        self._gt_initialized = False

        self.create_subscription(
            PointCloud2, 'nonground_odom', self._nonground_callback, 10)
        self.create_subscription(
            MarkerArray, 'interpolated_marker_map_odom', self._marker_callback, 10)
        self._point_tp_pub = self.create_publisher(MarkerArray, '/point_true_positives', 10)
        self._point_fn_pub = self.create_publisher(MarkerArray, '/point_false_negatives', 10)
        self._point_fp_pub = self.create_publisher(MarkerArray, '/point_false_positives', 10)
        self._edge_tp_pub = self.create_publisher(MarkerArray, '/edge_true_positives', 10)
        self._edge_fn_pub = self.create_publisher(MarkerArray, '/edge_false_negatives', 10)
        self._edge_fp_pub = self.create_publisher(MarkerArray, '/edge_false_positives', 10)
        self._gt_edge_pub = self.create_publisher(MarkerArray, '/gt_edges', 10)
        self._tp_preview_active = False
        self._preview_timer = self.create_timer(0.5, self._publish_preview)
        self._gt_timer = self.create_timer(0.5, self._publish_gt_edges)

    # ------------------------------------------------------------------
    def _nonground_callback(self, msg: PointCloud2):
        try:
            gen = point_cloud2.read_points(
                msg, skip_nans=True, field_names=('x', 'y', 'intensity'))
            data = [(p[0], p[1], int(p[2])) for p in gen]
        except Exception as e:
            self.get_logger().error(f"nonground_callback error: {e}")
            return

        if not data:
            return

        arr = np.array(data, dtype=np.float64)
        pts    = arr[:, :2]
        labels = arr[:, 2].astype(int)

        left_new  = pts[labels == INTENSITY_LEFT]
        right_new = pts[labels == INTENSITY_RIGHT]
        noise_new = pts[labels == INTENSITY_NOISE]

        with self._lock:
            # The simulator publishes the full cone map each frame; capture GT from frame #2.
            if (not self._gt_initialized) and (self._ng_frames >= 1):
                self._gt_left = unique_rows(left_new)
                self._gt_right = unique_rows(right_new)
                self._gt_noise = unique_rows(noise_new)
                self._gt_initialized = True
                if self.log_nonground_coords:
                    self._log_gt_points_once()
            self._ng_frames += 1

    def _log_gt_points_once(self):
        left_limit = min(len(self._gt_left), self.log_nonground_max_points)
        right_limit = min(len(self._gt_right), self.log_nonground_max_points)

        lines = [
            "GT points used for comparison (from initialization frame):",
            f"  LEFT  showing {left_limit}/{len(self._gt_left)} points (x, y):",
        ]
        for p in self._gt_left[:left_limit]:
            lines.append(f"    ({p[0]:.3f}, {p[1]:.3f})")
        if left_limit < len(self._gt_left):
            lines.append(f"    ... truncated {len(self._gt_left) - left_limit} points")

        lines.append(f"  RIGHT showing {right_limit}/{len(self._gt_right)} points (x, y):")
        for p in self._gt_right[:right_limit]:
            lines.append(f"    ({p[0]:.3f}, {p[1]:.3f})")
        if right_limit < len(self._gt_right):
            lines.append(f"    ... truncated {len(self._gt_right) - right_limit} points")

        lines.append(f"  NOISE count: {len(self._gt_noise)}")

        self._safe_info("\n".join(lines))

    def _marker_callback(self, msg: MarkerArray):
        # Point evaluation uses *_points markers; edge evaluation uses explicit LINE_LIST segments.
        left_points = []
        right_points = []
        left_edges = []
        right_edges = []

        for i, marker in enumerate(msg.markers):
            if marker.ns in NS_LEFT_NAMES:
                if '_points' in marker.ns:
                    left_points.extend([[p.x, p.y] for p in marker.points])
                elif marker.type == Marker.LINE_LIST:
                    for j in range(0, len(marker.points) - 1, 2):
                        p1 = marker.points[j]
                        p2 = marker.points[j + 1]
                        left_edges.append([p1.x, p1.y, p2.x, p2.y])
            elif marker.ns in NS_RIGHT_NAMES:
                if '_points' in marker.ns:
                    right_points.extend([[p.x, p.y] for p in marker.points])
                elif marker.type == Marker.LINE_LIST:
                    for j in range(0, len(marker.points) - 1, 2):
                        p1 = marker.points[j]
                        p2 = marker.points[j + 1]
                        right_edges.append([p1.x, p1.y, p2.x, p2.y])

        with self._lock:
            self._marker_left = np.array(left_points, dtype=np.float64) if left_points else np.empty((0, 2), dtype=np.float64)
            self._marker_right = np.array(right_points, dtype=np.float64) if right_points else np.empty((0, 2), dtype=np.float64)
            self._marker_left_edges = np.array(left_edges, dtype=np.float64) if left_edges else np.empty((0, 4), dtype=np.float64)
            self._marker_right_edges = np.array(right_edges, dtype=np.float64) if right_edges else np.empty((0, 4), dtype=np.float64)

    # ------------------------------------------------------------------
    def print_final(self):
        with self._lock:
            gt_l  = self._gt_left.copy()
            gt_r  = self._gt_right.copy()
            gt_n  = self._gt_noise.copy()
            mk_l  = self._marker_left.copy()
            mk_r  = self._marker_right.copy()
            mk_l_edges = self._marker_left_edges.copy()
            mk_r_edges = self._marker_right_edges.copy()
            ng_frames = self._ng_frames

        self._safe_print(
            f"Nonground frames processed: {ng_frames}\n"
            f"  GT left  cones (unique): {len(gt_l)}\n"
            f"  GT right cones (unique): {len(gt_r)}\n"
            f"  GT noise points:         {len(gt_n)}\n"
            f"  Marker left  points:     {len(mk_l)}\n"
            f"  Marker right points:     {len(mk_r)}\n"
            f"  Marker left  edges:      {len(mk_l_edges)}\n"
            f"  Marker right edges:      {len(mk_r_edges)}"
        )

        if len(gt_l) == 0 and len(gt_r) == 0:
            self._safe_print("[WARN] No GT data available - no message arrived on nonground_odom.")
            return
        if len(mk_l) == 0 and len(mk_r) == 0:
            self._safe_print("[WARN] No marker data available - no message arrived on interpolated_marker_map_odom.")
            return

        best = choose_best_alignment(gt_l, gt_r, mk_l, mk_r, self.match_threshold)
        left_stats = best['left']
        right_stats = best['right']

        mapping_text = (
            "normal (left->left, right->right)"
            if best['mapping'] == 'normal' else
            "swapped (left->right, right->left)"
        )

        self._safe_print(
            f"Best alignment selected: transform={best['transform']} mapping={mapping_text}"
        )

        if best['mapping'] == 'normal':
            left_marker_count = len(mk_l)
            right_marker_count = len(mk_r)
            left_title = "LEFT  (intensity=1 vs parallel_left_interpolated_map*)"
            right_title = "RIGHT (intensity=2 vs parallel_right_interpolated_map*)"
            left_marker_pts = transform_points(mk_l, best['transform'])
            right_marker_pts = transform_points(mk_r, best['transform'])
            left_marker_edges = transform_edges(mk_l_edges, best['transform'])
            right_marker_edges = transform_edges(mk_r_edges, best['transform'])
        else:
            left_marker_count = len(mk_r)
            right_marker_count = len(mk_l)
            left_title = "LEFT  (intensity=1 vs parallel_right_interpolated_map*)"
            right_title = "RIGHT (intensity=2 vs parallel_left_interpolated_map*)"
            left_marker_pts = transform_points(mk_r, best['transform'])
            right_marker_pts = transform_points(mk_l, best['transform'])
            left_marker_edges = transform_edges(mk_r_edges, best['transform'])
            right_marker_edges = transform_edges(mk_l_edges, best['transform'])

        self._print_results(left_title, left_stats, len(gt_l), left_marker_count)
        self._print_results(right_title, right_stats, len(gt_r), right_marker_count)

        overall_stats = {
            'TP': left_stats['TP'] + right_stats['TP'],
            'FP': left_stats['FP'] + right_stats['FP'],
            'FN': left_stats['FN'] + right_stats['FN'],
        }
        tp = overall_stats['TP']
        fp = overall_stats['FP']
        fn = overall_stats['FN']
        denom_p = tp + fp
        denom_r = tp + fn
        denom_f = 2 * tp + fp + fn
        overall_stats['Precision'] = (tp / denom_p) if denom_p > 0 else float('nan')
        overall_stats['Sensitivity'] = (tp / denom_r) if denom_r > 0 else float('nan')
        overall_stats['F-measure'] = (2 * tp / denom_f) if denom_f > 0 else float('nan')

        self._print_results(
            "OVERALL (LEFT + RIGHT)",
            overall_stats,
            len(gt_l) + len(gt_r),
            left_marker_count + right_marker_count,
        )

        all_cones = merge_points(gt_l, gt_r)
        all_markers = merge_points(left_marker_pts, right_marker_pts)
        hybrid_stats = hybrid_evaluate(all_cones, gt_n, all_markers, self.match_threshold)
        self._print_hybrid_results(
            "HYBRID (cones=positive, noise=negative)",
            hybrid_stats,
            len(all_cones),
            len(gt_n),
            len(all_markers),
        )

        edge_left = evaluate_edges(gt_l, left_marker_edges, self.edge_match_threshold,
                       gt_closed_loop=True)
        edge_right = evaluate_edges(gt_r, right_marker_edges, self.edge_match_threshold,
                        gt_closed_loop=True)

        self._print_edge_results(
            "EDGE LEFT  (consecutive connections)",
            edge_left['stats'],
            edge_left['n_gt_edges'],
            edge_left['n_marker_edges'],
        )
        self._print_edge_results(
            "EDGE RIGHT (consecutive connections)",
            edge_right['stats'],
            edge_right['n_gt_edges'],
            edge_right['n_marker_edges'],
        )

        edge_overall = evaluate_connection_metrics(
            edge_left['stats']['TP'] + edge_right['stats']['TP'],
            edge_left['stats']['FP'] + edge_right['stats']['FP'],
            edge_left['stats']['FN'] + edge_right['stats']['FN'],
        )
        self._print_edge_results(
            "EDGE OVERALL (LEFT + RIGHT)",
            edge_overall,
            edge_left['n_gt_edges'] + edge_right['n_gt_edges'],
            edge_left['n_marker_edges'] + edge_right['n_marker_edges'],
        )

    def enable_tp_preview(self):
        with self._lock:
            self._tp_preview_active = True
        self._safe_print(
            "Preview active: publishing /point_* and /edge_* TP/FP/FN topics, plus /gt_edges in odom frame. Press q or Ctrl+C to stop."
        )
        self._publish_preview()

    def _publish_gt_edges(self):
        with self._lock:
            gt_l = self._gt_left.copy()
            gt_r = self._gt_right.copy()

        if len(gt_l) == 0 and len(gt_r) == 0:
            return

        gt_left_edges = build_edges(gt_l, closed_loop=True)
        gt_right_edges = build_edges(gt_r, closed_loop=True)
        gt_edge_msg = MarkerArray()
        gt_edge_msg.markers.append(self._make_line_list_marker(
            marker_id=1,
            ns='gt_edge_left',
            edges=gt_left_edges,
            rgb=(0.0, 0.9, 0.9),
        ))
        gt_edge_msg.markers.append(self._make_line_list_marker(
            marker_id=2,
            ns='gt_edge_right',
            edges=gt_right_edges,
            rgb=(0.0, 0.6, 0.9),
        ))
        self._gt_edge_pub.publish(gt_edge_msg)

    def _publish_preview(self):
        with self._lock:
            if not self._tp_preview_active:
                return
            gt_l = self._gt_left.copy()
            gt_r = self._gt_right.copy()
            mk_l = self._marker_left.copy()
            mk_r = self._marker_right.copy()
            mk_l_edges = self._marker_left_edges.copy()
            mk_r_edges = self._marker_right_edges.copy()

        if len(gt_l) == 0 and len(gt_r) == 0:
            return

        best = choose_best_alignment(gt_l, gt_r, mk_l, mk_r, self.match_threshold)

        if best['mapping'] == 'normal':
            left_marker_pts = transform_points(mk_l, best['transform'])
            right_marker_pts = transform_points(mk_r, best['transform'])
            left_marker_edges = transform_edges(mk_l_edges, best['transform'])
            right_marker_edges = transform_edges(mk_r_edges, best['transform'])
        else:
            left_marker_pts = transform_points(mk_r, best['transform'])
            right_marker_pts = transform_points(mk_l, best['transform'])
            left_marker_edges = transform_edges(mk_r_edges, best['transform'])
            right_marker_edges = transform_edges(mk_l_edges, best['transform'])

        tp_left = matched_gt_points(gt_l, left_marker_pts, self.match_threshold)
        tp_right = matched_gt_points(gt_r, right_marker_pts, self.match_threshold)
        fn_left = unmatched_gt_points(gt_l, left_marker_pts, self.match_threshold)
        fn_right = unmatched_gt_points(gt_r, right_marker_pts, self.match_threshold)
        fp_left = unmatched_marker_points(gt_l, left_marker_pts, self.match_threshold)
        fp_right = unmatched_marker_points(gt_r, right_marker_pts, self.match_threshold)

        point_tp_msg = MarkerArray()
        point_tp_msg.markers.append(self._make_sphere_list_marker(
            marker_id=1,
            ns='true_positive_left',
            points=tp_left,
            rgb=(0.1, 0.9, 0.2),
        ))
        point_tp_msg.markers.append(self._make_sphere_list_marker(
            marker_id=2,
            ns='true_positive_right',
            points=tp_right,
            rgb=(0.1, 0.4, 1.0),
        ))
        self._point_tp_pub.publish(point_tp_msg)

        point_fn_msg = MarkerArray()
        point_fn_msg.markers.append(self._make_sphere_list_marker(
            marker_id=1,
            ns='false_negative_left',
            points=fn_left,
            rgb=(1.0, 0.2, 0.2),
        ))
        point_fn_msg.markers.append(self._make_sphere_list_marker(
            marker_id=2,
            ns='false_negative_right',
            points=fn_right,
            rgb=(1.0, 0.6, 0.0),
        ))
        self._point_fn_pub.publish(point_fn_msg)

        point_fp_msg = MarkerArray()
        point_fp_msg.markers.append(self._make_sphere_list_marker(
            marker_id=1,
            ns='false_positive_left',
            points=fp_left,
            rgb=(1.0, 0.0, 1.0),
        ))
        point_fp_msg.markers.append(self._make_sphere_list_marker(
            marker_id=2,
            ns='false_positive_right',
            points=fp_right,
            rgb=(0.8, 0.0, 0.8),
        ))
        self._point_fp_pub.publish(point_fp_msg)

        edge_left = classify_edges(
            gt_l,
            left_marker_edges,
            self.edge_match_threshold,
            gt_closed_loop=True,
        )
        edge_right = classify_edges(
            gt_r,
            right_marker_edges,
            self.edge_match_threshold,
            gt_closed_loop=True,
        )

        edge_tp_msg = MarkerArray()
        edge_tp_msg.markers.append(self._make_line_list_marker(
            marker_id=1,
            ns='edge_true_positive_left',
            edges=edge_left['tp_edges'],
            rgb=(0.1, 0.9, 0.2),
        ))
        edge_tp_msg.markers.append(self._make_line_list_marker(
            marker_id=2,
            ns='edge_true_positive_right',
            edges=edge_right['tp_edges'],
            rgb=(0.1, 0.4, 1.0),
        ))
        self._edge_tp_pub.publish(edge_tp_msg)

        edge_fn_msg = MarkerArray()
        edge_fn_msg.markers.append(self._make_line_list_marker(
            marker_id=1,
            ns='edge_false_negative_left',
            edges=edge_left['fn_edges'],
            rgb=(1.0, 0.2, 0.2),
        ))
        edge_fn_msg.markers.append(self._make_line_list_marker(
            marker_id=2,
            ns='edge_false_negative_right',
            edges=edge_right['fn_edges'],
            rgb=(1.0, 0.6, 0.0),
        ))
        self._edge_fn_pub.publish(edge_fn_msg)

        edge_fp_msg = MarkerArray()
        edge_fp_msg.markers.append(self._make_line_list_marker(
            marker_id=1,
            ns='edge_false_positive_left',
            edges=edge_left['fp_edges'],
            rgb=(1.0, 0.0, 1.0),
        ))
        edge_fp_msg.markers.append(self._make_line_list_marker(
            marker_id=2,
            ns='edge_false_positive_right',
            edges=edge_right['fp_edges'],
            rgb=(0.8, 0.0, 0.8),
        ))
        self._edge_fp_pub.publish(edge_fp_msg)

    def _make_sphere_list_marker(self, marker_id: int, ns: str, points: np.ndarray, rgb):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.45
        marker.scale.y = 0.45
        marker.scale.z = 0.45
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 1.0
        marker.points = []
        for xy in points:
            point = Point()
            point.x = float(xy[0])
            point.y = float(xy[1])
            point.z = 0.3
            marker.points.append(point)
        return marker

    def _make_line_list_marker(self, marker_id: int, ns: str, edges: np.ndarray, rgb):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 1.0
        marker.points = []
        for edge in edges:
            start = Point()
            start.x = float(edge[0])
            start.y = float(edge[1])
            start.z = 0.2
            end = Point()
            end.x = float(edge[2])
            end.y = float(edge[3])
            end.z = 0.2
            marker.points.append(start)
            marker.points.append(end)
        return marker

    def _print_results(self, title: str, s: dict, n_gt: int, n_markers: int):
        sep = "=" * 62
        self._safe_print(
            f"\n{sep}\n"
            f"  {title}\n"
            f"  GT cones    : {n_gt}\n"
            f"  Marker pts  : {n_markers}\n"
            f"  TP={s['TP']}  FP={s['FP']}  FN={s['FN']}"
            f"  (TN=0, not defined)\n"
            f"  Precision   = {self._fmt(s['Precision'])}"
            f"    (TP / (TP+FP))\n"
            f"  Sensitivity = {self._fmt(s['Sensitivity'])}"
            f"  (TP / (TP+FN))  [Recall / TPR]\n"
            f"  F-measure   = {self._fmt(s['F-measure'])}"
            f"  (2TP / (2TP+FP+FN))\n"
            f"{sep}"
        )

    def _print_hybrid_results(self, title: str, s: dict, n_cones: int, n_noise: int, n_markers: int):
        sep = "=" * 62
        self._safe_print(
            f"\n{sep}\n"
            f"  {title}\n"
            f"  GT cones    : {n_cones}\n"
            f"  GT noise    : {n_noise}\n"
            f"  Marker pts  : {n_markers}\n"
            f"  TP={s['TP']}  FP={s['FP']}  TN={s['TN']}  FN={s['FN']}\n"
            f"  Precision   = {self._fmt(s['Precision'])}\n"
            f"  Sensitivity = {self._fmt(s['Sensitivity'])}\n"
            f"  Specificity = {self._fmt(s['Specificity'])}\n"
            f"  NPV         = {self._fmt(s['NPV'])}\n"
            f"  F-measure   = {self._fmt(s['F-measure'])}\n"
            f"  Accuracy    = {self._fmt(s['Accuracy'])}\n"
            f"{sep}"
        )

    def _print_edge_results(self, title: str, s: dict, n_gt_edges: int, n_marker_edges: int):
        sep = "=" * 62
        self._safe_print(
            f"\n{sep}\n"
            f"  {title}\n"
            f"  GT edges    : {n_gt_edges}\n"
            f"  Marker edges: {n_marker_edges}\n"
            f"  TP={s['TP']}  FP={s['FP']}  FN={s['FN']}\n"
            f"  Precision   = {self._fmt(s['Precision'])}\n"
            f"  Sensitivity = {self._fmt(s['Sensitivity'])}\n"
            f"  F-measure   = {self._fmt(s['F-measure'])}\n"
            f"{sep}"
        )

    @staticmethod
    def _fmt(v) -> str:
        return f"{v:.4f}" if v == v else "  N/A  "

    def _safe_info(self, text: str):
        """Log via ROS when possible; fallback to stdout during shutdown."""
        try:
            self.get_logger().info(text)
        except Exception:
            print(text)

    def _safe_warn(self, text: str):
        """Warn via ROS when possible; fallback to stdout during shutdown."""
        try:
            self.get_logger().warn(text)
        except Exception:
            print(f"[WARN] {text}")

    @staticmethod
    def _safe_print(text: str):
        print(text)


def main(args=None):
    rclpy.init(args=args)
    node = CompareNode()

    print("Evaluator started.")
    print("Press e to run evaluation.")
    print("Press Ctrl+C to exit.")

    def handle_sigint(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, handle_sigint)

    stop_requested = False

    stdin_fd = None
    old_term_settings = None
    if sys.stdin.isatty():
        stdin_fd = sys.stdin.fileno()
        old_term_settings = termios.tcgetattr(stdin_fd)
        tty.setcbreak(stdin_fd)

    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)

            if stdin_fd is not None:
                ready, _, _ = select.select([stdin_fd], [], [], 0.0)
                if ready:
                    key = sys.stdin.read(1)
                    if key == 'e':
                        node.print_final()
                        if not node._tp_preview_active:
                            node.enable_tp_preview()
                        else:
                            node._publish_preview()
                    elif key == 'g':
                        print("Use e to run evaluation and start topic publishing.")
                    elif key == 'q':
                        stop_requested = True
    finally:
        if stdin_fd is not None and old_term_settings is not None:
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term_settings)

    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
