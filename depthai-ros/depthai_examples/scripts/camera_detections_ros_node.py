#!/usr/bin/env python3
"""
camera_detections_ros_node.py

Subscribes to /color/yolov4_Spatial_detections and publishes interactive markers
for CONFIRMED detected objects on /detected_objects/update.

Detection filter (avoids spam / transient ghost objects):
  1. Confirmed objects: array of (class_name, world_xyz). When a new detection
     falls within CONFIRM_RADIUS_M (20 cm) of any confirmed object → update its
     running-average position and move the marker; do NOT create a duplicate.
  2. Candidates: when a detection is NOT near any confirmed object, it enters
     the candidate pool anchored at its running-average world position.
  3. Confirmation: a candidate is promoted to confirmed once it accumulates
     CONFIRM_SIGHTINGS (3) detections within CANDIDATE_RADIUS_M (20 cm) of
     its anchor position.
  4. Confirmed markers persist until reset via service.

Interactive markers: clickable spheres in the 'map' frame. Click → BUTTON_CLICK
event logged (hook for arm planner).

Reset: ros2 service call /reset_detections std_srvs/srv/Empty
"""

import math
import json
import os
import time
import cv2
from pathlib import Path
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Deque

import rclpy
from rclpy.node import Node
from depthai_ros_msgs.msg import SpatialDetectionArray
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import Vector3, TransformStamped, PoseStamped
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
import tf2_ros
from cv_bridge import CvBridge

# ── Constants ─────────────────────────────────────────────────────────────────
_JSON_PATH = str(Path(__file__).parent / "coco_dataset.json")
with open(_JSON_PATH, "r") as f:
    COCO_DICT = json.load(f)
    print(f"[camera_detections] Loaded COCO dataset with {len(COCO_DICT)} entries")

_CAMERA_FRAME = "oak-d-base-frame"
_WORLD_FRAME  = "map"

# 20 cm radius for both candidate grouping and confirmed-object deduplication
CONFIRM_RADIUS_M:   float = 0.20
CANDIDATE_RADIUS_M: float = 0.20

# Minimum sightings within CANDIDATE_RADIUS_M before promoting to confirmed
CONFIRM_SIGHTINGS: int = 5

# Cap the effective n used in the running-average formula.
# Without this, after n=50+ sightings each new reading has <2% weight and the
# marker effectively stops moving.  With n capped at 20, a new detection always
# carries at least 1/21 ≈ 5% weight — enough to keep converging.
AVG_N_CAP: int = 20

# Density-based ghost pruning:
#   Each confirmed object keeps the timestamps of its last DENSITY_WINDOW_HITS
#   recent detection hits.
#
#   Pruner rule (relative, not absolute):
#     If a confirmed object has a nearby same-class competitor within
#     DENSITY_COMPARE_RADIUS that has >= DENSITY_DOMINATION_RATIO times more
#     recent hits, the sparser one is a ghost and gets removed.
#
#   Why relative works when you look away:
#     - Single real object, camera turns away → 0 hits on both sides → no pruning.
#     - Ghost + real object, camera looks at area → real accumulates hits, ghost 0
#       → ghost gets dominated and pruned.
#     - Two real objects → both accumulate hits, neither dominates → both survive.
DENSITY_WINDOW_HITS:     int   = 30    # sliding window size (max stored timestamps)
DENSITY_WINDOW_SEC:      float = 15.0  # how far back to count hits
DENSITY_COMPARE_RADIUS:  float = CONFIRM_RADIUS_M * 2.0  # search radius for competitors
DENSITY_DOMINATION_RATIO: float = 5.0  # competitor needs this many times more hits to prune
DENSITY_CHECK_SEC:       float = 5.0   # how often to run the pruner

# ── Per-class visual properties ───────────────────────────────────────────────
# Colors: (r, g, b, a) — tweak freely
CLASS_COLORS: dict = {
    # People
    "person":       (0.90, 0.72, 0.56, 0.95),  # warm skin tone
    # Drinkware / food containers
    "bottle":       (0.50, 0.85, 1.00, 0.90),  # light blue
    "cup":          (0.50, 0.85, 1.00, 0.90),  # light blue
    "wine glass":   (0.50, 0.85, 1.00, 0.90),  # light blue
    # Electronics
    "mouse":        (0.15, 0.15, 0.15, 0.95),  # near-black
    "keyboard":     (0.25, 0.25, 0.25, 0.90),  # dark grey
    "laptop":       (0.55, 0.55, 0.55, 0.95),  # grey
    "cell phone":   (0.35, 0.35, 0.35, 0.90),  # dark grey
    "tv":           (0.05, 0.05, 0.05, 0.95),  # black
    "monitor":      (0.05, 0.05, 0.05, 0.95),  # black
    # Toys
    "teddy bear":   (0.95, 0.10, 0.10, 0.95),  # red
    "sports ball":  (0.95, 0.55, 0.10, 0.90),  # orange
    # Furniture
    "chair":        (0.60, 0.40, 0.20, 0.85),  # warm brown
    "couch":        (0.65, 0.45, 0.25, 0.85),
    "sofa":         (0.65, 0.45, 0.25, 0.85),
    # default (applied when class not listed)
    "_default":     (0.20, 0.80, 1.00, 0.90),  # original cyan-blue
}

# Sphere diameter (metres) — chosen to roughly match physical object extent so the
# marker sits *around* the object rather than engulfing it.
# CLASS_SIZES: fallback marker diameter (m) when no bbox estimate is available.
# CLASS_SIZE_BOUNDS: (min_m, max_m) real-world plausible size range per class.
#   The geometric estimate is clamped to this range instead of a global _PHYS_MAX.
#   Sizes are intentionally generous — better to show a slightly oversized sphere
#   than to silently cap everything at 0.30 m.
CLASS_SIZES: dict[str, float] = {
    "person":       0.35,   # shoulder-width sphere
    "bottle":       0.08,
    "cup":          0.08,
    "wine glass":   0.07,
    "mouse":        0.08,
    "keyboard":     0.35,
    "laptop":       0.32,
    "cell phone":   0.08,
    "tv":           0.60,
    "monitor":      0.45,
    "chair":        0.55,
    "couch":        0.90,
    "sofa":         0.90,
    "teddy bear":   0.30,
    "sports ball":  0.22,
    "book":         0.20,
    "_default":     0.15,
}

# Per-class plausible real-world size bounds (min_m, max_m).
# The geometric estimate is clamped to these rather than a global 0.30 m cap.
CLASS_SIZE_BOUNDS: dict[str, tuple[float, float]] = {
    "person":       (0.25, 0.60),
    "bottle":       (0.04, 0.14),
    "cup":          (0.05, 0.14),
    "wine glass":   (0.04, 0.12),
    "mouse":        (0.05, 0.14),
    "keyboard":     (0.25, 0.50),
    "laptop":       (0.25, 0.45),
    "cell phone":   (0.05, 0.18),
    "tv":           (0.35, 2.00),
    "monitor":      (0.25, 1.00),
    "chair":        (0.40, 1.00),
    "couch":        (0.60, 2.20),
    "sofa":         (0.60, 2.20),
    "teddy bear":   (0.12, 0.60),
    "sports ball":  (0.08, 0.30),
    "book":         (0.12, 0.35),
    "_default":     (0.05, 0.80),
}


# OAK-D Lite 300×300 preview, ~81° HFOV (OV5645 sensor).
#
# Pure geometry:  physical_m = (bbox_px / W) × 2 × depth × tan(HFOV/2)
#               = bbox_px × depth × (2 × tan(40.5°) / 300)
#               = bbox_px × depth × 0.005693
#
# YOLO bboxes are empirically ~1.5× the actual object width (they include
# padding to give context to the detector).  Dividing by 1.5 converts bbox
# size to object size.  This is a physics-grounded factor, not a magic number.
#
# Per-class size clamping (CLASS_SIZE_BOUNDS) replaces the old global 0.30 m
# cap so each class is bounded to its real-world plausible range.
_PREVIEW_DIM:       int   = 300
_YOLO_BBOX_INFLATE: float = 3.5   # tuned: bbox_px * depth * scale / inflate → object diameter

# Camera field of view — OAK-D Lite with 300×300 square preview.
# Both H and V FOV are equal for a square sensor (81° total → 40.5° half-angle).
_CAM_HFOV_DEG: float = 81.0
_CAM_VFOV_DEG: float = 81.0
_CAM_TAN_H:    float = math.tan(math.radians(_CAM_HFOV_DEG / 2))   # tan(40.5°)
_CAM_TAN_V:    float = math.tan(math.radians(_CAM_VFOV_DEG / 2))

# Candidate FOV-based culling:
#   If a candidate has been inside the camera's FOV for CANDIDATE_FOV_TICKS
#   consecutive detection callbacks without reaching CONFIRM_SIGHTINGS, it is
#   a ghost/noise detection and gets discarded.
#   At ~30 Hz detections this is roughly 1 second of continuous visibility.
CANDIDATE_FOV_TICKS: int = 20
_BBOX_SCALE:        float = (2 * math.tan(math.radians(40.5)) / _PREVIEW_DIM) / _YOLO_BBOX_INFLATE
_PHYS_MIN:          float = 0.04  # below this = sensor noise (m)


# ── Data structures ───────────────────────────────────────────────────────────

@dataclass
class _Candidate:
    """Tentative detection waiting for confirmation."""
    class_name: str
    wx: float           # world-frame anchor position (running average)
    wy: float
    wz: float
    phys_size: float    # estimated physical diameter (m), running average
    sighting_count: int = 1
    fov_ticks: int = 0  # consecutive callbacks where this candidate was inside camera FOV


@dataclass
class _ConfirmedObject:
    """Fully confirmed detection with an active RViz marker."""
    class_name: str
    wx: float
    wy: float
    wz: float
    marker_name: str
    phys_size: float    # estimated physical diameter (m), running average
    sighting_count: int = CONFIRM_SIGHTINGS
    # Sliding window of hit timestamps — used for density-based ghost pruning.
    # Not in __init__ args so we use field(default_factory=...).
    recent_hits: Deque[float] = field(
        default_factory=lambda: deque(maxlen=DENSITY_WINDOW_HITS)
    )


# ── Helpers ───────────────────────────────────────────────────────────────────

def _dist3(ax, ay, az, bx, by, bz) -> float:
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)


def _point_in_camera_fov(tf_stamped: TransformStamped,
                         wx: float, wy: float, wz: float) -> bool:
    """Return True if the world-frame point (wx,wy,wz) falls inside the camera's FOV.

    Strategy:
      tf_stamped is T_world←camera (transforms camera-frame points to world).
      We need the inverse — T_camera←world — to express the world point in
      camera body frame, then check the horizontal/vertical angles.

    Camera body frame convention (REP 103, oak-d-base-frame):
      x = forward (optical depth axis)
      y = left
      z = up

    A point is visible if:
      - x > 0  (in front of camera)
      - |y / x| ≤ tan(HFOV/2)
      - |z / x| ≤ tan(VFOV/2)
    """
    t  = tf_stamped.transform.translation
    q  = tf_stamped.transform.rotation
    # Inverse quaternion = conjugate (negate xyz, keep w)
    qx, qy, qz, qw = -q.x, -q.y, -q.z, q.w
    # Translate: vector from camera origin to the point, in world frame
    rx, ry, rz = wx - t.x, wy - t.y, wz - t.z
    # Rotate by inverse quaternion → point in camera body frame
    ix =  qw*rx + qy*rz - qz*ry
    iy =  qw*ry + qz*rx - qx*rz
    iz =  qw*rz + qx*ry - qy*rx
    iw = -qx*rx - qy*ry - qz*rz
    px = ix*qw + iw*(-qx) + iy*(-qz) - iz*(-qy)
    py = iy*qw + iw*(-qy) + iz*(-qx) - ix*(-qz)
    pz = iz*qw + iw*(-qz) + ix*(-qy) - iy*(-qx)
    # px=forward, py=left, pz=up
    if px <= 0.10:   # 10 cm min distance — avoid divide-by-zero and near-plane noise
        return False
    return abs(py / px) <= _CAM_TAN_H and abs(pz / px) <= _CAM_TAN_V


def _transform_point(tf_stamped: TransformStamped, x: float, y: float, z: float):
    """Apply TransformStamped (translation + quaternion rotation) to a point."""
    t  = tf_stamped.transform.translation
    q  = tf_stamped.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    ix =  qw * x + qy * z - qz * y
    iy =  qw * y + qz * x - qx * z
    iz =  qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z
    rx = ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy)
    ry = iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz)
    rz = iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx)
    return rx + t.x, ry + t.y, rz + t.z


# ── Node ─────────────────────────────────────────────────────────────────────

class CameraDetectionsNode(Node):

    def __init__(self):
        super().__init__("camera_det_node")

        # TF & CV Bridges
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._bridge      = CvBridge()
        self._latest_img  = None

        # Interactive marker server.
        self._im_server = InteractiveMarkerServer(self, "detected_objects")
        self._im_server.clear()
        self._im_server.applyChanges()

        # Goal pose publisher — feeds nav2_goal_pose_listener.py on marker click
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # Detection state
        self._candidates:     List[_Candidate]       = []
        self._confirmed:      List[_ConfirmedObject] = []
        self._marker_counter: int                    = 0

        # Subscribers
        self._image_sub = self.create_subscription(
            Image,
            "/color/preview/image",  # Change this to your active preview image topic if different
            self._image_callback,
            10
        )

        self._sub = self.create_subscription(
            SpatialDetectionArray,
            "/color/yolov4_Spatial_detections",
            self._detections_callback,
            10,
        )

        # Service: call to wipe all markers and reset detection state
        self._reset_srv = self.create_service(
            Empty, "reset_detections", self._reset_callback
        )

        # Debug: log pairwise distances between all confirmed teddy bears every 2 s
        self.create_timer(2.0, self._log_teddy_distances)

        # TTL pruner: remove confirmed objects that stopped being seen
        self.create_timer(DENSITY_CHECK_SEC, self._prune_stale_confirmed)

        # FOV-based candidate culler (called inline in _detections_callback)

        self.get_logger().info(
            f"Detection filter ready  "
            f"(confirm_radius={CONFIRM_RADIUS_M * 100:.0f}cm, "
            f"sightings={CONFIRM_SIGHTINGS})  "
            f"reset via: ros2 service call /reset_detections std_srvs/srv/Empty"
        )
    # ── TTL pruner ──────────────────────────────────────────────────────────

    def _prune_stale_confirmed(self):
        """Remove ghost/duplicate confirmed objects via relative density comparison.

        For each confirmed object X we look for a nearby same-class competitor C
        (within DENSITY_COMPARE_RADIUS).  If C has >= DENSITY_DOMINATION_RATIO
        times more recent hits than X, X is a ghost and gets removed.

        Key property: if you look away, *both* X and C get 0 new hits, so neither
        dominates → the real object is safe.  Only when you look at the area does
        the real one pull ahead; the ghost stays at 0 and gets pruned.
        """
        now = time.monotonic()
        cutoff = now - DENSITY_WINDOW_SEC
        to_prune: list[_ConfirmedObject] = []

        for i, obj in enumerate(self._confirmed):
            if obj in to_prune:
                continue
            my_hits = sum(1 for t in obj.recent_hits if t >= cutoff)

            for j, other in enumerate(self._confirmed):
                if i == j or other in to_prune:
                    continue
                if other.class_name != obj.class_name:
                    continue
                dist = math.sqrt(
                    (obj.wx - other.wx) ** 2 +
                    (obj.wy - other.wy) ** 2 +
                    (obj.wz - other.wz) ** 2
                )
                if dist > DENSITY_COMPARE_RADIUS:
                    continue
                other_hits = sum(1 for t in other.recent_hits if t >= cutoff)
                if other_hits >= DENSITY_DOMINATION_RATIO * max(my_hits, 1):
                    self.get_logger().info(
                        f"[GHOST PRUNED] {obj.class_name} '{obj.marker_name}' removed — "
                        f"competitor '{other.marker_name}' has {other_hits} vs my {my_hits} "
                        f"hits in last {DENSITY_WINDOW_SEC:.0f}s "
                        f"(ratio {other_hits / max(my_hits, 1):.1f}×, dist {dist:.2f}m)"
                    )
                    to_prune.append(obj)
                    break

        for obj in to_prune:
            self._confirmed.remove(obj)
            self._im_server.erase(obj.marker_name)
        if to_prune:
            self._im_server.applyChanges()

    # ── FOV-based candidate culling ──────────────────────────────────────

    def _cull_fov_candidates(self, tf_stamped: TransformStamped) -> None:
        """Discard candidates that have been in the camera's FOV too long without confirming.

        Each call increments fov_ticks for every candidate currently visible.
        If fov_ticks reaches CANDIDATE_FOV_TICKS before sighting_count reaches
        CONFIRM_SIGHTINGS, the candidate is ghosting noise and gets removed.

        Candidates outside the FOV are left alone (robot may not have looked at
        them yet).
        """
        to_remove = []
        for cand in self._candidates:
            if _point_in_camera_fov(tf_stamped, cand.wx, cand.wy, cand.wz):
                cand.fov_ticks += 1
                if cand.fov_ticks >= CANDIDATE_FOV_TICKS:
                    to_remove.append(cand)
                    self.get_logger().info(
                        f"[FOV CULL] {cand.class_name} candidate discarded — "
                        f"{cand.fov_ticks} frames in FOV, only "
                        f"{cand.sighting_count}/{CONFIRM_SIGHTINGS} hits  "
                        f"world=({cand.wx:.2f}, {cand.wy:.2f}, {cand.wz:.2f})"
                    )
        for c in to_remove:
            self._candidates.remove(c)

    # ── Debug: teddy bear distance monitor ───────────────────────────────

    def _log_teddy_distances(self):
        bears = [o for o in self._confirmed if o.class_name == "teddy bear"]
        if len(bears) < 2:
            if bears:
                self.get_logger().info(
                    f"[TEDDY] 1 confirmed bear at "
                    f"({bears[0].wx:.3f}, {bears[0].wy:.3f}, {bears[0].wz:.3f})  "
                    f"phys_size={bears[0].phys_size:.3f}m  n={bears[0].sighting_count}"
                )
            return
        for i, a in enumerate(bears):
            for j, b in enumerate(bears):
                if j <= i:
                    continue
                d = _dist3(a.wx, a.wy, a.wz, b.wx, b.wy, b.wz)
                flag = " << SHOULD MERGE" if d < CONFIRM_RADIUS_M else ""
                self.get_logger().info(
                    f"[TEDDY DIST] {a.marker_name} <-> {b.marker_name}: "
                    f"{d * 100:.1f} cm  "
                    f"(merge threshold={CONFIRM_RADIUS_M * 100:.0f} cm){flag}"
                )

    # ── CV Image Callback ──────────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        """Grabs raw camera image updates for UI display overlays."""
        try:
            self._latest_img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Conversion Error: {e}")

    # ── Reset service ──────────────────────────────────────────────────────

    def _reset_callback(self, request, response):
        """Wipe all interactive markers and clear detection state."""
        self._im_server.clear()
        self._im_server.applyChanges()
        cleared = len(self._confirmed)
        self._confirmed.clear()
        self._candidates.clear()
        self._marker_counter = 0
        self.get_logger().info(
            f"[RESET] Cleared {cleared} confirmed object(s) and all candidates"
        )
        return response

    # ── Marker factory ─────────────────────────────────────────────────────

    def _make_marker(
        self, name: str, label: str, wx: float, wy: float, wz: float,
        phys_size: float = 0.0
    ) -> InteractiveMarker:
        # Resolve class name: strip trailing _N counter suffix for lookup
        base_class = " ".join(
            w for w in label.lower().split() if not w.lstrip("-").isdigit()
        )
        r, g, b, a = CLASS_COLORS.get(base_class, CLASS_COLORS["_default"])
        # Use bbox-derived size when available, else fall back to class table
        diameter   = phys_size if phys_size > 0.0 else CLASS_SIZES.get(base_class, CLASS_SIZES["_default"])

        im = InteractiveMarker()
        im.header.frame_id = _WORLD_FRAME
        im.header.stamp    = self.get_clock().now().to_msg()
        im.name            = name
        im.description     = ""   # suppress default label; we draw our own
        im.pose.position.x = wx
        im.pose.position.y = wy
        im.pose.position.z = wz
        im.scale           = max(0.15, diameter * 2.0)  # control scale proportional to sphere

        sphere       = Marker()
        sphere.type  = Marker.SPHERE
        sphere.scale = Vector3(x=diameter, y=diameter, z=diameter)
        sphere.color = ColorRGBA(r=r, g=g, b=b, a=a)

        # Red label sitting just above the sphere
        # x/y/z must all be set on TEXT_VIEW_FACING — leaving x=0 or y=0
        # causes RViz to stretch inter-character spacing wildly.
        char_h      = diameter * 0.7    # half the original size
        text_offset = diameter * 0.65   # close to top of sphere
        text_marker = Marker()
        text_marker.type  = Marker.TEXT_VIEW_FACING
        text_marker.text  = label
        text_marker.scale = Vector3(x=char_h, y=char_h, z=char_h)
        text_marker.color = ColorRGBA(r=1.0, g=0.15, b=0.15, a=1.0)  # bright red
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = text_offset

        ctrl                  = InteractiveMarkerControl()
        ctrl.name             = "visual"
        ctrl.always_visible   = True
        ctrl.interaction_mode = InteractiveMarkerControl.BUTTON
        ctrl.markers.append(sphere)
        ctrl.markers.append(text_marker)
        im.controls.append(ctrl)
        return im

    # ── TF helper ──────────────────────────────────────────────────────────

    def _get_camera_tf(self) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                _WORLD_FRAME, _CAMERA_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    # ── Confirmed-object merge pass ────────────────────────────────────────

    def _merge_confirmed(self):
        """Scan all confirmed objects once; merge the first pair of same-class
        objects that fall within CONFIRM_RADIUS_M of each other.  Repeat until
        no more merges are possible."""
        merged = True
        while merged:
            merged = False
            for i, a in enumerate(self._confirmed):
                for j, b in enumerate(self._confirmed):
                    if j <= i:
                        continue
                    if a.class_name != b.class_name:
                        continue
                    if _dist3(a.wx, a.wy, a.wz, b.wx, b.wy, b.wz) >= CONFIRM_RADIUS_M:
                        continue

                    # Weighted average by sighting count (both capped)
                    na, nb = min(a.sighting_count, AVG_N_CAP), min(b.sighting_count, AVG_N_CAP)
                    total  = na + nb
                    mx = (a.wx * na + b.wx * nb) / total
                    my = (a.wy * na + b.wy * nb) / total
                    mz = (a.wz * na + b.wz * nb) / total
                    ms = (a.phys_size * na + b.phys_size * nb) / total

                    # Remove both old markers
                    self._im_server.erase(a.marker_name)
                    self._im_server.erase(b.marker_name)
                    self._confirmed.remove(b)   # remove higher index first
                    self._confirmed.remove(a)

                    # Create merged confirmed object (keep a's marker name)
                    self._marker_counter += 1
                    new_name = f"{a.class_name.replace(' ', '_')}_{self._marker_counter}"
                    merged_obj = _ConfirmedObject(
                        class_name=a.class_name,
                        wx=mx, wy=my, wz=mz,
                        marker_name=new_name,
                        phys_size=ms,
                        sighting_count=total,
                    )
                    # Inherit all recent hits from both parents
                    for t in list(a.recent_hits) + list(b.recent_hits):
                        merged_obj.recent_hits.append(t)
                    self._confirmed.append(merged_obj)

                    self._im_server.insert(
                        self._make_marker(new_name, a.class_name, mx, my, mz,
                                          phys_size=ms),
                        feedback_callback=self._on_feedback,
                    )
                    self._im_server.applyChanges()

                    self.get_logger().info(
                        f"[MERGED] {a.class_name} ({a.marker_name} + {b.marker_name}) "
                        f"→ {new_name}  world=({mx:.3f}, {my:.3f}, {mz:.3f}) m"
                    )
                    merged = True
                    break   # restart outer while with updated list
                if merged:
                    break

    # ── Two-stage detection filter ─────────────────────────────────────────

    def _process_detection(self, class_name: str, wx: float, wy: float, wz: float,
                            phys_size: float = 0.0):
        """
        Stage 1 — Deduplication against confirmed objects:
          If (wx,wy,wz) is within CONFIRM_RADIUS_M of ANY confirmed object → already
          mapped, discard silently.

        Stage 2 — Candidate accumulation:
          Find the nearest candidate of the same class within CANDIDATE_RADIUS_M.
          Increment its sighting count. Once it hits CONFIRM_SIGHTINGS, promote it
          to confirmed and publish a marker.
          If no matching candidate exists, create one.
          """
        # Stage 1: near a confirmed object? → update running average and move marker
        for obj in self._confirmed:
            if (obj.class_name == class_name and
                    _dist3(wx, wy, wz, obj.wx, obj.wy, obj.wz) < CONFIRM_RADIUS_M):
                # Running average: no need to store all previous readings.
                # n is capped so the marker keeps responding to new detections
                # even after many sightings.
                n = min(obj.sighting_count, AVG_N_CAP)
                obj.wx = (obj.wx * n + wx) / (n + 1)
                obj.wy = (obj.wy * n + wy) / (n + 1)
                obj.wz = (obj.wz * n + wz) / (n + 1)
                if phys_size > 0.0:
                    obj.phys_size = (obj.phys_size * n + phys_size) / (n + 1)
                obj.sighting_count += 1
                obj.last_seen = time.monotonic()
                self._im_server.erase(obj.marker_name)
                self._im_server.insert(
                    self._make_marker(
                        obj.marker_name, obj.class_name, obj.wx, obj.wy, obj.wz,
                        phys_size=obj.phys_size,
                    ),
                    feedback_callback=self._on_feedback,
                )
                self._im_server.applyChanges()

                self.get_logger().debug(
                    f"[UPDATED] {class_name} marker moved to "
                    f"({obj.wx:.3f}, {obj.wy:.3f}, {obj.wz:.3f}) m "
                    f"[n={obj.sighting_count}]"
                )
                self._merge_confirmed()
                return  # handled — do not feed candidate pool

        # Stage 2: find matching candidate
        match: Optional[_Candidate] = None
        for cand in self._candidates:
            if (cand.class_name == class_name and
                    _dist3(wx, wy, wz, cand.wx, cand.wy, cand.wz) < CANDIDATE_RADIUS_M):
                match = cand
                break

        if match is None:
            self._candidates.append(_Candidate(
                class_name=class_name, wx=wx, wy=wy, wz=wz,
                phys_size=phys_size, sighting_count=1
            ))
            self.get_logger().debug(
                f"New candidate: {class_name} "
                f"world=({wx:.2f}, {wy:.2f}, {wz:.2f}) "
                f"[1/{CONFIRM_SIGHTINGS}]"
            )
            return

        # Update candidate position with running average (n capped)
        n = min(match.sighting_count, AVG_N_CAP)
        match.wx = (match.wx * n + wx) / (n + 1)
        match.wy = (match.wy * n + wy) / (n + 1)
        match.wz = (match.wz * n + wz) / (n + 1)
        if phys_size > 0.0:
            match.phys_size = (match.phys_size * n + phys_size) / (n + 1)
        match.sighting_count += 1

        self.get_logger().debug(
            f"Candidate sighting: {class_name} "
            f"avg=({match.wx:.3f}, {match.wy:.3f}, {match.wz:.3f}) "
            f"[{match.sighting_count}/{CONFIRM_SIGHTINGS}]"
        )

        if match.sighting_count >= CONFIRM_SIGHTINGS:
            # Promote to confirmed ──────────────────────────────────────────
            self._candidates.remove(match)
            self._marker_counter += 1
            marker_name = f"{class_name.replace(' ', '_')}_{self._marker_counter}"

            confirmed = _ConfirmedObject(
                class_name=class_name,
                wx=match.wx, wy=match.wy, wz=match.wz,
                marker_name=marker_name,
                phys_size=match.phys_size,
                sighting_count=match.sighting_count,
            )
            # Seed recent_hits with CONFIRM_SIGHTINGS timestamps so the object
            # doesn't get immediately pruned before the first detection cycle.
            now = time.monotonic()
            for _ in range(match.sighting_count):
                confirmed.recent_hits.append(now)
            self._confirmed.append(confirmed)

            self.get_logger().info(
                f"[CONFIRMED #{self._marker_counter}] {class_name}  "
                f"world=({match.wx:.3f}, {match.wy:.3f}, {match.wz:.3f}) m"
            )

            self._im_server.insert(
                self._make_marker(
                    marker_name, class_name, match.wx, match.wy, match.wz,
                    phys_size=match.phys_size,
                ),
                feedback_callback=self._on_feedback,
            )
            self._im_server.applyChanges()
            self._merge_confirmed()

    # ── Detection callback ─────────────────────────────────────────────────

    def _detections_callback(self, msg: SpatialDetectionArray):
        tf_stamped = self._get_camera_tf()
        
        # Local variable clone of image buffer for visual stream manipulation
        display_img = self._latest_img.copy() if self._latest_img is not None else None

        for det in msg.detections:
            if not det.results:
                continue

            class_id   = int(det.results[0].class_id)
            score      = det.results[0].score
            class_name = COCO_DICT.get(str(class_id + 1), "unknown")

            # Raw detection offset in OAK-D optical frame:
            #   optical x = right, optical y = UP (inverted vs ROS standard), optical z = forward (depth)
            ox, oy, oz = det.position.x, det.position.y, det.position.z

            # Remap optical → world body frame (REP 103: x=forward, y=left, z=up):
            #   world x = forward → +optical_z  (depth)
            #   world y = left    → -optical_x  (optical x is right → negate)
            #   world z = up      → +optical_y  (OAK-D optical y is already up → no negation)
            cx =  oz
            cy = -ox
            cz =  oy

            if tf_stamped is None:
                self.get_logger().warn(
                    f"No TF '{_CAMERA_FRAME}'→'{_WORLD_FRAME}' — skipping filter",
                    throttle_duration_sec=5.0,
                )
                self.get_logger().info(
                    f"class='{class_name}' | "
                    f"detection_offset: x={cx:.3f} y={cy:.3f} z={cz:.3f} m  "
                    f"conf={score:.2f}  "
                    f"[no TF — world pos unavailable]",
                    throttle_duration_sec=1.0,
                )
            else:
                # Camera origin in world frame (translation of the TF)
                t = tf_stamped.transform.translation
                cam_wx, cam_wy, cam_wz = t.x, t.y, t.z

                # Detection position in world frame
                wx, wy, wz = _transform_point(tf_stamped, cx, cy, cz)

                # Estimate physical object diameter from bbox pixels + depth.
                # Formula: physical_m = bbox_px * depth * (2*tan(HFOV/2) / W) / YOLO_inflate
                # Clamped to per-class real-world bounds (not a global cap).
                bbox = det.bbox
                bbox_px      = min(bbox.size_x, bbox.size_y)
                geo_estimate = bbox_px * abs(oz) * _BBOX_SCALE
                _lo, _hi     = CLASS_SIZE_BOUNDS.get(class_name,
                                                      CLASS_SIZE_BOUNDS["_default"])
                phys_size    = float(max(_PHYS_MIN, max(_lo, min(_hi, geo_estimate))))

                self.get_logger().info(
                    f"class='{class_name}' | "
                    f"bbox_px=({bbox.size_x:.1f}x{bbox.size_y:.1f}) depth={oz:.2f}m "
                    f"geo={geo_estimate:.3f}m phys={phys_size:.3f}m  "
                    f"conf={score:.2f}  |  "
                    f"obj_world: x={wx:.3f} y={wy:.3f} z={wz:.3f}",
                    throttle_duration_sec=1.0,
                )

                self._process_detection(class_name, wx, wy, wz, phys_size=phys_size)

            # ── OpenCV Screen Diagnostic Display ───────────────────────────────────
            if display_img is not None:
                bbox = det.bbox
                center_x = bbox.center.position.x
                center_y = bbox.center.position.y
                size_x = bbox.size_x
                size_y = bbox.size_y

                # Transform normalized/center values to bounding box pixel coordinates
                xmin = int(center_x - size_x / 2)
                ymin = int(center_y - size_y / 2)
                xmax = int(center_x + size_x / 2)
                ymax = int(center_y + size_y / 2)

                # Draw standard bounding boxes and center track points
                cv2.rectangle(display_img, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
                cv2.circle(display_img, (int(center_x), int(center_y)), 4, (0, 255, 0), -1)
                
                # Setup specific label texts with raw camera-frame values
                lbl_name = f"Object: {class_name} ({score:.2f})"
                lbl_ox = f"RAW Cam X: {ox:.3f}m"
                lbl_oy = f"RAW Cam Y: {oy:.3f}m"
                lbl_oz = f"RAW Cam Z: {oz:.3f}m"
                
                # Render label text box background block
                cv2.rectangle(display_img, (xmin, ymin - 65), (xmin + 180, ymin), (0, 0, 0), -1)
                cv2.putText(display_img, lbl_name, (xmin + 5, ymin - 52), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                cv2.putText(display_img, lbl_ox, (xmin + 5, ymin - 39), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                cv2.putText(display_img, lbl_oy, (xmin + 5, ymin - 26), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
                cv2.putText(display_img, lbl_oz, (xmin + 5, ymin - 13), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # After processing all detections, cull candidates that have been
        # visible in the FOV long enough to have confirmed if they were real.
        if tf_stamped is not None:
            self._cull_fov_candidates(tf_stamped)

        # Draw updated display loop to active screen window
        if display_img is not None:
            cv2.imshow("OAK-D Raw Optical Frame Diagnostics", display_img)
            cv2.waitKey(1)

    # ── Marker feedback ────────────────────────────────────────────────────

    def _on_feedback(self, feedback):
        from visualization_msgs.msg import InteractiveMarkerFeedback
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            pos = feedback.pose.position
            self.get_logger().info(
                f"[GRAB] '{feedback.marker_name}' at "
                f"({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) "
                f"in '{feedback.header.frame_id}'"
            )

            # Publish goal pose to nav2_goal_pose_listener.py on /goal_pose
            goal = PoseStamped()
            goal.header.frame_id = _WORLD_FRAME
            goal.header.stamp    = self.get_clock().now().to_msg()
            goal.pose.position.x = pos.x
            goal.pose.position.y = pos.y
            goal.pose.position.z = 0.0   # nav2 operates on 2-D ground plane
            goal.pose.orientation.w = 1.0  # identity — face toward goal
            self._goal_pub.publish(goal)
            self.get_logger().info(
                f"[NAV GOAL] Published to /goal_pose: "
                f"({pos.x:.3f}, {pos.y:.3f}) in '{_WORLD_FRAME}'"
            )


def main(args=None):
    # Minimal log format: "[LEVEL] [node]: message"
    # Use setdefault so an externally set variable still takes precedence.
    os.environ.setdefault(
        "RCUTILS_CONSOLE_OUTPUT_FORMAT",
        "[{severity}] [{name}]: {message}",
    )
    rclpy.init(args=args)
    node = CameraDetectionsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()