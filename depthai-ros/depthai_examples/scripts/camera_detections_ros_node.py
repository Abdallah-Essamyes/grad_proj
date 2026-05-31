#!/usr/bin/env python3
"""
camera_detections_ros_node.py

Subscribes to /color/yolov4_Spatial_detections and publishes interactive markers
for CONFIRMED detected objects on /detected_objects/update.

Detection filter (avoids spam / transient ghost objects):
  1. Confirmed objects: array of (class_name, world_xyz). When a new detection
     falls within CONFIRM_RADIUS_M (10 cm) of any confirmed object → IGNORE (already known).
  2. Candidates: when a detection is NOT near any confirmed object, it enters
     the candidate pool anchored at its first-sighting world position.
  3. Confirmation: a candidate is promoted to confirmed once it accumulates
     CONFIRM_SIGHTINGS (3) detections within CANDIDATE_RADIUS_M (10 cm) of
     its anchor position.
  4. Confirmed markers persist until reset via service.

Interactive markers: clickable spheres in the 'map' frame. Click → BUTTON_CLICK
event logged (hook for arm planner).

Reset: ros2 service call /reset_detections std_srvs/srv/Empty
"""

import math
import json
import cv2
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from depthai_ros_msgs.msg import SpatialDetectionArray
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import Vector3, TransformStamped
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

# 10 cm radius for both candidate grouping and confirmed-object deduplication
CONFIRM_RADIUS_M:   float = 0.10
CANDIDATE_RADIUS_M: float = 0.10

# Minimum sightings within CANDIDATE_RADIUS_M before promoting to confirmed
CONFIRM_SIGHTINGS: int = 3


# ── Data structures ───────────────────────────────────────────────────────────

@dataclass
class _Candidate:
    """Tentative detection waiting for confirmation."""
    class_name: str
    wx: float           # world-frame anchor position (first sighting)
    wy: float
    wz: float
    sighting_count: int = 1


@dataclass
class _ConfirmedObject:
    """Fully confirmed detection with an active RViz marker."""
    class_name: str
    wx: float
    wy: float
    wz: float
    marker_name: str


# ── Helpers ───────────────────────────────────────────────────────────────────

def _dist3(ax, ay, az, bx, by, bz) -> float:
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)


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
        super().__init__("camera_detections_ros_node")

        # TF & CV Bridges
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._bridge      = CvBridge()
        self._latest_img  = None

        # Interactive marker server.
        self._im_server = InteractiveMarkerServer(self, "detected_objects")
        self._im_server.clear()
        self._im_server.applyChanges()

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

        self.get_logger().info(
            f"Detection filter ready  "
            f"(confirm_radius={CONFIRM_RADIUS_M * 100:.0f}cm, "
            f"sightings={CONFIRM_SIGHTINGS})  "
            f"reset via: ros2 service call /reset_detections std_srvs/srv/Empty"
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
        self, name: str, label: str, wx: float, wy: float, wz: float
    ) -> InteractiveMarker:
        im = InteractiveMarker()
        im.header.frame_id = _WORLD_FRAME
        im.header.stamp    = self.get_clock().now().to_msg()
        im.name            = name
        im.description     = label
        im.pose.position.x = wx
        im.pose.position.y = wy
        im.pose.position.z = wz
        im.scale           = 0.25

        sphere       = Marker()
        sphere.type  = Marker.SPHERE
        sphere.scale = Vector3(x=0.12, y=0.12, z=0.12)
        sphere.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.9)

        ctrl                  = InteractiveMarkerControl()
        ctrl.name             = "visual"
        ctrl.always_visible   = True
        ctrl.interaction_mode = InteractiveMarkerControl.BUTTON
        ctrl.markers.append(sphere)
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

    # ── Two-stage detection filter ─────────────────────────────────────────

    def _process_detection(self, class_name: str, wx: float, wy: float, wz: float):
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
        # Stage 1: near a confirmed object?
        for obj in self._confirmed:
            if _dist3(wx, wy, wz, obj.wx, obj.wy, obj.wz) < CONFIRM_RADIUS_M:
                return  # already known — ignore

        # Stage 2: find matching candidate
        match: Optional[_Candidate] = None
        for cand in self._candidates:
            if (cand.class_name == class_name and
                    _dist3(wx, wy, wz, cand.wx, cand.wy, cand.wz) < CANDIDATE_RADIUS_M):
                match = cand
                break

        if match is None:
            self._candidates.append(_Candidate(
                class_name=class_name, wx=wx, wy=wy, wz=wz, sighting_count=1
            ))
            self.get_logger().debug(
                f"New candidate: {class_name} "
                f"world=({wx:.2f}, {wy:.2f}, {wz:.2f}) "
                f"[1/{CONFIRM_SIGHTINGS}]"
            )
            return

        match.sighting_count += 1
        self.get_logger().debug(
            f"Candidate sighting: {class_name} "
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
            )
            self._confirmed.append(confirmed)

            self.get_logger().info(
                f"[CONFIRMED #{self._marker_counter}] {class_name}  "
                f"world=({match.wx:.3f}, {match.wy:.3f}, {match.wz:.3f}) m"
            )

            self._im_server.insert(
                self._make_marker(
                    marker_name, class_name, match.wx, match.wy, match.wz
                ),
                feedback_callback=self._on_feedback,
            )
            self._im_server.applyChanges()

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

            # Raw detection offset in optical frame (REP 103 _optical convention):
            #   optical x = right, optical y = down, optical z = forward (depth)
            ox, oy, oz = det.position.x, det.position.y, det.position.z

            # Remap optical → world frame (REP 103 body/world convention):
            #   world x = forward → +optical_z
            #   world y = left    → -optical_x  (optical x is right)
            #   world z = up      → +optical_y  (optical y is down)
            cx =  oz
            cy = -ox
            cz = oy

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

                self.get_logger().info(
                    f"class='{class_name}' | "
                    f"detection_offset: x={cx:.3f} y={cy:.3f} z={cz:.3f} m  "
                    f"conf={score:.2f}  |  "
                    f"cam_origin_world: x={cam_wx:.3f} y={cam_wy:.3f} z={cam_wz:.3f}  |  "
                    f"obj_origin_world: x={wx:.3f} y={wy:.3f} z={wz:.3f}",
                    throttle_duration_sec=1.0,
                )

                self._process_detection(class_name, wx, wy, wz)

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


def main(args=None):
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