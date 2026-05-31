#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from depthai_ros_msgs.msg import SpatialDetectionArray
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import ColorRGBA, Header
import numpy as np
import time
from cv_bridge import CvBridge
import math
import json
from pathlib import Path
import tf2_ros
from geometry_msgs.msg import TransformStamped

_JSON_PATH = Path(__file__).resolve().parent.parent / "Jsons" / "coco_dataset.json"
with open(_JSON_PATH, "r") as f:
    COCO_dict = json.load(f)  # load JSON from the file
    print("opened coco dataset json with", len(COCO_dict), "entries")

# Camera base frame — detections arrive in this frame; we transform to map/odom.
# The oak-d-base-frame is used by the depthai driver.
_CAMERA_FRAME = "oak-d-base-frame"
_WORLD_FRAME = "map"


class camera_handler(Node):
    def __init__(self):
        super().__init__('camera_handler')

        # --- Parameters ---
        self.input_topic = '/color/yolov4_Spatial_detections'

        # TF buffer + listener to look up camera pose in the world frame
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Interactive marker server
        self._im_server = InteractiveMarkerServer(self, "detected_objects")

        # Subscriber
        self.sub = self.create_subscription(
            SpatialDetectionArray,
            self.input_topic,
            self.detections_callback,
            10
        )
        self.get_logger().info("DepthAI object reader started (with interactive markers)")

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    def _try_get_camera_transform(self):
        """Return the camera-to-world TransformStamped, or None if unavailable."""
        try:
            return self._tf_buffer.lookup_transform(
                _WORLD_FRAME,
                _CAMERA_FRAME,
                rclpy.time.Time(),          # latest available
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    @staticmethod
    def _transform_point(tf_stamped: TransformStamped, x: float, y: float, z: float):
        """Apply a TransformStamped translation+rotation (quaternion) to a point."""
        import numpy as np
        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        # Rotate point by quaternion then add translation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        # Using quaternion rotation formula: p' = q * p * q^-1
        # Efficient version:
        ix = qw * x + qy * z - qz * y
        iy = qw * y + qz * x - qx * z
        iz = qw * z + qx * y - qy * x
        iw = -qx * x - qy * y - qz * z
        rx = ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy)
        ry = iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz)
        rz = iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx)
        return rx + t.x, ry + t.y, rz + t.z

    def _make_interactive_marker(
        self,
        name: str,
        label: str,
        wx: float,
        wy: float,
        wz: float,
        frame_id: str,
    ) -> InteractiveMarker:
        """Build an interactive marker with a sphere + text + BUTTON control."""
        im = InteractiveMarker()
        im.header.frame_id = frame_id
        im.header.stamp = self.get_clock().now().to_msg()
        im.name = name
        im.description = f"{label}\n[Grab]"
        im.pose.position.x = wx
        im.pose.position.y = wy
        im.pose.position.z = wz
        im.scale = 0.25

        # ── Visual sphere ──────────────────────────────────────────────
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale = Vector3(x=0.12, y=0.12, z=0.12)
        sphere.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.85)

        vis_ctrl = InteractiveMarkerControl()
        vis_ctrl.name = "visual"
        vis_ctrl.always_visible = True
        vis_ctrl.markers.append(sphere)
        # BUTTON interaction_mode: click triggers feedback callback
        vis_ctrl.interaction_mode = InteractiveMarkerControl.BUTTON
        im.controls.append(vis_ctrl)

        return im

    # ------------------------------------------------------------------ #
    #  Callback                                                            #
    # ------------------------------------------------------------------ #

    def detections_callback(self, msg: SpatialDetectionArray):
        # Try to get the camera→world transform once per batch
        tf_stamped = self._try_get_camera_transform()
        if tf_stamped is None:
            # Can't place markers in world frame — log once and skip markers
            self.get_logger().warn(
                f"Could not find TF from '{_CAMERA_FRAME}' to '{_WORLD_FRAME}'. "
                "Markers will not be published.",
                throttle_duration_sec=5.0,
            )
            # Still log detections
            for det in msg.detections:
                if not det.results:
                    continue
                class_id = int(det.results[0].class_id)
                score = det.results[0].score
                class_name = COCO_dict.get(str(class_id + 1), "unknown")
                x_m, y_m, z_m = det.position.x, det.position.y, det.position.z
                angle_deg = math.degrees(math.atan2(x_m, z_m))
                self.get_logger().info(
                    f"{class_name} | z: {z_m:.2f} m | x: {x_m:.2f} m | "
                    f"{angle_deg:.1f}° | confidence {score}, class_id {class_id}",
                    throttle_duration_sec=1.0,
                )
            return

        # Determine which frame to use for markers
        marker_frame = _WORLD_FRAME

        seen_names: set[str] = set()

        for det in msg.detections:
            if not det.results:
                continue
            class_id = int(det.results[0].class_id)
            score = det.results[0].score
            class_name = COCO_dict.get(str(class_id + 1), "unknown")

            x_m = det.position.x
            z_m = det.position.z
            y_m = det.position.y
            true_distance = math.sqrt(x_m ** 2 + y_m ** 2 + z_m ** 2)
            angle_rad = math.atan2(x_m, z_m)
            angle_deg = math.degrees(angle_rad)

            self.get_logger().info(
                f"{class_name} | z: {z_m:.2f} m | x: {x_m:.2f} m | "
                f"{angle_deg:.1f}° | confidence {score}, class_id {class_id}",
                throttle_duration_sec=1.0,
            )

            # Transform detection position to world frame
            wx, wy, wz = self._transform_point(tf_stamped, x_m, y_m, z_m)

            # Use a stable marker name: class + rounded world position
            marker_name = (
                f"{class_name}_{wx:.1f}_{wy:.1f}_{wz:.1f}"
                .replace(" ", "_").replace("-", "n").replace(".", "p")
            )
            seen_names.add(marker_name)

            im = self._make_interactive_marker(
                name=marker_name,
                label=class_name,
                wx=wx,
                wy=wy,
                wz=wz,
                frame_id=marker_frame,
            )
            self._im_server.insert(im, feedback_callback=self._on_marker_feedback)

        self._im_server.applyChanges()

    # ------------------------------------------------------------------ #
    #  Marker feedback                                                     #
    # ------------------------------------------------------------------ #

    def _on_marker_feedback(self, feedback):
        """Called when the user clicks (BUTTON_CLICK) a marker in RViz."""
        from visualization_msgs.msg import InteractiveMarkerFeedback
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            pos = feedback.pose.position
            self.get_logger().info(
                f"[GRAB requested] Marker: '{feedback.marker_name}' "
                f"at ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) in '{feedback.header.frame_id}'"
            )
            # TODO: publish a grab goal/action here when the arm planner is ready


def main(args=None):
    rclpy.init(args=args)
    node = camera_handler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

