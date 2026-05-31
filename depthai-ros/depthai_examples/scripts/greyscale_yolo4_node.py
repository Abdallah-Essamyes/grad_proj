#!/usr/bin/env python3
"""
CPU-based YOLOv4 inference on greyscale images from /right/image_rect.

Model files are expected at:
  ~/.ros/yolov4/yolov4.cfg
  ~/.ros/yolov4/yolov4.weights
  ~/.ros/yolov4/coco.names  (one class name per line)

Download:
  mkdir -p ~/.ros/yolov4
  wget -P ~/.ros/yolov4 https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4.cfg
  wget -P ~/.ros/yolov4 https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
  wget -P ~/.ros/yolov4 https://raw.githubusercontent.com/AlexeyAB/darknet/master/data/coco.names

Alternatively override paths via ROS parameters:
  cfg_path, weights_path, names_path, input_size (default 416), conf_threshold, nms_threshold
"""

import math
import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
)

try:
    from interactive_markers.interactive_marker_server import InteractiveMarkerServer
    _HAS_IM = True
except ImportError:
    _HAS_IM = False


_DEFAULT_CFG = str(Path.home() / ".ros" / "yolov4" / "yolov4.cfg")
_DEFAULT_WEIGHTS = str(Path.home() / ".ros" / "yolov4" / "yolov4.weights")
_DEFAULT_NAMES = str(Path.home() / ".ros" / "yolov4" / "coco.names")


class GreyscaleYolo4Node(Node):
    """Run YOLOv4 on CPU using OpenCV DNN on /right/image_rect (greyscale)."""

    def __init__(self):
        super().__init__("greyscale_yolo4_node")

        # ── Parameters ───────────────────────────────────────────────────────────
        self.declare_parameter("cfg_path",       _DEFAULT_CFG)
        self.declare_parameter("weights_path",   _DEFAULT_WEIGHTS)
        self.declare_parameter("names_path",     _DEFAULT_NAMES)
        self.declare_parameter("input_size",     416)    # 416 or 608
        self.declare_parameter("conf_threshold", 0.40)
        self.declare_parameter("nms_threshold",  0.45)
        self.declare_parameter("input_topic",    "/right/image_rect")
        self.declare_parameter("num_threads",    4)      # OpenCV DNN thread count

        cfg_path      = self.get_parameter("cfg_path").value
        weights_path  = self.get_parameter("weights_path").value
        names_path    = self.get_parameter("names_path").value
        self._input_size    = int(self.get_parameter("input_size").value)
        self._conf_thr      = float(self.get_parameter("conf_threshold").value)
        self._nms_thr       = float(self.get_parameter("nms_threshold").value)
        input_topic         = self.get_parameter("input_topic").value
        num_threads         = int(self.get_parameter("num_threads").value)

        # ── Class names ───────────────────────────────────────────────────────────
        self._class_names = self._load_names(names_path)
        self.get_logger().info(
            f"Loaded {len(self._class_names)} class names from '{names_path}'"
        )

        # ── OpenCV DNN network ────────────────────────────────────────────────────
        if not os.path.isfile(cfg_path):
            self.get_logger().error(f"YOLOv4 cfg not found: {cfg_path}")
            raise FileNotFoundError(cfg_path)
        if not os.path.isfile(weights_path):
            self.get_logger().error(f"YOLOv4 weights not found: {weights_path}")
            raise FileNotFoundError(weights_path)

        self.get_logger().info(f"Loading YOLOv4 from '{weights_path}' …")
        self._net = cv2.dnn.readNetFromDarknet(cfg_path, weights_path)
        self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self._net.setNumThreads(num_threads)

        self._output_layers = [
            self._net.getLayerNames()[i - 1]
            for i in self._net.getUnconnectedOutLayers().flatten()
        ]
        self.get_logger().info(
            f"YOLOv4 ready. Input size: {self._input_size}px, "
            f"conf≥{self._conf_thr}, nms≤{self._nms_thr}, threads={num_threads}"
        )

        # ── ROS I/O ───────────────────────────────────────────────────────────────
        self._bridge = CvBridge()

        self._det_pub = self.create_publisher(Detection2DArray, "~/detections", 10)
        self._vis_pub = self.create_publisher(Image, "~/debug_image", 10)

        self._sub = self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=2,
            ),
        )

        # Optional interactive markers
        if _HAS_IM:
            self._im_server = InteractiveMarkerServer(self, "yolo4_detections")
        else:
            self._im_server = None

        self.get_logger().info(
            f"Subscribed to '{input_topic}'. Publishing detections on '~/detections'."
        )

    # ──────────────────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _load_names(path: str) -> list[str]:
        """Load COCO class names, one per line. Returns empty list on error."""
        try:
            with open(path) as f:
                return [line.strip() for line in f if line.strip()]
        except FileNotFoundError:
            return []

    def _to_bgr(self, ros_img: Image) -> np.ndarray:
        """Convert a ROS Image (mono8 or bgr8/rgb8) to BGR uint8 ndarray."""
        enc = ros_img.encoding.lower()
        if enc in ("mono8", "8uc1"):
            grey = self._bridge.imgmsg_to_cv2(ros_img, desired_encoding="mono8")
            return cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR)
        if enc in ("mono16", "16uc1"):
            grey16 = self._bridge.imgmsg_to_cv2(ros_img, desired_encoding="mono16")
            grey8 = cv2.convertScaleAbs(grey16, alpha=255.0 / 65535.0)
            return cv2.cvtColor(grey8, cv2.COLOR_GRAY2BGR)
        return self._bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")

    def _run_yolo(self, bgr: np.ndarray):
        """Run YOLOv4 and return (boxes, confidences, class_ids) after NMS."""
        h, w = bgr.shape[:2]
        blob = cv2.dnn.blobFromImage(
            bgr,
            scalefactor=1.0 / 255.0,
            size=(self._input_size, self._input_size),
            swapRB=False,  # already BGR
            crop=False,
        )
        self._net.setInput(blob)
        layer_outputs = self._net.forward(self._output_layers)

        boxes, confidences, class_ids = [], [], []
        for output in layer_outputs:
            for detection in output:
                scores = detection[5:]
                class_id = int(np.argmax(scores))
                confidence = float(scores[class_id])
                if confidence < self._conf_thr:
                    continue
                cx = int(detection[0] * w)
                cy = int(detection[1] * h)
                bw = int(detection[2] * w)
                bh = int(detection[3] * h)
                x1 = max(0, cx - bw // 2)
                y1 = max(0, cy - bh // 2)
                boxes.append([x1, y1, bw, bh])
                confidences.append(confidence)
                class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(
            boxes, confidences, self._conf_thr, self._nms_thr
        )
        if len(indices) == 0:
            return [], [], []

        indices = indices.flatten()
        return (
            [boxes[i] for i in indices],
            [confidences[i] for i in indices],
            [class_ids[i] for i in indices],
        )

    def _build_detection_array(
        self,
        boxes,
        confidences,
        class_ids,
        stamp,
        frame_id: str,
    ) -> Detection2DArray:
        arr = Detection2DArray()
        arr.header.stamp = stamp
        arr.header.frame_id = frame_id

        for (x1, y1, bw, bh), conf, cid in zip(boxes, confidences, class_ids):
            det = Detection2D()
            det.header = arr.header

            bbox = BoundingBox2D()
            bbox.center.position.x = float(x1 + bw / 2)
            bbox.center.position.y = float(y1 + bh / 2)
            bbox.size_x = float(bw)
            bbox.size_y = float(bh)
            det.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(cid)
            hyp.hypothesis.score = conf
            det.results.append(hyp)

            arr.detections.append(det)

        return arr

    def _draw_debug(self, bgr: np.ndarray, boxes, confidences, class_ids) -> np.ndarray:
        vis = bgr.copy()
        for (x1, y1, bw, bh), conf, cid in zip(boxes, confidences, class_ids):
            name = (
                self._class_names[cid]
                if cid < len(self._class_names)
                else str(cid)
            )
            label = f"{name} {conf:.2f}"
            x2, y2 = x1 + bw, y1 + bh
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                vis, label, (x1, max(y1 - 6, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
            )
        return vis

    def _update_markers(self, boxes, confidences, class_ids, frame_id: str):
        if self._im_server is None:
            return
        self._im_server.clear()
        for idx, ((x1, y1, bw, bh), conf, cid) in enumerate(
            zip(boxes, confidences, class_ids)
        ):
            name = (
                self._class_names[cid]
                if cid < len(self._class_names)
                else str(cid)
            )
            marker_name = f"yolo_{idx}_{name}"

            im = InteractiveMarker()
            im.header.frame_id = frame_id
            im.header.stamp = self.get_clock().now().to_msg()
            im.name = marker_name
            im.description = f"{name}\n{conf:.2f}"
            # Place in 2D image plane (z=0) at bbox centre; no depth available
            im.pose.position.x = float(x1 + bw / 2)
            im.pose.position.y = float(y1 + bh / 2)
            im.pose.position.z = 0.0
            im.scale = float(max(bw, bh))

            sphere = Marker()
            sphere.type = Marker.CUBE
            sphere.scale = Vector3(x=float(bw), y=float(bh), z=1.0)
            sphere.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.5)

            ctrl = InteractiveMarkerControl()
            ctrl.name = "visual"
            ctrl.always_visible = True
            ctrl.markers.append(sphere)
            ctrl.interaction_mode = InteractiveMarkerControl.BUTTON
            im.controls.append(ctrl)

            self._im_server.insert(
                im, feedback_callback=lambda fb, n=name, c=conf: self.get_logger().info(
                    f"[CLICK] {n} ({c:.2f})"
                )
            )
        self._im_server.applyChanges()

    # ──────────────────────────────────────────────────────────────────────────────
    # Callback
    # ──────────────────────────────────────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        try:
            bgr = self._to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}", throttle_duration_sec=5.0)
            return

        boxes, confidences, class_ids = self._run_yolo(bgr)

        # ── Publish Detection2DArray ───────────────────────────────────────────
        det_arr = self._build_detection_array(
            boxes, confidences, class_ids, msg.stamp, msg.header.frame_id
        )
        self._det_pub.publish(det_arr)

        # ── Publish annotated debug image ──────────────────────────────────────
        if self._vis_pub.get_subscription_count() > 0:
            vis = self._draw_debug(bgr, boxes, confidences, class_ids)
            debug_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            debug_msg.header = msg.header
            self._vis_pub.publish(debug_msg)

        # ── Interactive markers ────────────────────────────────────────────────
        self._update_markers(boxes, confidences, class_ids, msg.header.frame_id)

        # ── Log detections ─────────────────────────────────────────────────────
        if boxes:
            names = [
                self._class_names[cid] if cid < len(self._class_names) else str(cid)
                for cid in class_ids
            ]
            self.get_logger().info(
                f"Detected {len(boxes)} object(s): "
                + ", ".join(f"{n}({c:.2f})" for n, c in zip(names, confidences)),
                throttle_duration_sec=1.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = GreyscaleYolo4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
