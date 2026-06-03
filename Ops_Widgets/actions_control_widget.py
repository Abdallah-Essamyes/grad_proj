"""
ActionsControlWidget
====================
Loads a JSON file containing a list of robot actions (2-D arrays of upper-body
and leg angles) and lets the operator step through them one by one.

JSON format expected:
    [
        {"upper_body": [a0, a1, ...], "lower_body": [b0, b1, ...], "action_time": 1000},
        ...
    ]
``action_time`` is optional (ms); defaults to 1000 ms.

Keyboard shortcuts (only when this widget or any of its children have focus):
    A / Left  → previous action
    D / Right → next action
"""

import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QFileDialog, QMessageBox, QSizePolicy, QFrame, QScrollArea,
)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer
from PyQt6.QtGui import QFont, QKeySequence
from settings.settings import *

_BE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_DEFAULT_ACTION_TIME = 1000  # ms


def _ros_spin(node: Node):
    executor = rclpy.executors.SingleThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin()
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass


class ActionsControlWidget(QWidget):
    """
    Side-panel widget for stepping through a JSON action sequence and
    publishing each action to the robot's ROS topics.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self._actions: list[dict] = []
        self._current_idx: int = 0
        self._json_path: Path | None = None

        # ── ROS setup ───────────────────────────────────────────────────
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("actions_control_widget")
        self._pub_upper = self._node.create_publisher(
            UPPERBODY_MSG_TYPE, UPPERBODY_PUB_TOPIC, UPPERBODY_PUB_QOS
        )
        self._pub_legs = self._node.create_publisher(
            LEGS_MSG_TYPE, LEGS_PUB_TOPIC, LEGS_PUB_QOS
        )
        self._ros_thread = threading.Thread(
            target=_ros_spin, args=(self._node,), daemon=True
        )
        self._ros_thread.start()

        self._init_ui()

    # ------------------------------------------------------------------ #
    #  UI                                                                  #
    # ------------------------------------------------------------------ #

    def _init_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.setSpacing(6)

        # Title
        title = QLabel("Actions Control")
        title.setFont(QFont("Segoe UI", 11, QFont.Weight.Bold))
        outer.addWidget(title)

        # Load button
        load_btn = QPushButton("📂  Load JSON")
        load_btn.setFixedHeight(30)
        load_btn.clicked.connect(self._on_load)
        outer.addWidget(load_btn)

        # File name label
        self._file_label = QLabel("No file loaded")
        self._file_label.setStyleSheet("color: #8b949e; font-size: 10px;")
        self._file_label.setWordWrap(True)
        outer.addWidget(self._file_label)

        # Divider
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet("color: #30363d;")
        outer.addWidget(line)

        # Action counter
        self._counter_label = QLabel("Action: — / —")
        self._counter_label.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        self._counter_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        outer.addWidget(self._counter_label)

        # Prev / Next navigation row
        nav_row = QHBoxLayout()
        self._prev_btn = QPushButton("◀  Prev  (A)")
        self._prev_btn.setFixedHeight(34)
        self._prev_btn.clicked.connect(self._on_prev)
        self._prev_btn.setEnabled(False)

        self._next_btn = QPushButton("Next  (D)  ▶")
        self._next_btn.setFixedHeight(34)
        self._next_btn.clicked.connect(self._on_next)
        self._next_btn.setEnabled(False)

        nav_row.addWidget(self._prev_btn)
        nav_row.addWidget(self._next_btn)
        outer.addLayout(nav_row)

        # Send button
        self._send_btn = QPushButton("▶  Send Current Action")
        self._send_btn.setFixedHeight(32)
        self._send_btn.clicked.connect(self._on_send)
        self._send_btn.setEnabled(False)
        outer.addWidget(self._send_btn)

        # Values display (scrollable)
        self._values_label = QLabel("—")
        self._values_label.setFont(QFont("Monospace", 8))
        self._values_label.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        self._values_label.setWordWrap(True)
        self._values_label.setStyleSheet(
            "background: #161b22; color: #c9d1d9; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 4px;"
        )
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self._values_label)
        scroll.setMinimumHeight(120)
        scroll.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        scroll.setStyleSheet("border: none; background: transparent;")
        outer.addWidget(scroll)

    # ------------------------------------------------------------------ #
    #  Keyboard shortcuts                                                  #
    # ------------------------------------------------------------------ #

    def keyPressEvent(self, event):
        key = event.key()
        if key in (Qt.Key.Key_A, Qt.Key.Key_Left):
            self._on_prev()
            return
        if key in (Qt.Key.Key_D, Qt.Key.Key_Right):
            self._on_next()
            return
        super().keyPressEvent(event)

    # ------------------------------------------------------------------ #
    #  Handlers                                                            #
    # ------------------------------------------------------------------ #

    def _on_load(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Action JSON", str(Path.home()), "JSON Files (*.json)"
        )
        if not path:
            return
        self._load_file(Path(path))

    def _load_file(self, path: Path):
        try:
            with open(path, "r") as f:
                data = json.load(f)
        except Exception as e:
            QMessageBox.critical(self, "Load Error", f"Failed to load JSON:\n{e}")
            return

        if not isinstance(data, list) or not data:
            QMessageBox.warning(
                self, "Invalid Format",
                "Expected a non-empty JSON array of action objects."
            )
            return

        # Validate entries
        for i, entry in enumerate(data):
            if not isinstance(entry, dict):
                QMessageBox.warning(self, "Invalid Format", f"Entry {i} is not an object.")
                return
            if "upper_body" not in entry or "lower_body" not in entry:
                QMessageBox.warning(
                    self, "Invalid Format",
                    f"Entry {i} is missing 'upper_body' or 'lower_body'."
                )
                return

        self._actions = data
        self._json_path = path
        self._current_idx = 0
        self._file_label.setText(path.name)
        self._update_display()
        self._prev_btn.setEnabled(True)
        self._next_btn.setEnabled(True)
        self._send_btn.setEnabled(True)

    def _on_prev(self):
        if not self._actions:
            return
        self._current_idx = (self._current_idx - 1) % len(self._actions)
        self._update_display()

    def _on_next(self):
        if not self._actions:
            return
        self._current_idx = (self._current_idx + 1) % len(self._actions)
        self._update_display()

    def _on_send(self):
        if not self._actions:
            return
        self._send_action(self._actions[self._current_idx])

    # ------------------------------------------------------------------ #
    #  Display                                                             #
    # ------------------------------------------------------------------ #

    def _update_display(self):
        if not self._actions:
            self._counter_label.setText("Action: — / —")
            self._values_label.setText("—")
            return

        total = len(self._actions)
        idx = self._current_idx
        self._counter_label.setText(f"Action: {idx + 1} / {total}")

        action = self._actions[idx]
        upper = action.get("upper_body", [])
        lower = action.get("lower_body", [])
        t = action.get("action_time", _DEFAULT_ACTION_TIME)

        upper_str = ", ".join(str(v) for v in upper)
        lower_str = ", ".join(str(v) for v in lower)
        text = (
            f"<b>action_time:</b> {t} ms<br>"
            f"<b>upper_body:</b><br>&nbsp;&nbsp;[{upper_str}]<br>"
            f"<b>lower_body:</b><br>&nbsp;&nbsp;[{lower_str}]"
        )
        self._values_label.setText(text)
        self._values_label.setTextFormat(Qt.TextFormat.RichText)

    # ------------------------------------------------------------------ #
    #  ROS publishing                                                      #
    # ------------------------------------------------------------------ #

    def _send_action(self, action: dict):
        upper = list(action.get("upper_body", []))
        lower = list(action.get("lower_body", []))
        t = int(action.get("action_time", _DEFAULT_ACTION_TIME))

        # Format mirrors jsonGUI.do_action():
        #   upperbody_command: [u0..u6, std0..std3=90, playtime]  (12 elements)
        #   legs_command:      [l0..l11, playtime]                 (13 elements)
        upper_cmd = Int16MultiArray()
        lower_cmd = Int16MultiArray()
        upper_cmd.data = upper + [90, 90, 90, 90, t]
        lower_cmd.data = lower + [t]

        self._pub_upper.publish(upper_cmd)
        time.sleep(0.1)
        self._pub_legs.publish(lower_cmd)

    # ------------------------------------------------------------------ #
    #  Cleanup                                                             #
    # ------------------------------------------------------------------ #

    def closeEvent(self, event):
        try:
            self._node.destroy_node()
        except Exception:
            pass
        super().closeEvent(event)
