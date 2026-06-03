#!/usr/bin/env python3

import sys
import threading
import rclpy
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel, 
                             QTextEdit, QPushButton)
from PyQt6.QtCore import Qt, QSettings
from PyQt6.QtGui import QFont

# Import the node and signals from our modified file
import sys
from pathlib import Path

# 1. Resolve the path to the parent directory
# .parent goes up one level. Add more .parent if you need to go higher.
try:
    from subClasses.collisions import LegCommandVerifier, VerifierSignals
except ModuleNotFoundError:
    parent_dir = str(Path(__file__).resolve().parent.parent)
    print("ran directly")
    # 2. Add it to Python's system path
    sys.path.append(parent_dir)
    from subClasses.collisions import LegCommandVerifier, VerifierSignals

class CollisionsWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS 2 Leg Verifier")
        
        # Restore last window position/size
        self.settings = QSettings("NubiRobotics", "LegCommandVerifier")
        geometry = self.settings.value("geometry")
        if geometry:
            self.restoreGeometry(geometry)
        else:
            self.resize(500, 600)

        self.setup_ui()
        self.setup_ros()

    def setup_ui(self):
        layout = QVBoxLayout(self)

        # Title Label
        self.title_lbl = QLabel("Collisions")
        self.title_lbl.setFont(QFont("Arial", 24, QFont.Weight.Bold))
        self.title_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_lbl.setFixedHeight(50)
        layout.addWidget(self.title_lbl)

        # Progress Label
        self.progress_lbl = QLabel("Processed 0/0")
        self.progress_lbl.setFont(QFont("Arial", 12))
        self.progress_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.progress_lbl.setFixedHeight(30)
        layout.addWidget(self.progress_lbl)

        # Publish Status Label
        self.publish_lbl = QLabel("Idle")
        self.publish_lbl.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        self.publish_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.publish_lbl.setFixedHeight(30)
        self.publish_lbl.setStyleSheet("color: #4CAF50;") # Give it a nice green hue
        layout.addWidget(self.publish_lbl)

        # Big Text Box (Expanding)
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setFont(QFont("Consolas", 10))
        layout.addWidget(self.log_box)

        # Proceed Button
        self.proceed_btn = QPushButton("Proceed")
        self.proceed_btn.setFixedHeight(50)
        self.proceed_btn.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        self.proceed_btn.setStyleSheet("background-color: #f44336; color: white; border-radius: 5px;")
        self.proceed_btn.setVisible(False)
        self.proceed_btn.clicked.connect(self.on_proceed_clicked)
        layout.addWidget(self.proceed_btn)

    def setup_ros(self):
        # Setup signals
        self.signals = VerifierSignals()
        self.signals.batch_started.connect(self.on_batch_started)
        self.signals.progress_updated.connect(self.on_progress_updated)
        self.signals.invalid_detected.connect(self.on_invalid_detected)
        self.signals.needs_confirmation.connect(self.on_needs_confirmation)
        self.signals.publishing_status.connect(self.on_publishing_status)

        # Init ROS 2 and spin it in a background thread so the GUI doesn't freeze
        if not rclpy.ok():
            rclpy.init()
        self.ros_node = LegCommandVerifier(signals=self.signals)
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
        self.ros_thread.start()

    # ─── Signal Slots ────────────────────────────────────────────────────────
    
    def on_batch_started(self, total):
        self.progress_lbl.setText(f"Processed 0/{total}")
        self.publish_lbl.setText("Idle")
        self.log_box.clear()
        self.proceed_btn.setVisible(False)

    def on_progress_updated(self, current, total):
        self.progress_lbl.setText(f"Processed {current}/{total}")

    def on_invalid_detected(self, index, angles):
        # Round angles for cleaner printing
        rounded_angles = [round(a, 3) for a in angles]
        self.log_box.append(f'<span style="color:red;"><b>[Index {index}]</b> Invalid Angles:</span> {rounded_angles}')

    def on_needs_confirmation(self):
        self.log_box.append("<br><b>Are you sure you want to continue with all valid angles?</b><br><i>(The invalid ones will be ignored)</i>")
        self.proceed_btn.setVisible(True)

    def on_publishing_status(self, status_msg):
        self.publish_lbl.setText(status_msg)

    # ─── Button Events ───────────────────────────────────────────────────────
    
    def on_proceed_clicked(self):
        self.proceed_btn.setVisible(False)
        self.log_box.append("<br><span style='color:green;'>Proceeding...</span>")
        # Unblock the batch timer thread in the ROS node
        self.ros_node.proceed_event.set()

    # ─── Clean Shutdown ──────────────────────────────────────────────────────
    
    def closeEvent(self, event):
        # Save window state for next time
        self.settings.setValue("geometry", self.saveGeometry())
        
        # Ensure ROS thread doesn't hang if it's currently waiting on the proceed button
        self.ros_node.proceed_event.set()
        
        if rclpy.ok():
            self.ros_node.destroy_node()
            rclpy.shutdown()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = CollisionsWidget()
    widget.show()
    sys.exit(app.exec())