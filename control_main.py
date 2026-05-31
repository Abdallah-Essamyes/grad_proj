#!/usr/bin/env python3
"""Entry point — launches the full robot control GUI."""
import os
import sys

import rclpy
from PyQt6.QtWidgets import QApplication

from robot_control_gui import robotGUI


def main():
    rclpy.init()
    os.environ.setdefault("QT_SCALE_FACTOR", "0.9")
    app = QApplication(sys.argv)
    window = robotGUI()
    window.showNormal()
    app.exec()

    try:
        window.servo_control_gui.ros_node.destroy_node()
    except Exception:
        pass
    try:
        window.jsonGUI.node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(0)


if __name__ == "__main__":
    main()
