"""
ops_main.py — Entry point for the Robot Operations GUI.

Usage:
    python ops_main.py
"""

import sys
import os

from PyQt6.QtWidgets import QApplication
from robot_ops_gui import RobotOpsGUI


def main():
    os.environ.setdefault("QT_SCALE_FACTOR", "1.0")
    app = QApplication(sys.argv)
    window = RobotOpsGUI()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
