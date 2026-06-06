import os
from servo_control_gui import *
from Widgets.json_widget import *
from PyQt6.QtGui import QFont, QIcon


class robotGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.jsonGUI = jsonGUI() 
        self.servo_control_gui = servoGUI()
        self.initlayout()
        try:
            icon_path = Path(__file__).resolve().parent / "documents" / "robot_control_icon.jpg"
            self.setWindowIcon(QIcon(str(icon_path)))
        except Exception as e:
            print(f"Failed to set window icon: {e}")
            
    def initlayout(self):
        # Wire jsonGUI to read action time from the servo control GUI spinbox
        self.jsonGUI.action_time_source = lambda: self.servo_control_gui.action_time
        self.hlayout = QHBoxLayout()
        self.hlayout.addWidget(self.servo_control_gui)
        self.hlayout.addWidget(self.jsonGUI)
        self.setLayout(self.hlayout)
    def keyPressEvent(self, event):
        # Forward all key events to servoGUI
        QApplication.sendEvent(self.servo_control_gui, event)
        # Optionally, also call default behavior
        super().keyPressEvent(event)


def ros_spin(node):
    # Use a per-node executor if this helper is used; matches other modules.
    executor = rclpy.executors.SingleThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin()
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass

if __name__ == "__main__":
    rclpy.init()
    os.environ["QT_SCALE_FACTOR"] = "0.9"
    app = QApplication(sys.argv)
    app.setDesktopFileName("nubi-control")  # Wayland: matches app_id to nubi-control.desktop for taskbar icon
    robot_control_gui= robotGUI()
    robot_control_gui.showNormal()

    app.exec()
    # Clean up nodes (each GUI class spins its own node thread)
    try:
        robot_control_gui.servo_control_gui.ros_node.destroy_node()
    except Exception:
        pass
    try:
        robot_control_gui.jsonGUI.node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(0)