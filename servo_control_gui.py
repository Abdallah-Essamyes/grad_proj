from subClasses.ros_node import (ServoControlROSNode,
                                  servo_legs_pub_topic, servo_upperbody_pub_topic,
                                  Legs, Upperbody)
from Widgets.servo_widget import servo_control_subWidget
from Widgets.torque_widget import torque_control_subWidget
from Widgets.SidebarPanel import SidebarPanel
from subClasses.position_manager import (servo_widget_width, load_servo_positions,
                                          save_servo_positions, X_GROUP_FOR_ID,
                                          Y_GROUP_FOR_ID, return_servo_subWidgets_positions)
from Widgets.status_table import StatusReferenceTable
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QSlider, QSpinBox, QFrame, QCheckBox, QDoubleSpinBox
from PyQt6.QtCore import QRect, Qt, QObject, pyqtSignal, QTimer, QSize, QEvent, QSettings
from PyQt6.QtGui import QIcon, QKeySequence, QPixmap, QPalette, QBrush, QPainter, QColor
import sys
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32,Int16MultiArray
from pathlib import Path
from subClasses.Command_Array import *
from subClasses.Servo_Pair import Servo_Pair
from subClasses.constants import *
#the id of the servos in this list should be at its respective
#index in the low level handling
#put the required hotkey as well
#add positions for the subwidgets in the return_servo_subWidgets_positions function in servo_subclasses.py
command_array(name = Legs,
              ids_array= [16, 6 , 7 , 8, 10 , 9, 17,  11, 12, 13, 15, 14],
              hotkey_array = ['q','w','e','r','t','y','u','i','o','p','[',']'],
              pub_topic=servo_legs_pub_topic,
              pub_type=Int16MultiArray)

command_array(name = Upperbody,
              ids_array = [0, 1, 2, 3, 4, 18, 19, 101, 102, 103, 104],
              hotkey_array = ['a','s','d','f','g','h','j','k','l',';',"'"],
              pub_topic=servo_upperbody_pub_topic,
              pub_type=Int16MultiArray)


all_commands_dict = command_array.all_commands_dict

def ros_spin(node):
    # Spin this node with its own SingleThreadedExecutor to avoid
    # interfering with other spins in separate threads.
    executor = rclpy.executors.SingleThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin()
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass

class servoGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.ros_node = ServoControlROSNode()
        # step increment for servo changes (0-90)
        self.step = 10
        # torque verification polling state
        self._torque_verify_count = 0
        self._torque_response_count = 0      # responses received during this verification round
        self._torque_verify_timer = QTimer(self)
        self._torque_verify_timer.setInterval(1000)
        self._torque_verify_timer.timeout.connect(self._torque_verify_tick)
        # fired 1 s after the 5th poll; if we got <5 responses, reset to None
        self._torque_timeout_timer = QTimer(self)
        self._torque_timeout_timer.setSingleShot(True)
        self._torque_timeout_timer.setInterval(1000)
        self._torque_timeout_timer.timeout.connect(self._torque_verify_timeout)
        self.initlayout()
        self.place_servoSubwidgets()
        # Install app-level event filter so +/- work from any widget
        QApplication.instance().installEventFilter(self)
        self.ros_node.angles_callback_signal.connect(self.handle_angles_callback)
        self.ros_node.torque_feedback_signal.connect(self.handle_torque_feedback)
        self.ros_node.motor_status_signal.connect(self.handle_motor_status_callback)
        self.ros_thread = threading.Thread(
        target=ros_spin,
        args=(self.ros_node,),
        daemon=True)
        self.ros_thread.start()
        # Ask for torque once shortly after startup (give ROS time to connect)
        QTimer.singleShot(800, self.ros_node.request_torque_status)
        try:
            icon_path = Path(__file__).resolve().parent / "documents" / "robot_control_icon.jpg"
            self.setWindowIcon(QIcon(str(icon_path)))
        except Exception as e:
            print(f"Failed to set window icon: {e}")

    def initlayout(self):
        # ===== Load background image =====
        import os as _os_bg
        _bg_path = _os_bg.path.join(_os_bg.path.dirname(_os_bg.path.abspath(__file__)), "documents", "robot_higher_res_cropped.png")
        self.bg = QPixmap(_bg_path)
        # Expand the window a bit vertically so bottom widgets are not clipped
        extra_height = -60
        img_size = self.bg.size()
        img_size.setHeight(img_size.height() + extra_height)
        # Add a white sidebar to the right for the reference tables
        sidebar_width = 210
        self._img_width = img_size.width() - 70
        self._img_height = img_size.height()
        total_width = img_size.width() + sidebar_width
        self.setFixedSize(total_width, img_size.height())
        self.setAutoFillBackground(True)

    def paintEvent(self, event):
        painter = QPainter(self)
        # Draw the robot photo on the left
        painter.drawPixmap(0, 0, self._img_width, self._img_height, self.bg)
        # Fill the sidebar with white
        painter.fillRect(self._img_width, 0,
                         self.width() - self._img_width, self.height(),
                         QColor("white"))
        painter.end()

    def mousePressEvent(self, event):
        # Steal focus from any spinbox/textbox so hotkeys work immediately
        self.setFocus()
        super().mousePressEvent(event)

    def eventFilter(self, obj, event):
        # +/- step adjustment from anywhere in the app
        if event.type() == QEvent.Type.KeyPress:
            key = event.key()
            if key in (Qt.Key.Key_Plus, Qt.Key.Key_Equal):
                new_step = min(90, self.step + 10)
                self.step_spinBox.setValue(new_step)
                return True
            if key == Qt.Key.Key_Minus:
                new_step = max(0, self.step - 10)
                self.step_spinBox.setValue(new_step)
                return True
        if obj is getattr(self, '_action_time_spinbox_ref', None):
            if event.type() == QEvent.Type.KeyPress:
                if event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter, Qt.Key.Key_Escape):
                    self.setFocus()
                    return True
        return super().eventFilter(obj, event)

    def place_servoSubwidgets(self):        
        servo_control_subWidget.parent=self
        servo_control_subWidget.width = servo_widget_width
        torque_control_subWidget.parent = self
        # Store centerX and current position arrays for drag/link/save logic
        self._centerx = int(self.bg.size().width() / 2) - 15
        self._x_shifts, self._y_values = load_servo_positions()
        positions = {i: (self._centerx + self._x_shifts[i], self._y_values[i])
                     for i in self._x_shifts}
        self.servo_control_subWidgets_dict:dict[int,servo_control_subWidget] = dict()

        for command_array in (all_commands_dict.values()):
            for servo_id, servo_key in zip(command_array.ids_array, command_array.hotkey_array):
                # pass command array name to subwidget so it knows its group
                self.servo_control_subWidgets_dict[servo_id] = servo_control_subWidget(
                    hotkey = str(servo_key),
                    id = servo_id,
                    command_name = command_array.name,
                    display_name = STD_SERVO_DISPLAY.get(servo_id),
                    show_torque = servo_id not in STD_SERVO_IDS)
                self.servo_control_subWidgets_dict[servo_id].update_angle_signal.connect(
                    self.update_servo_position)
                self.servo_control_subWidgets_dict[servo_id].position_changed_signal.connect(
                    self.handle_widget_dragged)
                self.servo_control_subWidgets_dict[servo_id].move(positions[servo_id][0],
                                                                  positions[servo_id][1])
                # std servos start at 90 (mid-range) unless specified in a pair
                if servo_id in STD_SERVO_IDS:
                    # 1. Default start angle for independent standard servos
                    start_angle = 90
                    
                    # 2. Check if the servo is part of a pair
                    for pair in STD_SERVO_MOTION_PAIRS:
                        if servo_id == pair.servo_left_id:
                            start_angle = pair.servo_left_start_angle
                            break
                        elif servo_id == pair.servo_right_id:
                            start_angle = pair.servo_right_start_angle
                            break
                            
                    # 3. Apply the calculated start angle
                    self.servo_control_subWidgets_dict[servo_id].set_angle(start_angle) 


        xTorque,Ytorque = 50,25
        self.torque_lock_widget = torque_control_subWidget(torque_hotkey)
        self.torque_lock_widget.move(xTorque,Ytorque)
        self.torque_lock_widget.toggle_requested.connect(self.toggle_torque)
        self.torque_lock_widget.set_on_requested.connect(self.set_torque_on)
        self.torque_lock_widget.set_off_requested.connect(self.set_torque_off)
        self.status_ref_table = StatusReferenceTable(parent=self)
        self.status_ref_table.move(self._img_width - 240, 10)
        self.status_ref_table.show()
            # ── Sidebar panel ─────────────────────────────────────────────────────────
        sidebar_width = self.width() - self._img_width
        sidebar_top   = self.status_ref_table.y() + self.status_ref_table.height() + 10

        self.sidebar = SidebarPanel(panel_width=sidebar_width, parent=self)
        self.sidebar.move(self._img_width, sidebar_top)
        self.sidebar.show()

        # Connect outward signals to their targets
        self.sidebar.ask_status_requested.connect(self.ros_node.request_status)
        self.sidebar.ask_torque_requested.connect(self.ros_node.request_torque_status)
        self.sidebar.reinitialize_requested.connect(self.ros_node.reinitialize_servos)
        self.sidebar.clear_errors_requested.connect(self.ros_node.reset_error)
        self.sidebar.status_poll_tick.connect(self.ros_node.request_status)
        self.sidebar.torque_poll_tick.connect(self.ros_node.request_torque_status)
        self.sidebar.step_changed.connect(lambda v: setattr(self, 'step', v))
        self.sidebar.action_time_changed.connect(lambda v: setattr(self, 'action_time', v))

        # Keep a reference to the action_time spinbox so the event filter still works
        self._action_time_spinbox_ref = self.sidebar.action_time_spinBox
        self.sidebar.action_time_spinBox.installEventFilter(self)

        # Keep step_spinBox accessible from keyPressEvent (for +/- keys)
        self.step_spinBox = self.sidebar.step_spinBox
        #used for shifting incrementing/decrementing
        self.increment = True

    def handle_motor_status_callback(self, data: list):
        """Unpack nubi_response status payload: servo n -> error=data[2n], detail=data[2n+1].
        Standard (non-Herkulex) servos are excluded — they have no status register."""
        for servo_id, widget in self.servo_control_subWidgets_dict.items():
            if servo_id in STD_SERVO_IDS:
                continue  # not a Herkulex servo — no status to read
            err_idx = 2 * servo_id
            det_idx = 2 * servo_id + 1
            if det_idx < len(data):
                widget.set_status(data[err_idx], data[det_idx])

    #ros node subscribe callbacks
    def handle_angles_callback(self, name:str, angles_list:list[int]):
        ids = all_commands_dict[name].ids_array
        for i, id in enumerate(ids):
            if id in STD_SERVO_IDS:
                continue  # std servos have no position feedback from STM
            if i >= len(angles_list):
                break
            self.servo_control_subWidgets_dict[id].set_angle(angles_list[i])

    def _torque_verify_tick(self):
        """Poll torque status once per second, up to 5 times after a torque toggle."""
        if self._torque_verify_count <= 0:
            self._torque_verify_timer.stop()
            return
        self.ros_node.request_torque_status()
        self._torque_verify_count -= 1
        if self._torque_verify_count <= 0:
            self._torque_verify_timer.stop()
            # Give 1 extra second for the last response to arrive
            self._torque_timeout_timer.start()

    def _torque_verify_timeout(self):
        """Called 1 s after the 5th poll. If we didn't get all 5 responses, reset to None."""
        if self._torque_response_count < 5:
            print(f"Torque verification failed: only {self._torque_response_count}/5 responses received. Resetting to None.")
            self.torque_lock_widget.set_torque_state(None)

    def handle_torque_feedback(self, torque_list: list):
        """Handle torque feedback (list of 20 ints 0/1) from status_response."""
        if not torque_list:
            return
        self._torque_response_count += 1
        # Use servo 0 as representative state (all servos toggled together)
        torque_bool = bool(torque_list[0])
        self.torque_lock_widget.set_torque_state(torque_bool)
        # Update each servo subwidget's individual torque indicator
        for servo_id, widget in self.servo_control_subWidgets_dict.items():
            if servo_id < len(torque_list):
                widget.set_torque(bool(torque_list[servo_id]))

    def check_torque_timeout(self):
        # Replaced by polling; kept as no-op for compatibility
        pass

    

    def get_all_legs_angles(self):
        try:
            return [int(self.servo_control_subWidgets_dict[id].get_angle()) for id in all_commands_dict[Legs].ids_array]
        except ValueError as e:
            print(f"Error getting legs angles: {e}, check that the angles are being read and are not None")
            return False

    def get_all_upperbody_angles(self):
        try:
            return [int(self.servo_control_subWidgets_dict[id].get_angle()) for id in all_commands_dict[Upperbody].ids_array]
        except ValueError as e:
            print(f"Error getting upperbody angles: {e}, check that the angles are being read and are not None")
            return False

    def get_all_servo_angles_in_same_command_array(self,command_array:command_array):
        try:
            return [int(self.servo_control_subWidgets_dict[id].get_angle()) for id in command_array.ids_array]
        except ValueError as e:
            print(f"Error getting {command_array.name} angles: {e}, check that the angles are being read and are not None")
            return False

    def _send_torque(self, new_state: bool):
        self.torque_lock_widget.turn_blue()   # visual feedback: request sent
        self.ros_node.publish_torque(new_state)
        print(f"Torque Lock: {new_state}")
        # Reset counts and start verification: poll 5 times, once per second
        self._torque_verify_count = 5
        self._torque_response_count = 0
        self._torque_timeout_timer.stop()
        self._torque_verify_timer.start()

    def toggle_torque(self):
        new_state = not self.torque_lock_widget.torque_lock_status
        self._send_torque(new_state)

    def set_torque_on(self):
        self._send_torque(True)

    def set_torque_off(self):
        self._send_torque(False)

    def keyPressEvent(self,event):
        #always returns higher case
        key_pressed = QKeySequence(event.key()).toString().lower()
        print(f"Key pressed: {key_pressed}")
        if key_pressed == "shift":
            self.increment = not self.increment
            for servo_widget in self.servo_control_subWidgets_dict.values():
                servo_widget.switch_sign()
            return

        if key_pressed in ("+", "="):
            new_step = min(90, self.step + 10)
            self.step_spinBox.setValue(new_step)
            return

        if key_pressed == "-":
            new_step = max(0, self.step - 10)
            self.step_spinBox.setValue(new_step)
            return
        
        if key_pressed == self.torque_lock_widget.toggle_key:
            self.toggle_torque()
            
        for servo_widget in self.servo_control_subWidgets_dict.values():            
            #note its is known that the aligning the axis correctly
            #results in the servos in mirrored positions to rotate in opposite directions
            #this can be handled if desired
            if key_pressed == servo_widget.hotkey:
                if self.increment:
                    servo_widget.increment()
                else:
                    servo_widget.decrement()                    

    def handle_widget_dragged(self, widget_id: int, new_abs_x: int, new_abs_y: int):
        """Move linked widgets and persist positions after a drag.

        Servos sharing the same x_shift column move together horizontally;
        servos sharing the same y row move together vertically.
        """
        new_x_shift = new_abs_x - self._centerx
        delta_x = new_x_shift - self._x_shifts[widget_id]
        delta_y = new_abs_y  - self._y_values[widget_id]

        # Collect all IDs that need updating (union of x-group and y-group)
        x_group = X_GROUP_FOR_ID.get(widget_id, [widget_id])
        y_group = Y_GROUP_FOR_ID.get(widget_id, [widget_id])
        all_affected = set(x_group) | set(y_group)

        # Update dicts first so every move() call uses consistent values
        for sid in x_group:
            self._x_shifts[sid] += delta_x
        for sid in y_group:
            self._y_values[sid] += delta_y

        # Move every affected widget to its new position
        for sid in all_affected:
            w = self.servo_control_subWidgets_dict.get(sid)
            if w:
                w.move(self._centerx + self._x_shifts[sid], self._y_values[sid])

        save_servo_positions(self._x_shifts, self._y_values)

    #sign is 1 or -1
    def update_servo_position(self,servo_widget:servo_control_subWidget,sign:int):
        try:
            all_commands_dict[servo_widget.name]
        except Exception:
            print(f"Servo widget {servo_widget.id} has no valid command name; cannot publish")
            return

        # Herkulex servos: only this widget's angle is needed — skip the full-array read
        # (other widgets may be None/"N" and would cause int() failures)
        
        current = servo_widget.angle  # None if unpowered/unread
        if current is None:
            print(f"Error update_servo_position: servo widget {servo_widget.id}, check that the angles are being read and are not None")
            return
            
        new_servo_angle = current + sign * getattr(self, 'step', 1)
        
        # 1. Clamp the angle based on servo type
        if servo_widget.id in STD_SERVO_IDS:
            new_servo_angle = max(180-STD_SERVO_ANGLE_LIMIT, min(STD_SERVO_ANGLE_LIMIT, new_servo_angle))            
            
        else:
            new_servo_angle = max(-HS_SERVO_ANGLE_LIMIT, min(HS_SERVO_ANGLE_LIMIT, new_servo_angle))

        action_time = getattr(self, 'action_time', 500)
        
        # 2. Check if this servo is part of a motion pair
        matched_pair = None
        for pair in STD_SERVO_MOTION_PAIRS:
            if servo_widget.id in pair.get_ids():
                matched_pair = pair
                break  # Stop looking once we find the matching pair
                
        # 3. Execute the movement (Mirrored vs Single)
        if matched_pair:
            # Calculate the mirrored angles using your dataclass logic
            is_safe = matched_pair.set_angle(servo_widget.id, new_servo_angle)
            
            if is_safe:
                # Move Left Servo
                self.ros_node.move_one_servo(
                    matched_pair.servo_left_id,
                    matched_pair.servo_left_angle,
                    action_time
                )
                # Move Right Servo
                self.ros_node.move_one_servo(
                    matched_pair.servo_right_id,
                    matched_pair.servo_right_angle,
                    action_time
                )
                self.servo_control_subWidgets_dict[matched_pair.servo_left_id].set_angle(matched_pair.servo_left_angle)
                self.servo_control_subWidgets_dict[matched_pair.servo_right_id].set_angle(matched_pair.servo_right_angle)
            else:
                # Math hit the min/max angle limits defined in Servo_Pair
                print(f"Safety limits exceeded for pair: {matched_pair.get_ids()}")
                
        else:
            # Not in a pair, move normally
            servo_widget.set_angle(new_servo_angle)
            self.ros_node.move_one_servo(
                servo_widget.id,
                new_servo_angle,
                action_time
            )
        print(f"move_one_servo {'HS_Servo' if servo_widget.id not in STD_SERVO_IDS else 'STD_Servo'} id={servo_widget.id} angle={new_servo_angle}")
        return


if __name__ == "__main__":
    rclpy.init()
    app = QApplication(sys.argv)
    servogui_window = servoGUI()
    servogui_window.show()
    app.exec()

    # Clean up
    try:
        servogui_window.ros_node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(0)