"""
main_ros.py  –  N.U.B.I. Dynamics Controller
=============================================
Subscribes to:
  - legs_feedback      (Float32MultiArray)  –  12-DOF leg joints in degrees
  - upperbody_feedback (Float32MultiArray)  –  7-DOF upper-body joints in degrees

Publishes to:
  - status_command     (Int16MultiArray)    –  [COLLISION_FLAG, 1|0]

When "ROS Control" mode is active the received angles are forwarded directly
into SharedState.ctrl_overrides (radians), which the PhysicsEngine picks up
each step.  The LiveCollisionMonitor publishes live collision status to ROS at
the rate configured in the "Obstacle Detection Frequency" spin-box.
"""

import sys
import math

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSlider
from PySide6.QtCore import Qt, QTimer, Slot, QEvent
from PySide6.QtGui import QSurfaceFormat, QFont

from behaviors import NubiBehaviors
from classes import (
    Theme, SharedState,
    ROS_AVAILABLE, start_ros_thread,
    PhysicsEngine, MujocoRenderer,
    DiagnosticsPanel, BehaviorPanel, ActuatorPanel,
    LiveCollisionMonitor,
)

QApplication.setAttribute(Qt.ApplicationAttribute.AA_UseDesktopOpenGL)



# =============================================================================
# Main Window
# =============================================================================
class MainWindow(QMainWindow):
    def __init__(self, nubi_path: str):
        super().__init__()
        self.setWindowTitle("N.U.B.I. Dynamics Controller | ROS 2 + GUI")

        self.nubi         = NubiBehaviors(nubi_path)
        self.shared_state = SharedState(self.nubi.model, self.nubi.data)
        self.physics      = PhysicsEngine(self.nubi, self.shared_state)

        self._last_hovered_ui = ""
        self._last_locked_ui  = ""

        self._build_ui()

        # ROS – background spin thread (no-op when ROS unavailable)
        self.ros_node = start_ros_thread(self.shared_state)

        # Live collision publisher
        self.collision_monitor = LiveCollisionMonitor(
            self.shared_state, self.ros_node,
            frequency_hz=LiveCollisionMonitor.DEFAULT_HZ,
        )
        self.collision_monitor.start()

        # Timers
        self._physics_timer = QTimer()
        self._physics_timer.timeout.connect(self.physics.step)
        self._physics_timer.start(5)

        self._render_timer = QTimer()
        self._render_timer.timeout.connect(self.mj_viewer.update)
        self._render_timer.start(16)

        self._slider_sync_timer = QTimer()
        self._slider_sync_timer.timeout.connect(self._sync_sliders_from_ros)
        self._slider_sync_timer.start(33)

        # Signal wiring
        self.physics.collision_status.connect(self.diagnostics.col_indicator.set_collision)
        self.diagnostics.frequency_changed.connect(self.collision_monitor.set_frequency)
        self.behavior_panel.mode_changed.connect(self._change_mode)
        self.behavior_panel.ros_toggled.connect(self._toggle_ros_mode)
        self.actuator_panel.slider_moved.connect(self._on_slider_moved)
        self.actuator_panel.highlight_requested.connect(self.mj_viewer.set_highlight)
        self.actuator_panel.highlight_cleared.connect(lambda: self.mj_viewer.set_highlight(""))
        self.mj_viewer.hover_signal.connect(self._highlight_slider_row)
        self.mj_viewer.click_signal.connect(self._lock_slider_row)

        # Default to ROS mode
        if ROS_AVAILABLE:
            self.behavior_panel.activate_ros()
            with self.shared_state.lock:
                self.shared_state.mode = "ros"

    # =========================================================================
    # UI construction
    # =========================================================================
    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)

        root = QHBoxLayout(central)
        root.setContentsMargins(15, 15, 15, 15)
        root.setSpacing(15)

        # Left column: behaviors + diagnostics
        self.behavior_panel = BehaviorPanel(self.nubi.menu, ROS_AVAILABLE)
        self.diagnostics    = DiagnosticsPanel(
            default_hz=LiveCollisionMonitor.DEFAULT_HZ,
            min_hz=LiveCollisionMonitor.MIN_HZ,
            max_hz=LiveCollisionMonitor.MAX_HZ,
        )
        left = QVBoxLayout()
        left.addWidget(self.behavior_panel, 2)
        left.addWidget(self.diagnostics,    1)
        root.addLayout(left, 1)

        # Centre: MuJoCo viewport
        self.mj_viewer = MujocoRenderer(self.nubi.model, self.shared_state)
        container = QWidget.createWindowContainer(self.mj_viewer)
        container.setMinimumSize(500, 500)
        container.setStyleSheet(
            f"border: 2px solid {Theme.CYAN}; border-radius: 8px;"
            f" background-color: {Theme.BG};"
        )
        container.setMouseTracking(True)
        container.installEventFilter(self)
        self._viewport_container = container
        root.addWidget(container, 4)

        # Right column: actuator sliders
        self.actuator_panel = ActuatorPanel(list(self.nubi.actuators))
        # Install wheel-event filter on every slider
        for slider, _ in self.actuator_panel.sliders().values():
            slider.installEventFilter(self)
        root.addWidget(self.actuator_panel, 2)

    # =========================================================================
    # Mode management
    # =========================================================================
    def _change_mode(self, mode: str) -> None:
        self.behavior_panel.deactivate_ros()
        with self.shared_state.lock:
            self.shared_state.mode = mode
            self.shared_state.ctrl_overrides.clear()
        self.actuator_panel.reset()

    def _toggle_ros_mode(self, checked: bool) -> None:
        if checked:
            self.behavior_panel.set_ros_active(True)
            with self.shared_state.lock:
                self.shared_state.mode = "ros"
                self.shared_state.ctrl_overrides.clear()
            self.actuator_panel.reset()
        else:
            self.behavior_panel.set_ros_active(False)
            with self.shared_state.lock:
                self.shared_state.mode = ""
                self.shared_state.ctrl_overrides.clear()

    # =========================================================================
    # Slider helpers
    # =========================================================================
    def _on_slider_moved(self, name: str, val: int) -> None:
        self.behavior_panel.deactivate_ros()
        with self.shared_state.lock:
            self.shared_state.mode = "custom"
            self.shared_state.ctrl_overrides[name] = math.radians(val)

    def _sync_sliders_from_ros(self) -> None:
        with self.shared_state.lock:
            if self.shared_state.mode != "ros":
                return
            overrides = dict(self.shared_state.ctrl_overrides)
        self.actuator_panel.sync_from_overrides(overrides)

    # =========================================================================
    # Hover / Lock (3D viewport ↔ slider panel sync)
    # =========================================================================
    @Slot(str)
    def _highlight_slider_row(self, act_name: str) -> None:
        if self.mj_viewer.locked_actuator or self._last_hovered_ui == act_name:
            return
        self.actuator_panel.set_row_style(self._last_hovered_ui, "")
        self.actuator_panel.set_row_style(
            act_name,
            f"HoverRow {{ background-color: {Theme.CYAN_HOVER}; "
            f"border-radius: 4px; border: 1px solid {Theme.CYAN}; }}"
            if act_name else "",
        )
        self._last_hovered_ui = act_name

    @Slot(str)
    def _lock_slider_row(self, act_name: str) -> None:
        if self._last_locked_ui == act_name:
            return
        self.actuator_panel.set_row_style(self._last_locked_ui, "")
        self.actuator_panel.set_row_style(
            act_name,
            f"HoverRow {{ background-color: {Theme.CYAN_LOCKED}; "
            f"border: 1px solid {Theme.CYAN}; border-radius: 6px; }}"
            if act_name else "",
        )
        self.actuator_panel.ensure_row_visible(act_name)
        self._last_locked_ui = act_name

    # =========================================================================
    # Event filter
    # =========================================================================
    def eventFilter(self, obj, event) -> bool:
        if isinstance(obj, QSlider) and event.type() == QEvent.Type.Wheel:
            delta = event.angleDelta().y()
            if delta:
                obj.setValue(obj.value() + (1 if delta > 0 else -1))
            return True
        if (obj is self._viewport_container and
                event.type() == QEvent.Type.MouseMove and
                not event.buttons()):
            self.mj_viewer.process_hover(event.position())
        return super().eventFilter(obj, event)

    def keyPressEvent(self, event) -> None:
        if event.key() == Qt.Key_Escape and self.mj_viewer.locked_actuator:
            self.mj_viewer.locked_actuator = ""
            self.mj_viewer.click_signal.emit("")
        super().keyPressEvent(event)


# =============================================================================
# Entry point
# =============================================================================
if __name__ == "__main__":
    fmt = QSurfaceFormat()
    fmt.setDepthBufferSize(24)
    fmt.setStencilBufferSize(8)
    fmt.setSamples(8)
    fmt.setProfile(QSurfaceFormat.CompatibilityProfile)
    QSurfaceFormat.setDefaultFormat(fmt)

    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 10))
    app.setStyleSheet(Theme.app_stylesheet())

    window = MainWindow("nubi.xml")
    window.resize(1300, 850)
    window.show()
    sys.exit(app.exec())


# ──────────────────────────────────────────────────────────────────────────────
