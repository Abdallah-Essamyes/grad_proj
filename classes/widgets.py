"""Reusable UI widgets.

Classes
-------
HoverRow          – QFrame that emits hover signals
CollisionIndicator – circular LED widget
DiagnosticsPanel  – collision LED + frequency spinbox
BehaviorPanel     – behavior buttons + ROS-control toggle
ActuatorPanel     – scrollable slider list for every actuator
"""

import math

from PySide6.QtWidgets import (
    QFrame, QWidget, QGroupBox,
    QVBoxLayout, QHBoxLayout,
    QPushButton, QSlider, QLabel,
    QScrollArea, QDoubleSpinBox,
)
from PySide6.QtCore import Qt, Signal, Slot
from PySide6.QtGui import QPainter, QColor, QFont, QBrush, QRadialGradient, QPen

from .theme import Theme


class HoverRow(QFrame):
    """A QFrame that emits signals when the mouse enters or leaves."""

    hoverEntered = Signal(str)
    hoverLeft    = Signal(str)

    def __init__(self, name: str, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = name

    def enterEvent(self, event) -> None:
        self.hoverEntered.emit(self.name)
        super().enterEvent(event)

    def leaveEvent(self, event) -> None:
        self.hoverLeft.emit(self.name)
        super().leaveEvent(event)


class CollisionIndicator(QWidget):
    """Circular LED-style indicator that shows SAFE / COLLISION state."""

    def __init__(self):
        super().__init__()
        self.setFixedSize(140, 140)
        self.is_colliding = False

    @Slot(bool)
    def set_collision(self, state: bool) -> None:
        if self.is_colliding != state:
            self.is_colliding = state
            self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(15, 15, -15, -15)
        gradient = QRadialGradient(rect.center(), rect.width() / 2)

        if self.is_colliding:
            gradient.setColorAt(0.0, QColor(Theme.WARN_CORE))
            gradient.setColorAt(0.8, QColor(Theme.WARN_MID))
            gradient.setColorAt(1.0, QColor(Theme.WARN_EDGE))
            border_color = QColor(Theme.WARN_CORE)
            text = "COLLISION"
        else:
            gradient.setColorAt(0.0, QColor(Theme.SAFE_CORE))
            gradient.setColorAt(0.8, QColor(Theme.SAFE_MID))
            gradient.setColorAt(1.0, QColor(Theme.SAFE_EDGE))
            border_color = QColor(Theme.SAFE_CORE)
            text = "SAFE"

        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(border_color, 2))
        painter.drawEllipse(rect)

        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor(Theme.PANEL_BORDER), 4))
        painter.drawEllipse(self.rect().adjusted(5, 5, -5, -5))

        painter.setPen(QColor(Theme.TEXT))
        painter.setFont(QFont("Segoe UI", 11, QFont.Bold))
        painter.drawText(self.rect(), Qt.AlignCenter, text)


# =============================================================================
# DiagnosticsPanel
# =============================================================================
class DiagnosticsPanel(QGroupBox):
    """System Diagnostics group: collision LED + collision-publish Hz spinbox.

    Signals
    -------
    frequency_changed(float)  – emitted when the spinbox value changes
    """

    frequency_changed = Signal(float)

    def __init__(self, default_hz: float, min_hz: float, max_hz: float):
        super().__init__("System Diagnostics")
        lay = QVBoxLayout(self)
        lay.setAlignment(Qt.AlignCenter)

        self.col_indicator = CollisionIndicator()
        lay.addWidget(self.col_indicator)

        freq_row = QHBoxLayout()
        freq_lbl = QLabel("Collision Hz")
        freq_lbl.setStyleSheet(f"color: {Theme.MUTED}; font-size: 11px;")

        self._freq_spin = QDoubleSpinBox()
        self._freq_spin.setRange(min_hz, max_hz)
        self._freq_spin.setDecimals(1)
        self._freq_spin.setSingleStep(1.0)
        self._freq_spin.setValue(default_hz)
        self._freq_spin.setSuffix(" Hz")
        self._freq_spin.setToolTip(
            "Rate at which live collision status is published on status_command"
        )
        self._freq_spin.valueChanged.connect(self.frequency_changed)

        freq_row.addWidget(freq_lbl)
        freq_row.addWidget(self._freq_spin)
        lay.addLayout(freq_row)


# =============================================================================
# BehaviorPanel
# =============================================================================
class BehaviorPanel(QGroupBox):
    """Kinematic Behaviors group: one button per menu entry + ROS Control toggle.

    Signals
    -------
    mode_changed(str)   – emitted when a behavior button is clicked (carries the mode key)
    ros_toggled(bool)   – emitted when the ROS Control button is toggled
    """

    mode_changed = Signal(str)
    ros_toggled  = Signal(bool)

    def __init__(self, menu: dict, ros_available: bool):
        """Parameters
        ----------
        menu          : {mode_key: callable}  from NubiBehaviors.menu
        ros_available : bool
        """
        super().__init__("Kinematic Behaviors")
        lay = QVBoxLayout(self)
        lay.setSpacing(10)

        for cmd, func in menu.items():
            label = func.__name__.split("_")[-1].title()
            btn   = QPushButton(label)
            btn.clicked.connect(lambda _, c=cmd: self.mode_changed.emit(c))
            lay.addWidget(btn)

        self._ros_btn = QPushButton("ROS Control")
        self._ros_btn.setCheckable(True)
        self._ros_btn.setEnabled(ros_available)
        if not ros_available:
            self._ros_btn.setToolTip("rclpy not found – ROS Control unavailable")
        self._ros_btn.toggled.connect(self.ros_toggled)
        lay.addWidget(self._ros_btn)
        lay.addStretch()

    # ------------------------------------------------------------------
    def set_ros_active(self, active: bool) -> None:
        """Apply/remove the active-ROS visual style without emitting toggled."""
        self._ros_btn.setStyleSheet(
            f"background-color: {Theme.ROS_ACTIVE_BG}; "
            f"border: 1px solid {Theme.ROS_ACTIVE}; "
            f"color: {Theme.ROS_ACTIVE}; "
            "padding: 10px; border-radius: 6px; font-weight: bold;"
            if active else ""
        )

    def deactivate_ros(self) -> None:
        """Uncheck and un-style the ROS button silently."""
        if self._ros_btn.isChecked():
            self._ros_btn.blockSignals(True)
            self._ros_btn.setChecked(False)
            self._ros_btn.blockSignals(False)
        self.set_ros_active(False)

    def activate_ros(self) -> None:
        """Check and style the ROS button silently."""
        self._ros_btn.blockSignals(True)
        self._ros_btn.setChecked(True)
        self._ros_btn.blockSignals(False)
        self.set_ros_active(True)


# =============================================================================
# ActuatorPanel
# =============================================================================
class ActuatorPanel(QGroupBox):
    """Actuator Array group: a scrollable list of labelled sliders.

    Signals
    -------
    slider_moved(str, int)      – actuator name + degree value when user drags a slider
    highlight_requested(str)    – forwarded hover-enter from HoverRow (actuator name)
    highlight_cleared()         – forwarded hover-leave
    """

    slider_moved        = Signal(str, int)
    highlight_requested = Signal(str)
    highlight_cleared   = Signal()

    def __init__(self, actuator_names: list[str]):
        super().__init__("Actuator Array")
        lay = QVBoxLayout(self)

        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)

        inner = QWidget()
        c_lay = QVBoxLayout(inner)
        c_lay.setSpacing(6)

        self._rows:   dict[str, HoverRow]              = {}
        self._sliders: dict[str, tuple[QSlider, QLabel]] = {}

        for name in actuator_names:
            row = HoverRow(name)
            row_lay = QHBoxLayout(row)
            row_lay.setContentsMargins(8, 4, 8, 4)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(-150, 150)
            value_lbl = QLabel("0°")
            value_lbl.setFixedWidth(45)
            value_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            slider.valueChanged.connect(
                lambda v, n=name, l=value_lbl: self._on_slider_moved(n, v, l)
            )

            row.hoverEntered.connect(self.highlight_requested)
            row.hoverLeft.connect(lambda _: self.highlight_cleared.emit())

            self._rows[name]    = row
            self._sliders[name] = (slider, value_lbl)

            name_lbl = QLabel(name[:-2])    # strip "_p" suffix
            name_lbl.setFixedWidth(100)
            name_lbl.setStyleSheet(f"color: {Theme.MUTED}; font-weight: bold;")

            row_lay.addWidget(name_lbl)
            row_lay.addWidget(slider)
            row_lay.addWidget(value_lbl)
            c_lay.addWidget(row)

        self._scroll.setWidget(inner)
        lay.addWidget(self._scroll)

    # ------------------------------------------------------------------
    # Public API used by MainWindow
    # ------------------------------------------------------------------
    def sliders(self) -> dict[str, tuple[QSlider, QLabel]]:
        return self._sliders

    def rows(self) -> dict[str, HoverRow]:
        return self._rows

    def scroll_area(self) -> QScrollArea:
        return self._scroll

    def reset(self) -> None:
        """Set all sliders to 0 without emitting slider_moved."""
        for slider, lbl in self._sliders.values():
            slider.blockSignals(True)
            slider.setValue(0)
            slider.blockSignals(False)
            lbl.setText("0°")

    def sync_from_overrides(self, overrides: dict[str, float]) -> None:
        """Update slider positions from a {name: radians} dict (no signals emitted)."""
        for act_name, rad_val in overrides.items():
            if act_name not in self._sliders:
                continue
            slider, lbl = self._sliders[act_name]
            deg = max(slider.minimum(),
                      min(slider.maximum(), int(round(math.degrees(rad_val)))))
            if slider.value() != deg:
                slider.blockSignals(True)
                slider.setValue(deg)
                slider.blockSignals(False)
                lbl.setText(f"{deg}°")

    def set_row_style(self, name: str, style: str) -> None:
        if name and name in self._rows:
            self._rows[name].setStyleSheet(
                style or "HoverRow { background-color: transparent; }"
            )

    def ensure_row_visible(self, name: str) -> None:
        if name and name in self._rows:
            self._scroll.ensureWidgetVisible(self._rows[name], xmargin=50, ymargin=50)

    # ------------------------------------------------------------------
    def _on_slider_moved(self, name: str, val: int, lbl: QLabel) -> None:
        lbl.setText(f"{val}°")
        self.slider_moved.emit(name, val)
