import os
from PyQt6.QtWidgets import (QWidget, QLabel, QCheckBox, QDoubleSpinBox,
                              QPushButton, QSlider, QSpinBox)
from PyQt6.QtCore import Qt, QSettings, QTimer, pyqtSignal


class SidebarPanel(QWidget):
    # ── Signals emitted outward (connect these in the main window) ────────────
    ask_status_requested   = pyqtSignal()
    ask_torque_requested   = pyqtSignal()
    reinitialize_requested = pyqtSignal()
    clear_errors_requested = pyqtSignal()
    status_poll_tick       = pyqtSignal()   # fired by internal timer
    torque_poll_tick       = pyqtSignal()   # fired by internal timer
    step_changed           = pyqtSignal(int)
    action_time_changed    = pyqtSignal(int)

    def __init__(self, panel_width: int, parent=None):
        super().__init__(parent)
        self._panel_width = panel_width

        # Poll timers live here now, not in the main window
        self._status_poll_timer = QTimer(self)
        self._status_poll_timer.timeout.connect(self.status_poll_tick)
        self._torque_poll_timer = QTimer(self)
        self._torque_poll_timer.timeout.connect(self.torque_poll_tick)

        self._build_ui()
        self._wire_internal_signals()
        self._load_settings()

    # ─────────────────────────────────────────────────────────────────────────
    # UI Construction
    # ─────────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        lx = 6                            # left padding inside the panel
        w  = self._panel_width - lx - 4  # usable width for full-width buttons

        # ── Shared stylesheets ──────────────────────────────────────────────
        check_svg_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "documents", "white_check.svg"
        )
        if not os.path.exists(check_svg_path):
            with open(check_svg_path, "w") as f:
                f.write(
                    '<svg xmlns="http://www.w3.org/2000/svg" width="12" height="12" viewBox="0 0 12 12">'
                    '<polyline points="1,7 4,11 11,1" stroke="white" fill="none" stroke-width="2.2"/>'
                    '</svg>'
                )
        svg_escaped = check_svg_path.replace("\\", "/")

        cb_ss = (
            "QCheckBox {{ color: black; background: transparent; spacing: 5px; }}\n"
            "QCheckBox::indicator {{ width: 15px; height: 15px; background: black; "
            "border: 1px solid #888; border-radius: 2px; }}\n"
            "QCheckBox::indicator:checked {{ background: black; border: 1px solid #aaa; "
            "image: url({svg}); }}\n"
        ).format(svg=svg_escaped)

        label_ss  = "color: black; background: transparent;"
        blue_btn  = ("QPushButton { background: #2980b9; color: white; font-weight: bold; "
                     "font-size: 12px; border-radius: 4px; padding: 3px; }\n"
                     "QPushButton:pressed { background: #1a5276; }")
        red_btn   = ("QPushButton { background: #c0392b; color: white; font-weight: bold; "
                     "font-size: 13px; border-radius: 4px; padding: 4px; }\n"
                     "QPushButton:pressed { background: #96281b; }")

        # ── Status section ──────────────────────────────────── local y = 0
        y = 0
        self.status_auto_cb = QCheckBox("Auto", parent=self)
        self.status_auto_cb.setStyleSheet(cb_ss)
        self.status_auto_cb.move(lx, y)
        self.status_auto_cb.adjustSize()

        self.status_freq_label = QLabel("Status Frequency", parent=self)
        self.status_freq_label.setStyleSheet(label_ss + " font-weight: bold;")
        self.status_freq_label.move(lx + self.status_auto_cb.width() + 2, y)
        self.status_freq_label.adjustSize()

        self.status_freq_spin = QDoubleSpinBox(parent=self)
        self.status_freq_spin.setRange(0.1, 20.0)
        self.status_freq_spin.setSingleStep(0.5)
        self.status_freq_spin.setValue(1.0)
        self.status_freq_spin.setDecimals(1)
        self.status_freq_spin.resize(65, 24)
        self.status_freq_spin.move(lx, y + 22)

        self.status_hz_label = QLabel("Hz", parent=self)
        self.status_hz_label.setStyleSheet(label_ss)
        self.status_hz_label.move(lx + 68, y + 26)
        self.status_hz_label.adjustSize()

        self.ask_status_btn = QPushButton("Ask for Status", parent=self)
        self.ask_status_btn.setStyleSheet(blue_btn)
        self.ask_status_btn.resize(w, 26)
        self.ask_status_btn.move(lx, y + 50)

        # ── Torque section ──────────────────────────────────── local y = 88
        y = 88
        self.torque_auto_cb = QCheckBox("Auto", parent=self)
        self.torque_auto_cb.setStyleSheet(cb_ss)
        self.torque_auto_cb.move(lx, y)
        self.torque_auto_cb.adjustSize()

        self.torque_freq_label = QLabel("Torque Frequency", parent=self)
        self.torque_freq_label.setStyleSheet(label_ss + " font-weight: bold;")
        self.torque_freq_label.move(lx + self.torque_auto_cb.width() + 2, y)
        self.torque_freq_label.adjustSize()

        self.torque_freq_spin = QDoubleSpinBox(parent=self)
        self.torque_freq_spin.setRange(0.1, 20.0)
        self.torque_freq_spin.setSingleStep(0.5)
        self.torque_freq_spin.setValue(1.0)
        self.torque_freq_spin.setDecimals(1)
        self.torque_freq_spin.resize(65, 24)
        self.torque_freq_spin.move(lx, y + 22)

        self.torque_hz_label = QLabel("Hz", parent=self)
        self.torque_hz_label.setStyleSheet(label_ss)
        self.torque_hz_label.move(lx + 68, y + 26)
        self.torque_hz_label.adjustSize()

        self.ask_torque_btn = QPushButton("Ask for Torque", parent=self)
        self.ask_torque_btn.setStyleSheet(blue_btn)
        self.ask_torque_btn.resize(w, 26)
        self.ask_torque_btn.move(lx, y + 50)

        # ── Step control ────────────────────────────────────── local y = 204
        y = 204
        self.step_warning = QLabel("Step Size, Max angle is 90", parent=self)
        self.step_warning.setStyleSheet("color: red; font-weight: bold;")
        self.step_warning.move(lx, y - 14)
        self.step_warning.hide()

        self.step_label = QLabel("Step: 10", parent=self)
        self.step_label.setStyleSheet(label_ss)
        self.step_label.move(lx, y)
        self.step_label.adjustSize()

        self.step_slider = QSlider(Qt.Orientation.Horizontal, parent=self)
        self.step_slider.setRange(0, 90)
        self.step_slider.setValue(10)
        self.step_slider.resize(w - 42, 20)
        self.step_slider.move(lx, y + 20)

        self.step_spinBox = QSpinBox(parent=self)
        self.step_spinBox.setRange(0, 90)
        self.step_spinBox.setValue(10)
        self.step_spinBox.resize(40, 22)
        self.step_spinBox.move(lx + w - 40, y + 16)

        # ── Action time ─────────────────────────────────────── local y = 252
        y = 252
        self.action_time_label = QLabel("Action Time", parent=self)
        self.action_time_label.setStyleSheet(
            "color: black; background-color: white; font-weight: bold;")
        self.action_time_label.move(lx, y)
        self.action_time_label.adjustSize()

        self.action_time_spinBox = QSpinBox(parent=self)
        self.action_time_spinBox.setRange(200, 2856)
        self.action_time_spinBox.setValue(1000)
        self.action_time_spinBox.resize(80, 22)
        self.action_time_spinBox.move(lx + w - 80, y - 4)

        # ── Buttons ─────────────────────────────────────────── local y = 280
        y = 280
        self.error_clear_btn = QPushButton("Clear Errors", parent=self)
        self.error_clear_btn.setStyleSheet(red_btn)
        self.error_clear_btn.resize(w, 30)
        self.error_clear_btn.move(lx, y)

        self.reinit_btn = QPushButton("Reinitialize", parent=self)
        self.reinit_btn.setStyleSheet(blue_btn.replace("font-size: 12px", "font-size: 13px"))
        self.reinit_btn.resize(w, 30)
        self.reinit_btn.move(lx, y + 36)

    # ─────────────────────────────────────────────────────────────────────────
    # Internal signal wiring
    # ─────────────────────────────────────────────────────────────────────────

    def _wire_internal_signals(self):
        # Buttons → outward signals
        self.ask_status_btn.clicked.connect(self.ask_status_requested)
        self.ask_torque_btn.clicked.connect(self.ask_torque_requested)
        self.reinit_btn.clicked.connect(self.reinitialize_requested)
        self.error_clear_btn.clicked.connect(self.clear_errors_requested)

        # Auto checkboxes / spinboxes → internal timers
        self.status_auto_cb.toggled.connect(self._on_status_auto_toggled)
        self.status_freq_spin.valueChanged.connect(self._on_status_freq_changed)
        self.torque_auto_cb.toggled.connect(self._on_torque_auto_toggled)
        self.torque_freq_spin.valueChanged.connect(self._on_torque_freq_changed)

        # Persist any change to settings
        self.status_auto_cb.toggled.connect(self._save_settings)
        self.status_freq_spin.valueChanged.connect(self._save_settings)
        self.torque_auto_cb.toggled.connect(self._save_settings)
        self.torque_freq_spin.valueChanged.connect(self._save_settings)

        # Step slider ↔ spinbox (keep in sync, then emit upward)
        self.step_slider.valueChanged.connect(self.step_spinBox.setValue)
        self.step_spinBox.valueChanged.connect(self.step_slider.setValue)
        self.step_spinBox.valueChanged.connect(self._on_step_changed)
        self.step_spinBox.editingFinished.connect(self._on_step_editing_finished)

        # Action time
        self.action_time_spinBox.valueChanged.connect(self.action_time_changed)

    # ─────────────────────────────────────────────────────────────────────────
    # Internal handlers
    # ─────────────────────────────────────────────────────────────────────────

    def _on_status_auto_toggled(self, checked: bool):
        if checked:
            self._status_poll_timer.start(max(50, int(1000 / self.status_freq_spin.value())))
        else:
            self._status_poll_timer.stop()

    def _on_status_freq_changed(self, hz: float):
        if self.status_auto_cb.isChecked():
            self._status_poll_timer.start(max(50, int(1000 / hz)))

    def _on_torque_auto_toggled(self, checked: bool):
        if checked:
            self._torque_poll_timer.start(max(50, int(1000 / self.torque_freq_spin.value())))
        else:
            self._torque_poll_timer.stop()

    def _on_torque_freq_changed(self, hz: float):
        if self.torque_auto_cb.isChecked():
            self._torque_poll_timer.start(max(50, int(1000 / hz)))

    def _on_step_changed(self, val: int):
        clamped = min(val, 90)
        if clamped != val:
            self.step_spinBox.setValue(clamped)
        self.step_label.setText(f"Step: {clamped}")
        self.step_changed.emit(clamped)

    def _on_step_editing_finished(self):
        try:
            val = int(self.step_spinBox.text())
        except ValueError:
            return
        if val > 90:
            self.step_spinBox.setValue(90)

    # ─────────────────────────────────────────────────────────────────────────
    # Settings persistence
    # ─────────────────────────────────────────────────────────────────────────

    def _save_settings(self, *_):
        s = QSettings("NUBI", "ServoGUI")
        s.setValue("status_auto", int(self.status_auto_cb.isChecked()))
        s.setValue("status_hz",   self.status_freq_spin.value())
        s.setValue("torque_auto", int(self.torque_auto_cb.isChecked()))
        s.setValue("torque_hz",   self.torque_freq_spin.value())
        s.sync()

    def _load_settings(self):
        s = QSettings("NUBI", "ServoGUI")

        # Disconnect save_settings while restoring so we don't overwrite with defaults
        for sig in (self.status_auto_cb.toggled, self.status_freq_spin.valueChanged,
                    self.torque_auto_cb.toggled, self.torque_freq_spin.valueChanged):
            sig.disconnect(self._save_settings)

        self.status_freq_spin.setValue(float(s.value("status_hz", 1.0)))
        self.torque_freq_spin.setValue(float(s.value("torque_hz", 1.0)))

        def _to_bool(v):
            return str(v).strip().lower() in ("1", "true")

        self.status_auto_cb.setChecked(_to_bool(s.value("status_auto", 0)))
        self.torque_auto_cb.setChecked(_to_bool(s.value("torque_auto", 0)))

        for sig in (self.status_auto_cb.toggled, self.status_freq_spin.valueChanged,
                    self.torque_auto_cb.toggled, self.torque_freq_spin.valueChanged):
            sig.connect(self._save_settings)