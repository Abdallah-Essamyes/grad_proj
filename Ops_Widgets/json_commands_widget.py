import json
from pathlib import Path

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QCheckBox,
    QLabel, QInputDialog, QMessageBox, QSizePolicy
)
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QFont

COMMANDS_JSON_PATH = Path(__file__).parent.parent / "ops_jsons" / "commands.json"
OPS_GUI_JSON_PATH  = Path(__file__).parent.parent / "ops_jsons" / "ops_gui.json"

_MAX_ROWS = 5   # rows per column before wrapping to a new column


class JsonCommandsWidget(QWidget):
    """
    Commands panel.  Layout:
        [Title]
        [+ Add]  [▶ Start All]
        [col0: rows 0-4]  [col1: rows 5-9]  …  <stretch>

    Each row: [Run button]  [☑ include]  [✕ delete]
    Columns are real QWidget children — no layout recycling, no ghost columns.
    """

    launch_commands = pyqtSignal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._commands: dict[str, str] = {}

        self._load_json()
        self._load_gui_state()
        self._init_ui()

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def _load_json(self):
        COMMANDS_JSON_PATH.parent.mkdir(parents=True, exist_ok=True)
        if COMMANDS_JSON_PATH.exists():
            try:
                with open(COMMANDS_JSON_PATH) as f:
                    self._commands = json.load(f)
            except Exception:
                self._commands = {}
        else:
            self._commands = {}
            self._save_json()

    def _save_json(self):
        with open(COMMANDS_JSON_PATH, "w") as f:
            json.dump(self._commands, f, indent=4)

    def _load_gui_state(self):
        self._gui_state: dict = {"checkboxes": {}}
        if OPS_GUI_JSON_PATH.exists():
            try:
                with open(OPS_GUI_JSON_PATH) as f:
                    self._gui_state = json.load(f)
            except Exception:
                pass
        self._gui_state.setdefault("checkboxes", {})

    def _save_gui_state(self):
        OPS_GUI_JSON_PATH.parent.mkdir(parents=True, exist_ok=True)
        with open(OPS_GUI_JSON_PATH, "w") as f:
            json.dump(self._gui_state, f, indent=4)

    # ------------------------------------------------------------------
    # UI init  (called once)
    # ------------------------------------------------------------------

    def _init_ui(self):
        self._outer = QVBoxLayout(self)
        self._outer.setContentsMargins(4, 4, 4, 4)
        self._outer.setSpacing(4)

        # Title
        title = QLabel("Commands")
        title.setFont(QFont("Segoe UI", 11, QFont.Weight.Bold))
        self._outer.addWidget(title)

        # Header buttons
        header = QHBoxLayout()
        self._add_btn = QPushButton("+  Add")
        self._add_btn.setFixedHeight(30)
        self._add_btn.clicked.connect(self._on_add)
        self._start_all_btn = QPushButton("▶  Start All")
        self._start_all_btn.setFixedHeight(30)
        self._start_all_btn.clicked.connect(self._on_start_all)
        header.addWidget(self._add_btn)
        header.addWidget(self._start_all_btn)
        header.addStretch()
        self._outer.addLayout(header)

        # Placeholder for the columns row — rebuilt by _rebuild_ui()
        self._cols_row = QHBoxLayout()
        self._cols_row.setContentsMargins(0, 0, 0, 0)
        self._cols_row.setSpacing(12)
        self._outer.addLayout(self._cols_row)
        self._outer.addStretch()
       # self.setStyleSheet("""
       #     background-color: #f0f0f0;
       #     color: #333;
       #     """)
        self._rebuild_ui()

    # ------------------------------------------------------------------
    # Rebuild  (destroy all column widgets, recreate from scratch)
    # ------------------------------------------------------------------

    def _rebuild_ui(self):
        # Remove and delete every item currently in _cols_row
        while self._cols_row.count():
            item = self._cols_row.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()

        names = list(self._commands.keys())
        if not names:
            self.updateGeometry()
            return

        col_widget: QWidget | None = None
        col_layout: QVBoxLayout | None = None

        for idx, name in enumerate(names):
            # Start a new column every _MAX_ROWS entries
            if idx % _MAX_ROWS == 0:
                col_widget = QWidget()
                col_widget.setSizePolicy(
                    QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
                )
                col_layout = QVBoxLayout(col_widget)
                col_layout.setContentsMargins(0, 0, 0, 0)
                col_layout.setSpacing(3)
                self._cols_row.addWidget(col_widget,stretch=1)

            # Row widget
            row = QWidget()
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.setSpacing(4)

            run_btn = QPushButton(name)
            run_btn.setFixedHeight(28)
            run_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            run_btn.setToolTip(self._commands.get(name, ""))
            run_btn.clicked.connect(lambda _c, n=name: self._on_run_single(n))

            cb = QCheckBox()
            cb.setToolTip("Include in Start All")
            cb.setChecked(bool(self._gui_state["checkboxes"].get(name, True)))
            cb.toggled.connect(lambda checked, n=name: self._on_checkbox_toggled(n, checked))

            del_btn = QPushButton("✕")
            del_btn.setFixedSize(24, 28)
            del_btn.setToolTip("Delete")
            del_btn.clicked.connect(lambda _c, n=name: self._on_delete(n))

            row_layout.addWidget(run_btn)
            row_layout.addWidget(cb)
            row_layout.addWidget(del_btn)

            col_layout.addWidget(row)

        # Trailing stretch so the last (partial) column doesn't stretch vertically
        if col_layout is not None:
            col_layout.addStretch()

     

        self.updateGeometry()

    # ------------------------------------------------------------------
    # Handlers
    # ------------------------------------------------------------------

    def _on_add(self):
        from PyQt6.QtWidgets import QDialog, QDialogButtonBox, QLineEdit, QFormLayout

        dialog = QDialog(self)
        dialog.setWindowTitle("Add Command")
        dialog.setMinimumWidth(500)

        name_edit = QLineEdit()
        cmd_edit  = QLineEdit()

        form = QFormLayout(dialog)
        form.addRow("Name:",    name_edit)
        form.addRow("Command:", cmd_edit)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)

        if dialog.exec() != QDialog.DialogCode.Accepted:
            return

        name = name_edit.text().strip()
        cmd  = cmd_edit.text().strip()

        if not name or not cmd:
            QMessageBox.warning(self, "Empty field", "Both name and command are required.")
            return
        if name in self._commands:
            QMessageBox.warning(self, "Duplicate", f"'{name}' already exists.")
            return

        self._commands[name] = cmd
        self._save_json()
        self._rebuild_ui()

    def _on_delete(self, name: str):
        reply = QMessageBox.question(
            self, "Delete", f"Delete '{name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self._commands.pop(name, None)
            self._save_json()
            self._gui_state["checkboxes"].pop(name, None)
            self._save_gui_state()
            self._rebuild_ui()

    def _on_run_single(self, name: str):
        cmd = self._commands.get(name)
        if cmd:
            self.launch_commands.emit([(name, cmd)])

    def _on_checkbox_toggled(self, name: str, checked: bool):
        self._gui_state["checkboxes"][name] = checked
        self._save_gui_state()

    def _on_start_all(self):
        checked = [
            (n, self._commands[n])
            for n in self._commands
            if self._gui_state["checkboxes"].get(n, True)
        ]
        if not checked:
            checked = list(self._commands.items())
        self.launch_commands.emit(checked)

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

    def checked_commands(self) -> list[tuple[str, str]]:
        return [
            (n, self._commands[n])
            for n in self._commands
            if self._gui_state["checkboxes"].get(n, True)
        ]


if __name__ == "__main__":
    import sys
    from PyQt6.QtWidgets import QApplication
    app = QApplication(sys.argv)
    w = JsonCommandsWidget()
    w.setWindowTitle("Commands")
    w.show()
    sys.exit(app.exec())
