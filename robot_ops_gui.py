from pathlib import Path
import sys

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QSplitter, QLabel
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QIcon
    
from Ops_Widgets.json_commands_widget import JsonCommandsWidget
from Ops_Widgets.terminal_grid_widget import TerminalGridWidget
from Ops_Widgets.actions_control_widget import ActionsControlWidget


class RobotOpsGUI(QMainWindow):
    """
    Main operations GUI.

    Layout (vertical splitter):
        ┌──────────────────────────────┐
        │  JsonCommandsWidget          │  ← commands panel (collapsible)
        ├──────────────────────────────┤
        │  TerminalGridWidget          │  ← terminal panes (expands)
        └──────────────────────────────┘
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("NUBI - Robot Operations")
        icon_path = Path(__file__).resolve().parent / "documents" / "ops_icon_white.png"
        self.setWindowIcon(QIcon(str(icon_path)))
        self.resize(1200, 800)

        self._commands_widget = JsonCommandsWidget()
        self._terminal_widget = TerminalGridWidget()
        self._actions_widget = ActionsControlWidget()

        # Connect: when commands are launched, pass them to the terminal grid
        self._commands_widget.launch_commands.connect(self._terminal_widget.launch)

        # Splitter gives user drag-to-resize between panels
        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.addWidget(self._commands_widget)
        splitter.addWidget(self._terminal_widget)
        splitter.setStretchFactor(0, 0)   # commands panel – sized to content
        splitter.setStretchFactor(1, 1)   # terminal area – takes remaining space
        # Let Qt use each widget's sizeHint for initial sizes
        splitter.setSizes([self._commands_widget.sizeHint().height(), 600])

        # Right-side panel: vertical splitter (commands + terminals)
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.addWidget(splitter)

        # Horizontal layout: actions panel on the left, rest on the right
        h_splitter = QSplitter(Qt.Orientation.Horizontal)
        self._actions_widget.setMinimumWidth(200)
        self._actions_widget.setMaximumWidth(320)
        h_splitter.addWidget(self._actions_widget)
        h_splitter.addWidget(right_panel)
        h_splitter.setStretchFactor(0, 0)
        h_splitter.setStretchFactor(1, 1)
        h_splitter.setSizes([240, 960])

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(0)
        layout.addWidget(h_splitter)
        self.setCentralWidget(container)

        self._apply_style()
        
    def closeEvent(self, event):
        """Stop all running terminal processes before closing the window."""
        self._terminal_widget.stop_all()
        event.accept()
        
    def _apply_style(self):
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #0d1117;
                color: #c9d1d9;
            }
            QPushButton {
                background-color: #21262d;
                color: #c9d1d9;
                border: 1px solid #30363d;
                border-radius: 4px;
                padding: 2px 8px;
            }
            QPushButton:hover {
                background-color: #30363d;
                border-color: #8b949e;
            }
            QPushButton:pressed {
                background-color: #161b22;
            }
            QCheckBox {
                color: #c9d1d9;
            }
            QScrollArea {
                border: none;
                background: transparent;
            }
            QScrollBar:vertical {
                background: #161b22;
                width: 8px;
                border-radius: 4px;
            }
            QScrollBar::handle:vertical {
                background: #30363d;
                border-radius: 4px;
                min-height: 20px;
            }
            QSplitter::handle {
                background: #30363d;
                height: 3px;
            }
            QLabel {
                color: #c9d1d9;
            }
        """)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotOpsGUI()
    window.show()
    sys.exit(app.exec())
