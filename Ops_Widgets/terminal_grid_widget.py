import signal

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTextEdit,
    QPushButton, QSizePolicy, QSplitter
)
from PyQt6.QtCore import Qt, QProcess, QProcessEnvironment, pyqtSignal
from PyQt6.QtGui import QFont, QTextCursor, QColor


# ─────────────────────────────────────────────────────────── #
#  Single terminal pane                                        #
# ─────────────────────────────────────────────────────────── #

class _TerminalPane(QWidget):
    """One process pane: title bar + scrolling text output."""

    close_requested = pyqtSignal(object)  # emits self

    def __init__(self, name: str, command: str, parent=None):
        super().__init__(parent)
        self.name = name
        self.command = command
        self._process: QProcess | None = None
        self._init_ui()
        self.start()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        # ── Title bar ──────────────────────────────────────────────────
        bar = QHBoxLayout()

        self._status_dot = QLabel("●")
        self._status_dot.setStyleSheet("color: #888;")
        self._status_dot.setFixedWidth(14)

        name_label = QLabel(self.name)
        name_label.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        name_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        self._stop_btn = QPushButton("■ Stop")
        self._stop_btn.setFixedHeight(22)
        self._stop_btn.setStyleSheet("font-size: 10px;")
        self._stop_btn.clicked.connect(self.stop)

        self._restart_btn = QPushButton("↺ Restart")
        self._restart_btn.setFixedHeight(22)
        self._restart_btn.setStyleSheet("font-size: 10px;")
        self._restart_btn.clicked.connect(self.restart)

        self._clear_btn = QPushButton("✕ Clear")
        self._clear_btn.setFixedHeight(22)
        self._clear_btn.setStyleSheet("font-size: 10px;")

        self._close_btn = QPushButton("✕ Close")
        self._close_btn.setFixedHeight(22)
        self._close_btn.setStyleSheet("font-size: 10px; color: #f85149;")
        self._close_btn.clicked.connect(lambda: self.close_requested.emit(self))

        bar.addWidget(self._status_dot)
        bar.addWidget(name_label)
        bar.addWidget(self._stop_btn)
        bar.addWidget(self._restart_btn)
        bar.addWidget(self._clear_btn)
        bar.addWidget(self._close_btn)
        layout.addLayout(bar)

        # ── Output text area ───────────────────────────────────────────
        self._output = QTextEdit()
        self._output.setReadOnly(True)
        self._output.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        self._output.setFont(QFont("Monospace", 8))
        self._output.setStyleSheet(
            "background: #0d1117; color: #c9d1d9; border: 1px solid #30363d;"
        )
        layout.addWidget(self._output)

        self._clear_btn.clicked.connect(self._output.clear)
        self.setStyleSheet("border: 1px solid #30363d; border-radius: 4px;")

    # ── Process management ──────────────────────────────────────────── #

    def start(self):
        if self._process and self._process.state() != QProcess.ProcessState.NotRunning:
            return
        self._process = QProcess(self)
        self._process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self._process.readyReadStandardOutput.connect(self._on_output)
        self._process.finished.connect(self._on_finished)
        self._process.started.connect(self._on_started)
        self._process.errorOccurred.connect(self._on_error)

        env = QProcessEnvironment.systemEnvironment()
        self._process.setProcessEnvironment(env)

        self._append_system(f"$ {self.command}\n")
        self._process.start(
            "/usr/bin/setsid",
            ["/bin/bash", "-c", self.command]
        )
        self._set_running(True)

    def stop(self):
        if self._process and self._process.state() != QProcess.ProcessState.NotRunning:
            self._append_system("\n[Sending interrupt…]\n")
            pid = self._process.processId()
            if pid:
                try:
                    import os
                    os.killpg(pid, signal.SIGINT)
                except (ProcessLookupError, PermissionError, OSError):
                    try:
                        import os
                        os.kill(pid, signal.SIGINT)
                    except (ProcessLookupError, PermissionError):
                        pass
            if not self._process.waitForFinished(5000):
                pid = self._process.processId()
                if pid:
                    try:
                        import os
                        os.killpg(pid, signal.SIGKILL)
                    except (ProcessLookupError, PermissionError, OSError):
                        pass
                self._process.kill()

    def restart(self):
        self.stop()
        self._output.clear()
        self.start()

    # ── Private callbacks ───────────────────────────────────────────── #

    def _on_output(self):
        raw = bytes(self._process.readAllStandardOutput()).decode("utf-8", errors="replace")
        self._append_stdout(raw)

    def _on_started(self):
        self._set_running(True)

    def _on_finished(self, exit_code, exit_status):
        self._append_system(f"\n[Process exited with code {exit_code}]\n")
        self._set_running(False)

    def _on_error(self, error):
        self._append_system(f"\n[Process error: {error}]\n")
        self._set_running(False)

    def _append_stdout(self, text: str):
        cursor = self._output.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        self._output.setTextCursor(cursor)
        self._output.insertPlainText(text)
        sb = self._output.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _append_system(self, text: str):
        cursor = self._output.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        fmt = cursor.charFormat()
        fmt.setForeground(QColor("#8b949e"))
        cursor.setCharFormat(fmt)
        cursor.insertText(text)
        fmt.setForeground(QColor("#c9d1d9"))
        cursor.setCharFormat(fmt)
        self._output.setTextCursor(cursor)
        sb = self._output.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _set_running(self, running: bool):
        if running:
            self._status_dot.setStyleSheet("color: #3fb950;")
        else:
            self._status_dot.setStyleSheet("color: #f85149;")


# ─────────────────────────────────────────────────────────── #
#  Terminal grid widget                                        #
# ─────────────────────────────────────────────────────────── #

MAX_COLS = 3


class TerminalGridWidget(QWidget):
    """
    Displays N terminal panes filling left-to-right, wrapping to a new
    row every MAX_COLS (3) panes.

    Layout structure:
        QVBoxLayout  (outer, one entry per row)
          └── QSplitter(Horizontal)  — one per row of up to 3 panes
                ├── _TerminalPane
                ├── _TerminalPane
                └── _TerminalPane

    Column splitters within each row are the QSplitter handles.
    Rows grow/shrink equally via stretch=1.

    Call `launch(commands)` to append new panes; existing panes stay alive.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._panes: list[_TerminalPane] = []

        self._outer = QVBoxLayout(self)
        self._outer.setContentsMargins(4, 4, 4, 4)
        self._outer.setSpacing(6)

        self._placeholder = QLabel(
            "No active terminals.\nCheck commands and click ▶ Start All."
        )
        self._placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._placeholder.setStyleSheet("color: #555; font-size: 11px;")
        self._outer.addWidget(self._placeholder)

    # ------------------------------------------------------------------ #
    #  Public API                                                         #
    # ------------------------------------------------------------------ #

    def launch(self, commands: list[tuple[str, str]]):
        """Append new panes for each (name, command) tuple and re-layout."""
        if not commands:
            return

        self._placeholder.hide()

        for name, cmd in commands:
            pane = _TerminalPane(name, cmd, self)
            pane.close_requested.connect(self._on_pane_close_requested)
            self._panes.append(pane)

        self._relayout()

    # ------------------------------------------------------------------ #
    #  Internal layout                                                    #
    # ------------------------------------------------------------------ #

    def _relayout(self):
        """Tear down every QSplitter row and rebuild from self._panes."""
        while self._outer.count():
            item = self._outer.takeAt(0)
            w = item.widget()
            if w and w is not self._placeholder:
                if isinstance(w, QSplitter):
                    while w.count():
                        child = w.widget(0)
                        child.setParent(None)  # detach, don't delete
                w.deleteLater()

        if not self._panes:
            self._placeholder.show()
            self._outer.addWidget(self._placeholder)
            return

        for row_panes in _chunk(self._panes, MAX_COLS):
            row_splitter = QSplitter(Qt.Orientation.Horizontal, self)
            row_splitter.setHandleWidth(4)
            row_splitter.setChildrenCollapsible(False)
            row_splitter.setSizePolicy(
                QSizePolicy.Policy.Expanding,
                QSizePolicy.Policy.Expanding,
            )

            for pane in row_panes:
                pane.setParent(row_splitter)
                row_splitter.addWidget(pane)
                pane.show()

            row_splitter.setSizes([100] * len(row_panes))
            self._outer.addWidget(row_splitter, stretch=1)

    # ------------------------------------------------------------------ #
    #  Handlers                                                           #
    # ------------------------------------------------------------------ #

    def _on_pane_close_requested(self, pane: _TerminalPane):
        pane.stop()
        self._panes.remove(pane)
        pane.setParent(None)
        pane.deleteLater()
        self._relayout()

    # ------------------------------------------------------------------ #
    #  Public helpers                                                     #
    # ------------------------------------------------------------------ #

    def stop_all(self):
        for pane in self._panes:
            pane.stop()

    def clear_panes(self):
        """Stop and destroy all panes, reset to placeholder."""
        for pane in self._panes:
            pane.stop()
        self._panes.clear()
        self._relayout()


# ------------------------------------------------------------------ #
#  Helpers                                                            #
# ------------------------------------------------------------------ #

def _chunk(lst: list, n: int) -> list[list]:
    """Split lst into sublists of at most n items."""
    return [lst[i:i + n] for i in range(0, len(lst), n)]