"""Theme – single source of truth for all colour/style constants."""


class Theme:
    BG           = "#0A0E14"
    PANEL        = "#0D1826"
    PANEL_BORDER = "#1E2D40"
    TEXT         = "#E2E8F0"
    MUTED        = "#8B9BB4"

    CYAN        = "#00E5FF"
    CYAN_HOVER  = "rgba(0, 229, 255, 0.15)"
    CYAN_LOCKED = "rgba(0, 229, 255, 0.3)"

    SAFE_CORE = "#00FFcc"
    SAFE_MID  = "#008C73"
    SAFE_EDGE = "#00332B"

    WARN_CORE = "#FF3366"
    WARN_MID  = "#B20000"
    WARN_EDGE = "#4A0000"

    ROS_ACTIVE    = "#00FF88"
    ROS_ACTIVE_BG = "rgba(0, 255, 136, 0.15)"

    @staticmethod
    def to_rgba(hex_color: str, alpha: float = 1.0) -> list[float]:
        h = hex_color.lstrip("#")
        return [int(h[i : i + 2], 16) / 255.0 for i in (0, 2, 4)] + [alpha]

    # ------------------------------------------------------------------
    # Application-level stylesheet (call once on QApplication)
    # ------------------------------------------------------------------
    @classmethod
    def app_stylesheet(cls) -> str:
        return f"""
            QMainWindow {{ background-color: {cls.BG}; color: {cls.TEXT}; }}
            QGroupBox {{
                font-weight: 800; font-size: 13px; text-transform: uppercase;
                border: 1px solid {cls.PANEL_BORDER}; border-radius: 8px;
                margin-top: 20px; color: {cls.CYAN}; background-color: {cls.PANEL};
            }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 15px; padding: 0 8px; }}
            QPushButton {{
                background-color: #152336; border: 1px solid {cls.PANEL_BORDER};
                color: {cls.CYAN}; padding: 10px; border-radius: 6px; font-weight: bold;
            }}
            QPushButton:hover {{ background-color: {cls.CYAN}; color: {cls.BG}; border: 1px solid #00FFFF; }}
            QPushButton:pressed {{ background-color: #00B3CC; }}
            QPushButton:checked {{
                background-color: {cls.ROS_ACTIVE_BG}; border: 1px solid {cls.ROS_ACTIVE};
                color: {cls.ROS_ACTIVE};
            }}
            QPushButton:disabled {{ color: #3A4A5A; border-color: #1E2D40; }}

            QSlider::groove:horizontal {{ border: 1px solid {cls.PANEL_BORDER}; height: 4px; background: {cls.BG}; border-radius: 2px; }}
            QSlider::handle:horizontal {{ background: {cls.CYAN}; width: 16px; margin: -6px 0; border-radius: 8px; }}
            QSlider::handle:horizontal:hover {{ background: #FFFFFF; border: 2px solid {cls.CYAN}; width: 18px; margin: -7px 0; border-radius: 9px; }}
            QSlider::sub-page:horizontal {{ background: rgba(0, 229, 255, 0.4); border-radius: 2px; }}

            QScrollArea, QScrollArea > QWidget > QWidget {{ background-color: transparent; border: none; }}
            QScrollBar:vertical {{ background: {cls.BG}; width: 10px; margin: 0px; }}
            QScrollBar::handle:vertical {{ background: {cls.PANEL_BORDER}; min-height: 20px; border-radius: 5px; }}
            QScrollBar::handle:vertical:hover {{ background: {cls.CYAN}; }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0px; }}

            QLabel {{ color: {cls.TEXT}; font-size: 13px; font-family: 'Segoe UI', sans-serif; }}

            QDoubleSpinBox, QSpinBox {{
                background-color: #152336; border: 1px solid {cls.PANEL_BORDER};
                color: {cls.TEXT}; padding: 4px 8px; border-radius: 4px; font-size: 12px;
            }}
            QDoubleSpinBox:focus, QSpinBox:focus {{
                border: 1px solid {cls.CYAN};
            }}
        """
