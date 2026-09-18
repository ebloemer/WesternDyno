WESTERN_DARK = """
QWidget {
    background: #0b0d12;
    color: #e8e9ef;
    font-family: "Segoe UI";
    font-size: 10.5pt;
}
QMainWindow { background: #080a0f; }
QFrame#sidebar { background: #11141c; border-right: 1px solid #292d39; }
QFrame#topbar, QFrame#safetybar { background: #10131a; border: 1px solid #292d39; }
QFrame#card, QGroupBox {
    background: #131720;
    border: 1px solid #2b303d;
    border-radius: 8px;
}
QGroupBox { margin-top: 12px; padding: 14px 10px 10px 10px; font-weight: 600; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; color: #c9b4ff; }
QLabel#brand { font-size: 17pt; font-weight: 700; color: #ffffff; }
QLabel#muted { color: #9298a8; }
QLabel#metricValue { font-size: 25pt; font-weight: 650; color: #ffffff; }
QLabel#metricUnit { color: #9ba1b1; font-size: 9pt; }
QLabel#pageTitle { font-size: 20pt; font-weight: 650; color: #ffffff; }
QLabel#engineeringValue { font-size: 11pt; font-weight: 600; color: #ffffff; padding: 4px 12px 4px 4px; }
QLabel#manualBanner { background: #252a36; border: 1px solid #383e4d; border-radius: 6px; padding: 9px; font-weight: 700; }
QLabel#manualBanner[live="true"] { background: #9b1f2d; border-color: #df3d4f; color: white; }
QPushButton {
    background: #252a36;
    border: 1px solid #383e4d;
    border-radius: 6px;
    padding: 8px 13px;
}
QPushButton:hover { background: #303646; border-color: #6e4db7; }
QPushButton:pressed { background: #3a2b59; }
QPushButton:disabled { color: #666c79; background: #191c24; border-color: #252934; }
QPushButton#nav { text-align: left; border: none; background: transparent; padding: 11px 14px; }
QPushButton#nav:checked { background: #2b1d42; color: #d9c8ff; border-left: 3px solid #8f63d9; }
QPushButton#primary { background: #5d36a0; border-color: #8058c8; font-weight: 600; }
QPushButton#primary:hover { background: #7045b7; }
QPushButton#danger { background: #9b1f2d; border-color: #df3d4f; font-weight: 700; }
QPushButton#danger:hover { background: #be293a; }
QPushButton#disarm { background: #d98516; color: #111111; border-color: #f3ad42; font-weight: 700; }
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QTextEdit {
    background: #0d1016;
    border: 1px solid #343a49;
    border-radius: 5px;
    padding: 6px;
    min-height: 24px;
    selection-background-color: #6e47ad;
}
QSpinBox, QDoubleSpinBox { padding-right: 30px; }
QSpinBox::up-button, QDoubleSpinBox::up-button {
    subcontrol-origin: border;
    subcontrol-position: top right;
    width: 26px;
    min-height: 15px;
    border-left: 1px solid #343a49;
    border-bottom: 1px solid #343a49;
}
QSpinBox::down-button, QDoubleSpinBox::down-button {
    subcontrol-origin: border;
    subcontrol-position: bottom right;
    width: 26px;
    min-height: 15px;
    border-left: 1px solid #343a49;
}
QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover { background: #303646; }
QSlider::groove:horizontal { height: 7px; background: #252a36; border-radius: 3px; }
QSlider::sub-page:horizontal { background: #5d36a0; border-radius: 3px; }
QSlider::handle:horizontal { background: #d9c8ff; border: 1px solid #8058c8; width: 18px; margin: -6px 0; border-radius: 9px; }
QScrollArea { border: none; background: transparent; }
QScrollArea > QWidget > QWidget { background: transparent; }
QMenu { background: #181c26; border: 1px solid #383e4d; padding: 5px; }
QMenu::item { padding: 7px 28px 7px 12px; border-radius: 4px; }
QMenu::item:selected { background: #2b1d42; }
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {
    border-color: #8f63d9;
}
QProgressBar { background: #0d1016; border: 1px solid #343a49; border-radius: 5px; text-align: center; }
QProgressBar::chunk { background: #7045b7; border-radius: 4px; }
QTableWidget { background: #0d1016; alternate-background-color: #121620; gridline-color: #282d39; }
QHeaderView::section { background: #181c26; color: #c9ced9; border: none; padding: 7px; }
QToolTip { background: #1b202b; color: white; border: 1px solid #6e4db7; }
"""


def build_theme(preferences) -> str:
    """Apply the persisted RGB palette while retaining readable derived shades."""
    accent = _rgb(preferences.accent_rgb)
    accent_light = _blend(preferences.accent_rgb, (255, 255, 255), 0.28)
    accent_dark = _blend(preferences.accent_rgb, (0, 0, 0), 0.40)
    background = _rgb(preferences.background_rgb)
    background_dark = _blend(preferences.background_rgb, (0, 0, 0), 0.28)
    panel = _rgb(preferences.panel_rgb)
    panel_light = _blend(preferences.panel_rgb, (255, 255, 255), 0.06)
    warning = _rgb(preferences.warning_rgb)
    warning_light = _blend(preferences.warning_rgb, (255, 255, 255), 0.22)
    return (
        WESTERN_DARK
        .replace("#0b0d12", background)
        .replace("#080a0f", background_dark)
        .replace("#131720", panel)
        .replace("#11141c", panel_light)
        .replace("#5d36a0", accent)
        .replace("#7045b7", accent_light)
        .replace("#8058c8", accent_light)
        .replace("#8f63d9", accent_light)
        .replace("#6e4db7", accent)
        .replace("#3a2b59", accent_dark)
        .replace("#2b1d42", accent_dark)
        .replace("#9b1f2d", warning)
        .replace("#be293a", warning_light)
        .replace("#df3d4f", warning_light)
    )


def _rgb(value) -> str:
    red, green, blue = (max(0, min(255, int(item))) for item in value)
    return f"#{red:02x}{green:02x}{blue:02x}"


def _blend(source, target, amount: float) -> str:
    mixed = tuple(round(float(a) + (float(b) - float(a)) * amount) for a, b in zip(source, target))
    return _rgb(mixed)
