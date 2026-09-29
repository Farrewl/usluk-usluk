"""app/gui_theme.py — Tema gelap tunggal untuk seluruh GUI (gaya OpenCode).

Glassmorphism di Qt Widgets dibuat PALSu: kartu semi-transparan
(rgba) + border tipis + radius besar, TANPA blur (blur asli
memakan CPU dan membunuh 25 fps di Raspberry Pi).

Satu file ini mengatur seluruh stylesheet; main.py hanya memanggil
`apply_theme(app)`. Warna aksen konsisten:
  emerald = OK / AUTO, amber = MANUAL / waspada, red = KILL / bahaya.
"""

DARK_QSS = """
QMainWindow, QWidget {
    background: #0e1113;
    color: #e6edf3;
    font-family: "Inter", "Segoe UI", sans-serif;
}
QLabel { background: transparent; }
QLabel.card-title {
    font-size: 11px; font-weight: bold; letter-spacing: 2px;
    color: #8b949e; background: transparent;
}
QLabel.big-value {
    font-family: monospace; font-weight: bold;
    background: transparent;
}
/* --- kartu glass (tanpa blur) --- */
QFrame.card, QWidget.card {
    background: rgba(255, 255, 255, 0.06);
    border: 1px solid rgba(255, 255, 255, 0.12);
    border-radius: 14px;
}
QPushButton {
    background: rgba(255, 255, 255, 0.08);
    border: 1px solid rgba(255, 255, 255, 0.14);
    border-radius: 10px; padding: 8px 14px; color: #e6edf3;
}
QPushButton:hover { background: rgba(255, 255, 255, 0.14); }
QPushButton:checked { background: rgba(243, 156, 18, 0.25); }
QPushButton#killBtn {
    background: #c0392b; border: none; border-radius: 12px;
    font-weight: bold; font-size: 16px; padding: 12px;
}
QPushButton#killBtn:hover { background: #e74c3c; }
QPushButton#killBtn:checked { background: #ff3b30; }
QPushButton.toolBtn { padding: 4px 10px; font-size: 12px; border-radius: 8px; }
QSplitter::handle { background: rgba(255, 255, 255, 0.08); }
QTabWidget::pane { border: none; background: transparent; }
QTabBar::tab {
    background: rgba(255, 255, 255, 0.05); color: #8b949e;
    padding: 8px 18px; border-top-left-radius: 10px;
    border-top-right-radius: 10px; margin-right: 4px;
}
QTabBar::tab:selected { background: rgba(255, 255, 255, 0.12); color: #ffffff; }
QProgressBar {
    background: rgba(255, 255, 255, 0.07); border: none;
    border-radius: 6px; text-align: center; color: #e6edf3;
    font-family: monospace; height: 16px;
}
QProgressBar::chunk { background: #2ecc71; border-radius: 6px; }
QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox {
    background: rgba(255, 255, 255, 0.07);
    border: 1px solid rgba(255, 255, 255, 0.14);
    border-radius: 8px; padding: 5px; color: #e6edf3;
}
QCheckBox { background: transparent; spacing: 8px; }
QStatusBar { background: #0e1113; color: #8b949e; }
QStatusBar::item { border: none; }
QScrollBar:vertical { background: transparent; width: 10px; }
QScrollBar::handle:vertical {
    background: rgba(255, 255, 255, 0.15); border-radius: 5px;
    min-height: 30px;
}
"""

# Warna teks status (dipakai via QSS `color:` sekali saat berubah,
# BUKAN tiap frame — lihat main.py _set_chip agar tidak lag).
COLOR_OK = "#2ecc71"
COLOR_WARN = "#f39c12"
COLOR_BAD = "#ff6b6b"
COLOR_IDLE = "#95a5a6"
COLOR_ACCENT = "#00d4aa"


def apply_theme(app):
    """Terapkan stylesheet gelap ke QApplication."""
    app.setStyleSheet(DARK_QSS)
