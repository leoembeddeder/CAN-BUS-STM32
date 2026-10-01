"""
STM32 Automotive Dashboard
OpCode Labs – 2026
────────────────────────────────────────────────────────────
Frame format (TX and RX):
  [ 0xA5 ][ CMD: 1 byte ][ DATA: 4 bytes big-endian uint32 ] = 6 bytes total

Dependencies:
  pip install pyserial PyQt5
────────────────────────────────────────────────────────────
"""

import sys
import struct
import serial
import serial.tools.list_ports
import math
from collections import deque
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox,
    QGroupBox, QTextEdit, QSplitter,
    QFrame, QProgressBar, QStatusBar,
    QCheckBox, QSizePolicy
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QRectF, QPointF
from PyQt5.QtGui import (
    QFont, QColor, QPainter, QPen, QBrush,
    QLinearGradient, QRadialGradient, QPainterPath, QPolygonF, QConicalGradient
)

# ═══════════════════════════════════════════
#  PROTOCOL
# ═══════════════════════════════════════════
SOF = 0xA5
FRAME_LEN = 6

CMD_TRIGGER_ABS  = 0x11  # kept for ABS trigger button (but note: also CMD_CAN_321 = 0x11)
CMD_CAN_406      = 0x30
CMD_CAN_55C      = 0x31
CMD_CAN_679      = 0x88
CMD_CAN_456      = 0x33
CMD_CAN_401      = 0x35
CMD_WINDOW_RIGHT = 0x87
CMD_WINDOW_LEFT  = 0x89

# New commands
CMD_CAN_321      = 0x11  # Vitesse
CMD_CAN_390      = 0x10  # Débit air
CMD_CAN_790      = 0x12  # Porte droite notification
CMD_CAN_791      = 0x13  # Porte gauche button + notification
CMD_PORTE_GAUCHE = 0x13  # Alias for button

RX_SENSOR_NAMES = {
    CMD_CAN_406: "CAN Speed Motor 0x406",
    CMD_CAN_55C: "CAN Pressure 0x55C",
    CMD_CAN_679: "CAN ABS 0x679",
    CMD_CAN_456: "CAN Temp 0x456",
    CMD_CAN_401: "CAN Window Left 0x401",
    CMD_CAN_321: "CAN Vitesse 0x321",
    CMD_CAN_390: "CAN Débit Air 0x390",
    CMD_CAN_790: "CAN Porte Droite 0x790",
    CMD_CAN_791: "CAN Porte Gauche 0x791",
}

# ═══════════════════════════════════════════
#  PALETTE  — dark cockpit / motorsport
# ═══════════════════════════════════════════
C_BG      = "#0A0C10"
C_PANEL   = "#11141B"
C_CARD    = "#181C25"
C_BORDER  = "#2A2F3D"
C_AMBER   = "#F0A500"
C_AMBER_D = "#6A4800"
C_GREEN   = "#00E676"
C_RED     = "#FF3D3D"
C_BLUE    = "#2979FF"
C_CYAN    = "#00E5FF"
C_PURPLE  = "#AA00FF"
C_TEXT    = "#CDD3E8"
C_MUTED   = "#555E78"
C_DIAL_BG = "#0D1018"
C_NEEDLE  = "#FF3D3D"
C_TICK    = "#3A4255"
C_TICK_LIT= "#CDD3E8"

C_GAUGE_ACCENT = C_AMBER

QSS = f"""
* {{
    font-family: "Consolas", "Courier New", monospace;
    font-size: 12px;
    color: {C_TEXT};
}}
QMainWindow, QWidget {{ background-color: {C_BG}; }}

QGroupBox {{
    background-color: {C_PANEL};
    border: 1px solid {C_BORDER};
    border-radius: 8px;
    margin-top: 18px;
    padding-top: 12px;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    top: 2px;
    color: {C_AMBER};
    font-size: 9px;
    font-weight: bold;
    letter-spacing: 3px;
}}

QPushButton {{
    background-color: {C_CARD};
    border: 1px solid {C_BORDER};
    border-radius: 5px;
    padding: 6px 10px;
    min-height: 28px;
    font-weight: bold;
    font-size: 11px;
}}
QPushButton:hover   {{ background-color: {C_AMBER_D}; border-color: {C_AMBER}; color: {C_AMBER}; }}
QPushButton:pressed {{ background-color: {C_AMBER};   color: #111; }}
QPushButton:disabled {{ background-color: {C_PANEL};  color: {C_MUTED}; border-color: {C_BORDER}; }}

QPushButton#btn_connect    {{ border-color:{C_GREEN}; color:{C_GREEN}; background:#061A10; }}
QPushButton#btn_connect:hover {{ background:{C_GREEN}; color:#000; }}
QPushButton#btn_disconnect {{ border-color:{C_RED};   color:{C_RED};   background:#1A0606; }}
QPushButton#btn_disconnect:hover {{ background:{C_RED}; color:#fff; }}

QComboBox {{
    background-color: {C_CARD};
    border: 1px solid {C_BORDER};
    border-radius: 4px;
    padding: 4px 7px;
    min-height: 26px;
}}
QComboBox::drop-down {{ border: none; width: 20px; }}
QComboBox QAbstractItemView {{
    background: {C_CARD};
    border: 1px solid {C_AMBER};
    selection-background-color: {C_AMBER_D};
}}

QTextEdit {{
    background-color: #060810;
    border: 1px solid {C_BORDER};
    border-radius: 4px;
    color: #5AFF8A;
    font-size: 11px;
    padding: 3px;
}}

QProgressBar {{
    background-color: {C_CARD};
    border: 1px solid {C_BORDER};
    border-radius: 3px;
    max-height: 6px;
}}
QProgressBar::chunk {{ border-radius: 3px; }}

QCheckBox {{ spacing: 6px; color: {C_TEXT}; }}
QCheckBox::indicator {{
    width: 13px; height: 13px;
    border: 1px solid {C_BORDER};
    border-radius: 3px;
    background: {C_CARD};
}}
QCheckBox::indicator:checked {{ background: {C_AMBER}; border-color: {C_AMBER}; }}

QStatusBar {{ background: {C_PANEL}; border-top: 1px solid {C_BORDER}; color: {C_MUTED}; font-size: 11px; }}
QSplitter::handle {{ background: {C_BORDER}; width: 1px; }}
"""


# ═══════════════════════════════════════════
#  SERIAL READER THREAD
# ═══════════════════════════════════════════
class SerialReader(QThread):
    sig_frame = pyqtSignal(int, float)
    sig_raw   = pyqtSignal(bytes)
    sig_error = pyqtSignal(str)

    def __init__(self, port: serial.Serial, parent=None):
        super().__init__(parent)
        self._port   = port
        self._active = True
        self._buf    = bytearray()

    def stop(self):
        self._active = False

    def run(self):
        while self._active:
            try:
                waiting = self._port.in_waiting
                if waiting > 0:
                    self._buf.extend(self._port.read(waiting))
                    self._parse()
                else:
                    self.msleep(5)
            except serial.SerialException as exc:
                self.sig_error.emit(str(exc))
                break

    def _parse(self):
        while len(self._buf) >= FRAME_LEN:
            idx = self._buf.find(SOF)
            if idx == -1:
                self._buf.clear(); return
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < FRAME_LEN:
                return
            frame = bytes(self._buf[:FRAME_LEN])
            del self._buf[:FRAME_LEN]
            cmd   = frame[1]
            value = struct.unpack('>I', frame[2:6])[0]
            self.sig_raw.emit(frame)
            self.sig_frame.emit(cmd, float(value))


# ═══════════════════════════════════════════
#  CIRCULAR GAUGE
# ═══════════════════════════════════════════
class CircularGauge(QWidget):
    def __init__(self, label="", unit="", v_min=0, v_max=255,
                 danger_pct=0.85, parent=None):
        super().__init__(parent)
        self.label      = label
        self.unit       = unit
        self.v_min      = v_min
        self.v_max      = v_max
        self.color      = QColor(C_GAUGE_ACCENT)
        self.danger_pct = danger_pct
        self._value     = 0.0
        self._anim_val  = 0.0
        self._timer     = QTimer(self)
        self._timer.timeout.connect(self._animate)
        self._timer.start(16)
        self.setMinimumSize(160, 175)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_value(self, v: float):
        self._value = max(self.v_min, min(self.v_max, v))

    def _animate(self):
        diff = self._value - self._anim_val
        if abs(diff) > 0.3:
            self._anim_val += diff * 0.18
            self.update()
        elif self._anim_val != self._value:
            self._anim_val = self._value
            self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        W, H   = self.width(), self.height()
        size   = min(W, H - 30)
        cx     = W / 2
        cy     = (H - 30) / 2 + 4
        r      = size / 2 - 10

        grad = QRadialGradient(cx, cy, r)
        grad.setColorAt(0.0, QColor("#1A1F2E"))
        grad.setColorAt(1.0, QColor(C_DIAL_BG))
        p.setBrush(QBrush(grad))
        p.setPen(QPen(QColor(C_BORDER), 2))
        p.drawEllipse(QPointF(cx, cy), r, r)

        start_angle = 225
        sweep_angle = 270

        pen = QPen(QColor(C_TICK), r * 0.07, Qt.SolidLine, Qt.RoundCap)
        p.setPen(pen)
        rect = QRectF(cx - r*0.78, cy - r*0.78, r*1.56, r*1.56)
        p.drawArc(rect, int(start_angle*16), int(-sweep_angle*16))

        pct = (self._anim_val - self.v_min) / max(self.v_max - self.v_min, 1)
        pct = max(0.0, min(1.0, pct))

        arc_color = QColor(C_RED) if pct > self.danger_pct else self.color

        pen2 = QPen(arc_color, r * 0.07, Qt.SolidLine, Qt.RoundCap)
        p.setPen(pen2)
        p.drawArc(rect, int(start_angle*16), int(-sweep_angle*pct*16))

        num_ticks = 10
        for i in range(num_ticks + 1):
            frac  = i / num_ticks
            angle = math.radians(start_angle - frac * sweep_angle)
            is_maj = (i % 2 == 0)
            r_out  = r * 0.68
            r_in   = r * (0.55 if is_maj else 0.60)
            x1 = cx + r_out * math.cos(angle)
            y1 = cy - r_out * math.sin(angle)
            x2 = cx + r_in  * math.cos(angle)
            y2 = cy - r_in  * math.sin(angle)
            col = QColor(C_TICK_LIT) if is_maj else QColor(C_TICK)
            p.setPen(QPen(col, 1.5 if is_maj else 1.0))
            p.drawLine(QPointF(x1, y1), QPointF(x2, y2))

            if is_maj:
                val_at = self.v_min + frac * (self.v_max - self.v_min)
                lbl_r  = r * 0.43
                lx = cx + lbl_r * math.cos(angle) - 14
                ly = cy - lbl_r * math.sin(angle) - 8
                p.setFont(QFont("Consolas", max(6, int(r * 0.085))))
                p.setPen(QColor(C_MUTED))
                p.drawText(QRectF(lx, ly, 28, 16), Qt.AlignCenter, f"{val_at:.0f}")

        needle_angle = math.radians(start_angle - pct * sweep_angle)
        nx = cx + r * 0.60 * math.cos(needle_angle)
        ny = cy - r * 0.60 * math.sin(needle_angle)
        p.setPen(QPen(QColor(C_NEEDLE), 2.5, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(cx, cy), QPointF(nx, ny))

        p.setBrush(QColor(C_BORDER))
        p.setPen(QPen(QColor(C_MUTED), 1))
        p.drawEllipse(QPointF(cx, cy), r * 0.07, r * 0.07)

        glow = QColor(C_NEEDLE); glow.setAlpha(180)
        p.setBrush(glow); p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(nx, ny), 3, 3)

        val_str   = f"{self._anim_val:.0f}"
        color_now = QColor(C_RED) if pct > self.danger_pct else arc_color
        p.setFont(QFont("Consolas", max(14, int(r*0.22)), QFont.Bold))
        p.setPen(color_now)
        p.drawText(QRectF(cx - r, cy + r*0.10, r*2, r*0.40), Qt.AlignCenter, val_str)

        p.setFont(QFont("Consolas", max(8, int(r*0.10))))
        p.setPen(QColor(C_MUTED))
        p.drawText(QRectF(cx - r, cy + r*0.38, r*2, r*0.24), Qt.AlignCenter, self.unit)

        p.setFont(QFont("Consolas", max(9, int(r*0.11)), QFont.Bold))
        p.setPen(self.color)
        p.drawText(QRectF(0, H - 26, W, 22), Qt.AlignCenter, self.label.upper())


# ═══════════════════════════════════════════
#  WARNING LIGHT
# ═══════════════════════════════════════════
class WarningLight(QWidget):
    def __init__(self, label="ABS", color="#FF3D3D", parent=None):
        super().__init__(parent)
        self.label   = label
        self.color   = QColor(color)
        self._active = False
        self._blink  = False
        self._timer  = QTimer(self)
        self._timer.timeout.connect(self._toggle_blink)
        self.setFixedSize(80, 60)

    def set_active(self, state: bool):
        self._active = state
        if state:
            self._timer.start(400)
        else:
            self._timer.stop()
            self._blink = False
            self.update()

    def _toggle_blink(self):
        self._blink = not self._blink
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        lit  = self._active and self._blink

        if lit:
            glow = QColor(self.color); glow.setAlpha(40)
            p.setBrush(glow); p.setPen(Qt.NoPen)
            p.drawRoundedRect(2, 2, W-4, H-4-18, 8, 8)

        body_color = self.color if lit else QColor(C_CARD)
        border_col = self.color if self._active else QColor(C_BORDER)
        p.setBrush(body_color)
        p.setPen(QPen(border_col, 1.5))
        p.drawRoundedRect(6, 4, W-12, H-24, 6, 6)

        p.setFont(QFont("Consolas", 9, QFont.Bold))
        text_col = QColor("#111") if lit else (self.color if self._active else QColor(C_MUTED))
        p.setPen(text_col)
        p.drawText(QRectF(6, 4, W-12, H-24), Qt.AlignCenter, self.label)

        p.setFont(QFont("Consolas", 8))
        p.setPen(QColor(C_MUTED) if not self._active else self.color)
        p.drawText(QRectF(0, H-18, W, 16), Qt.AlignCenter, self.label)


# ═══════════════════════════════════════════
#  MINI BAR GAUGE
# ═══════════════════════════════════════════
class MiniGauge(QFrame):
    def __init__(self, title, unit, v_min, v_max, parent=None):
        super().__init__(parent)
        color = C_GAUGE_ACCENT
        self._color = color
        self._vmin  = v_min
        self._vmax  = v_max
        self.setStyleSheet(
            f"QFrame {{ background:{C_CARD}; border:1px solid {C_BORDER}; border-radius:7px; }}"
        )
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(2)

        row = QHBoxLayout()
        lbl_t = QLabel(title.upper())
        lbl_t.setStyleSheet(
            f"color:{C_MUTED}; font-size:9px; letter-spacing:1px; border:none; background:transparent;"
        )
        self.lbl_val = QLabel("—")
        self.lbl_val.setStyleSheet(
            f"color:{color}; font-size:16px; font-weight:bold; border:none; background:transparent;"
        )
        lbl_u = QLabel(unit)
        lbl_u.setStyleSheet(
            f"color:{C_MUTED}; font-size:9px; border:none; background:transparent;"
        )
        row.addWidget(lbl_t)
        row.addStretch()
        row.addWidget(self.lbl_val)
        row.addWidget(lbl_u)
        lay.addLayout(row)

        self.bar = QProgressBar()
        self.bar.setRange(0, 1000)
        self.bar.setValue(0)
        self.bar.setTextVisible(False)
        self.bar.setStyleSheet(
            f"QProgressBar {{ background:{C_PANEL}; border:1px solid {C_BORDER}; "
            f"border-radius:3px; max-height:5px; }}"
            f"QProgressBar::chunk {{ background:{color}; border-radius:3px; }}"
        )
        lay.addWidget(self.bar)
        self.setFixedHeight(62)

    def set_value(self, val: float):
        self.lbl_val.setText(f"{val:.0f}")
        span = max(self._vmax - self._vmin, 1e-9)
        pct  = int(1000 * (val - self._vmin) / span)
        self.bar.setValue(max(0, min(1000, pct)))


# ═══════════════════════════════════════════
#  CLOCK WIDGET
# ═══════════════════════════════════════════
class ClockWidget(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("HORLOGE  SYSTÈME", parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 18, 12, 12)
        lay.setSpacing(10)

        self._lbl_time = QLabel("--:--:--")
        self._lbl_time.setAlignment(Qt.AlignCenter)
        self._lbl_time.setStyleSheet(
            f"color:{C_AMBER}; font-size:38px; font-weight:bold; "
            f"letter-spacing:4px; border:none; background:transparent;"
        )
        lay.addWidget(self._lbl_time)

        self._lbl_date = QLabel("---")
        self._lbl_date.setAlignment(Qt.AlignCenter)
        self._lbl_date.setStyleSheet(
            f"color:{C_MUTED}; font-size:12px; letter-spacing:2px; border:none; background:transparent;"
        )
        lay.addWidget(self._lbl_date)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"background:{C_BORDER}; max-height:1px;")
        lay.addWidget(sep)

        row = QHBoxLayout()
        lbl_up = QLabel("UPTIME")
        lbl_up.setStyleSheet(f"color:{C_MUTED}; font-size:9px; letter-spacing:2px; border:none; background:transparent;")
        self._lbl_uptime = QLabel("00:00:00")
        self._lbl_uptime.setStyleSheet(f"color:{C_TEXT}; font-size:11px; border:none; background:transparent;")
        row.addWidget(lbl_up)
        row.addStretch()
        row.addWidget(self._lbl_uptime)
        lay.addLayout(row)

        row2 = QHBoxLayout()
        lbl_fr = QLabel("TRAMES RX")
        lbl_fr.setStyleSheet(f"color:{C_MUTED}; font-size:9px; letter-spacing:2px; border:none; background:transparent;")
        self._lbl_frames = QLabel("0")
        self._lbl_frames.setStyleSheet(f"color:{C_AMBER}; font-size:11px; font-weight:bold; border:none; background:transparent;")
        row2.addWidget(lbl_fr)
        row2.addStretch()
        row2.addWidget(self._lbl_frames)
        lay.addLayout(row2)

        lay.addStretch()

        self._start = datetime.now()
        self._tick  = QTimer(self)
        self._tick.timeout.connect(self._update)
        self._tick.start(1000)
        self._update()

    def _update(self):
        now = datetime.now()
        self._lbl_time.setText(now.strftime("%H:%M:%S"))
        days = ["Lun", "Mar", "Mer", "Jeu", "Ven", "Sam", "Dim"]
        months = ["Jan","Fév","Mar","Avr","Mai","Jun","Jul","Aoû","Sep","Oct","Nov","Déc"]
        self._lbl_date.setText(
            f"{days[now.weekday()]}  {now.day:02d} {months[now.month-1]} {now.year}"
        )
        delta = now - self._start
        total = int(delta.total_seconds())
        h, rem = divmod(total, 3600)
        m, s   = divmod(rem, 60)
        self._lbl_uptime.setText(f"{h:02d}:{m:02d}:{s:02d}")

    def set_frame_count(self, n: int):
        self._lbl_frames.setText(str(n))


def cmd_btn(text: str, color: str) -> QPushButton:
    btn = QPushButton(text)
    btn.setStyleSheet(
        f"QPushButton {{ background:{C_CARD}; border:1px solid {color}; border-radius:5px; "
        f"padding:6px 8px; color:{color}; font-weight:bold; min-height:28px; font-size:11px; }}"
        f"QPushButton:hover  {{ background:{color}; color:#000; }}"
        f"QPushButton:pressed {{ background:{color}; color:#000; }}"
        f"QPushButton:disabled {{ background:{C_PANEL}; color:{C_MUTED}; border-color:{C_BORDER}; }}"
    )
    return btn


# ═══════════════════════════════════════════
#  MAIN WINDOW
# ═══════════════════════════════════════════
class STM32Dashboard(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpCode Labs  •  CAN Automotive Dashboard")
        self.resize(1380, 860)
        self.setStyleSheet(QSS)

        self._serial   = None
        self._reader   = None
        self._cmd_widgets = []
        self._window_frame_active              = False
        self._abs_notification_active          = False
        self._window_left_notification_active  = False
        self._porte_droite_notification_active = False
        self._porte_gauche_notification_active = False
        self._build_ui()
        self._refresh_ports()
        self._apply_connected(False)

        self._tick = QTimer(self)
        self._tick.timeout.connect(self._update_clock)
        self._tick.start(1000)

    # ──────────────────────────────────────
    def _build_ui(self):
        root_w = QWidget()
        self.setCentralWidget(root_w)
        root = QVBoxLayout(root_w)
        root.setContentsMargins(10, 8, 10, 4)
        root.setSpacing(6)

        root.addWidget(self._mk_header())

        body = QHBoxLayout()
        body.setSpacing(8)

        left = QWidget(); left.setFixedWidth(240)
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0,0,0,0); lv.setSpacing(7)
        lv.addWidget(self._mk_connection())
        lv.addWidget(self._mk_commands())
        lv.addStretch()
        body.addWidget(left)

        center = QWidget()
        cv = QVBoxLayout(center)
        cv.setContentsMargins(0,0,0,0); cv.setSpacing(7)
        cv.addWidget(self._mk_main_gauges(), stretch=3)
        cv.addWidget(self._mk_secondary_gauges(), stretch=2)
        cv.addWidget(self._mk_warning_bar())
        body.addWidget(center, stretch=1)

        right = QWidget(); right.setFixedWidth(310)
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0,0,0,0); rv.setSpacing(7)
        self._clock_widget = ClockWidget()
        rv.addWidget(self._clock_widget)
        rv.addWidget(self._mk_log(), stretch=1)
        body.addWidget(right)

        root.addLayout(body, stretch=1)

        self._status = QStatusBar()
        self.setStatusBar(self._status)
        self._status.showMessage("Disconnected  •  OpCode Labs CAN Node")

    # ──────────────────────────────────────
    def _mk_header(self) -> QWidget:
        w = QFrame()
        w.setStyleSheet(
            f"QFrame {{ background:{C_PANEL}; border:1px solid {C_BORDER}; border-radius:8px; }}"
        )
        h = QHBoxLayout(w)
        h.setContentsMargins(18, 6, 18, 6)

        dot = QLabel("◈")
        dot.setStyleSheet(f"color:{C_AMBER}; font-size:18px; border:none; background:transparent;")
        h.addWidget(dot)

        title = QLabel("OPCODE LABS  —  CAN AUTOMOTIVE DASHBOARD")
        title.setStyleSheet(
            f"color:{C_AMBER}; font-size:14px; font-weight:bold; "
            f"letter-spacing:4px; border:none; background:transparent;"
        )
        h.addWidget(title)
        h.addStretch()

        self._lbl_clock = QLabel("--:--:--")
        self._lbl_clock.setStyleSheet(
            f"color:{C_MUTED}; font-size:11px; border:none; background:transparent;"
        )
        h.addWidget(self._lbl_clock)
        h.addSpacing(20)

        self._lbl_status = QLabel("● OFFLINE")
        self._lbl_status.setStyleSheet(
            f"color:{C_RED}; font-weight:bold; font-size:12px; border:none; background:transparent;"
        )
        h.addWidget(self._lbl_status)
        return w

    # ──────────────────────────────────────
    def _mk_connection(self) -> QGroupBox:
        grp = QGroupBox("CONNECTION")
        lay = QVBoxLayout(grp); lay.setSpacing(6)

        r1 = QHBoxLayout()
        self._combo_port = QComboBox()
        self._btn_refresh = QPushButton("⟳")
        self._btn_refresh.setFixedWidth(30)
        self._btn_refresh.clicked.connect(self._refresh_ports)
        r1.addWidget(QLabel("Port:")); r1.addWidget(self._combo_port, 1); r1.addWidget(self._btn_refresh)
        lay.addLayout(r1)

        r2 = QHBoxLayout()
        self._combo_baud = QComboBox()
        for b in ["9600","19200","38400","57600","115200","230400"]:
            self._combo_baud.addItem(b)
        self._combo_baud.setCurrentText("115200")
        r2.addWidget(QLabel("Baud:")); r2.addWidget(self._combo_baud, 1)
        lay.addLayout(r2)

        br = QHBoxLayout()
        self._btn_conn = QPushButton("CONNECT");    self._btn_conn.setObjectName("btn_connect")
        self._btn_disc = QPushButton("DISCONNECT"); self._btn_disc.setObjectName("btn_disconnect")
        self._btn_conn.clicked.connect(self._connect)
        self._btn_disc.clicked.connect(self._disconnect)
        br.addWidget(self._btn_conn); br.addWidget(self._btn_disc)
        lay.addLayout(br)
        return grp

    # ──────────────────────────────────────
    def _mk_commands(self) -> QGroupBox:
        grp = QGroupBox("COMMANDS  →  STM32")
        grid = QGridLayout(grp); grid.setSpacing(5)

        btns = [
            ("Window RIGHT",  CMD_WINDOW_RIGHT,  C_AMBER,   0, 0),
            ("Trigger ABS",   CMD_TRIGGER_ABS,   C_RED,     0, 1),
            ("Window LEFT",   CMD_WINDOW_LEFT,   C_BLUE,    1, 0),
            ("Porte Gauche",  CMD_PORTE_GAUCHE,  C_GREEN,   1, 1),
        ]
        for label, cmd, color, row, col in btns:
            b = cmd_btn(label, color)
            b.clicked.connect(lambda _, c=cmd: self._send(c, 0))
            grid.addWidget(b, row, col)
            self._cmd_widgets.append(b)
            if cmd == CMD_WINDOW_RIGHT:
                self._btn_window_frame = b
            if cmd == CMD_TRIGGER_ABS:
                self._btn_abs = b
            if cmd == CMD_WINDOW_LEFT:
                self._btn_window_left = b
            if cmd == CMD_PORTE_GAUCHE:
                self._btn_porte_gauche = b

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"background:{C_BORDER}; max-height:1px;")
        grid.addWidget(sep, 2, 0, 1, 2)

        lbl_m = QLabel("MANUAL FRAME")
        lbl_m.setStyleSheet(
            f"color:{C_AMBER}; font-size:9px; font-weight:bold; letter-spacing:2px;"
        )
        grid.addWidget(lbl_m, 3, 0, 1, 2)

        self._edit_cmd = QComboBox(); self._edit_cmd.setEditable(True)
        for c in ["0x10","0x11","0x12","0x13","0x87","0x89"]:
            self._edit_cmd.addItem(c)
        self._edit_cmd.setCurrentText("0x10")

        self._edit_data = QComboBox(); self._edit_data.setEditable(True)
        for d in ["0x00000000","0x00000001","0xDEADBEEF"]:
            self._edit_data.addItem(d)
        self._edit_data.setCurrentText("0x00000000")

        grid.addWidget(QLabel("CMD :"), 4, 0); grid.addWidget(self._edit_cmd,  4, 1)
        grid.addWidget(QLabel("DATA:"), 5, 0); grid.addWidget(self._edit_data, 5, 1)

        btn_send = cmd_btn("▶️  SEND FRAME", C_AMBER)
        btn_send.clicked.connect(self._send_manual)
        grid.addWidget(btn_send, 6, 0, 1, 2)
        self._cmd_widgets.append(btn_send)
        return grp

    # ──────────────────────────────────────
    def _mk_main_gauges(self) -> QGroupBox:
        """Top row: original 3 gauges — pressure, speed motor, temperature."""
        grp = QGroupBox("SENSOR  DASHBOARD  ←  STM32")
        h = QHBoxLayout(grp)
        h.setSpacing(16); h.setContentsMargins(10, 16, 10, 10)

        self._gauge_pressure = CircularGauge(label="Pressure",    unit="bar", v_min=0, v_max=255, danger_pct=0.85)
        self._gauge_speed    = CircularGauge(label="Speed Motor", unit="rpm", v_min=0, v_max=255, danger_pct=0.90)
        self._gauge_temp     = CircularGauge(label="Temperature", unit="°C",  v_min=0, v_max=255, danger_pct=0.80)

        h.addWidget(self._gauge_speed)
        h.addWidget(self._gauge_pressure)
        h.addWidget(self._gauge_temp)
        return grp

    # ──────────────────────────────────────
    def _mk_secondary_gauges(self) -> QGroupBox:
        """Second row: new gauges — Vitesse (0x321) and Débit Air (0x390)."""
        grp = QGroupBox("CAPTEURS  ADDITIONNELS  ←  CAN  0x321 / 0x390")
        h = QHBoxLayout(grp)
        h.setSpacing(16); h.setContentsMargins(10, 16, 10, 10)

        self._gauge_vitesse   = CircularGauge(
            label="Vitesse",   unit="km/h", v_min=0, v_max=255, danger_pct=0.90
        )
        self._gauge_debit_air = CircularGauge(
            label="Débit Air", unit="g/s",  v_min=0, v_max=255, danger_pct=0.85
        )

        # Add stretch spacers so the two gauges sit nicely side-by-side centered
        h.addStretch(1)
        h.addWidget(self._gauge_vitesse)
        h.addStretch(1)
        h.addWidget(self._gauge_debit_air)
        h.addStretch(1)
        return grp

    # ──────────────────────────────────────
    def _mk_warning_bar(self) -> QWidget:
        w = QFrame()
        w.setStyleSheet(
            f"QFrame {{ background:{C_PANEL}; border:1px solid {C_BORDER}; border-radius:7px; }}"
        )
        w.setFixedHeight(72)
        h = QHBoxLayout(w)
        h.setContentsMargins(14, 8, 14, 8); h.setSpacing(12)

        lbl = QLabel("WARNINGS")
        lbl.setStyleSheet(
            f"color:{C_MUTED}; font-size:9px; letter-spacing:2px; border:none; background:transparent;"
        )
        h.addWidget(lbl); h.addSpacing(6)

        self._warn_abs          = WarningLight("ABS",       C_AMBER)
        self._warn_window_left  = WarningLight("WIN LEFT",  C_BLUE)
        self._warn_window_right = WarningLight("WIN RIGHT", C_AMBER)
        self._warn_porte_droite = WarningLight("PRT DRTE",  C_GREEN)
        self._warn_porte_gauche = WarningLight("PRT GCHE",  C_PURPLE)
        self._warn_can          = WarningLight("CAN",       C_CYAN)

        for w2 in [self._warn_abs, self._warn_window_left, self._warn_window_right,
                   self._warn_porte_droite, self._warn_porte_gauche, self._warn_can]:
            h.addWidget(w2)

        h.addStretch()

        self._lbl_frames = QLabel("FRAMES  0")
        self._lbl_frames.setStyleSheet(
            f"color:{C_MUTED}; font-size:10px; border:none; background:transparent;"
        )
        h.addWidget(self._lbl_frames)
        self._frame_count = 0
        return w

    # ──────────────────────────────────────
    def _mk_log(self) -> QGroupBox:
        grp = QGroupBox("FRAME  LOG")
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(6, 6, 6, 6); lay.setSpacing(4)

        self._log = QTextEdit(); self._log.setReadOnly(True)
        lay.addWidget(self._log)

        row = QHBoxLayout()
        self._chk_log_tx = QCheckBox("Log TX")
        self._chk_log_rx = QCheckBox("Log RX"); self._chk_log_rx.setChecked(True)
        btn_clr = QPushButton("Clear"); btn_clr.setFixedWidth(65)
        btn_clr.clicked.connect(self._log.clear)
        row.addWidget(self._chk_log_tx); row.addWidget(self._chk_log_rx)
        row.addStretch(); row.addWidget(btn_clr)
        lay.addLayout(row)
        return grp

    # ──────────────────────────────────────
    def _refresh_ports(self):
        self._combo_port.clear()
        ports = serial.tools.list_ports.comports()
        default_index = 0
        if ports:
            for i, p in enumerate(ports):
                desc = (f"{p.device}  –  {p.description[:24]}"
                        if p.description not in ("n/a", "") else p.device)
                self._combo_port.addItem(desc, userData=p.device)
                if p.device == "/dev/ttyACM0":
                    default_index = i
            self._combo_port.setCurrentIndex(default_index)
        else:
            self._combo_port.addItem("No ports detected", userData="")

    # ──────────────────────────────────────
    def _connect(self):
        port = self._combo_port.currentData()
        if not port:
            self._sys_log("No valid port selected."); return
        baud = int(self._combo_baud.currentText())
        try:
            self._serial = serial.Serial(port, baud, timeout=0.1)
            self._reader = SerialReader(self._serial, self)
            self._reader.sig_frame.connect(self._on_frame)
            self._reader.sig_raw.connect(self._on_raw)
            self._reader.sig_error.connect(self._on_serial_error)
            self._reader.start()
            self._apply_connected(True)
            self._warn_can.set_active(True)
            self._status.showMessage(f"Connected  •  {port}  •  {baud} baud")
            self._sys_log(f"Opened {port} @ {baud}")
        except serial.SerialException as exc:
            self._sys_log(f"ERROR: {exc}")

    def _disconnect(self):
        if self._reader:
            self._reader.stop(); self._reader.wait(600); self._reader = None
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None
        self._apply_connected(False)
        self._warn_can.set_active(False)
        self._status.showMessage("Disconnected")
        self._sys_log("Port closed.")

    def _apply_connected(self, state: bool):
        self._btn_conn.setEnabled(not state)
        self._btn_disc.setEnabled(state)
        self._combo_port.setEnabled(not state)
        self._combo_baud.setEnabled(not state)
        for w in self._cmd_widgets:
            w.setEnabled(state)
        if state:
            self._lbl_status.setText("● ONLINE")
            self._lbl_status.setStyleSheet(
                f"color:{C_GREEN}; font-weight:bold; font-size:12px; border:none; background:transparent;"
            )
        else:
            self._lbl_status.setText("● OFFLINE")
            self._lbl_status.setStyleSheet(
                f"color:{C_RED}; font-weight:bold; font-size:12px; border:none; background:transparent;"
            )

    # ──────────────────────────────────────
    def _send(self, cmd: int, data: int = 0):
        if not (self._serial and self._serial.is_open):
            return
        if cmd == CMD_WINDOW_RIGHT:
            self._window_frame_active = not self._window_frame_active
            self._update_window_btn()
        frame = struct.pack('>BBI', SOF, cmd, data)
        self._serial.write(frame)
        if self._chk_log_tx.isChecked():
            self._tx_log(cmd, frame)

    def _send_manual(self):
        try:
            cmd  = int(self._edit_cmd.currentText(),  16)
            data = int(self._edit_data.currentText(), 16)
        except ValueError:
            self._sys_log("Invalid CMD or DATA — use hex format"); return
        self._send(cmd, data)

    # ──────────────────────────────────────
    def _on_frame(self, cmd: int, value: float):
        self._frame_count += 1
        self._lbl_frames.setText(f"FRAMES  {self._frame_count}")
        self._clock_widget.set_frame_count(self._frame_count)

        # ── Original sensors ────────────────────────────────────────────
        if cmd == CMD_CAN_406:
            speed = (int(value) >> 24) & 0xFF
            self._gauge_speed.set_value(speed)
            self._status.showMessage(f"CAN 0x406 → Speed Motor = {speed} rpm", 1500)

        elif cmd == CMD_CAN_55C:
            pressure = int(value) & 0xFF
            self._gauge_pressure.set_value(pressure)
            self._status.showMessage(f"CAN 0x55C → Pressure = {pressure} bar", 1500)

        elif cmd == CMD_CAN_456:
            temp = int(value) & 0xFF
            self._gauge_temp.set_value(temp)
            self._status.showMessage(f"CAN 0x456 → Temp = {temp} °C", 1500)

        elif cmd == CMD_CAN_679:
            self._abs_notification_active = not self._abs_notification_active
            if self._abs_notification_active:
                self._btn_abs.setStyleSheet(
                    f"QPushButton {{ background:{C_RED}; border:1px solid {C_RED}; "
                    f"border-radius:5px; padding:6px 8px; color:#fff; "
                    f"font-weight:bold; min-height:28px; font-size:11px; }}"
                )
                self._warn_abs.set_active(True)
                self._status.showMessage("CAN 0x679 → ABS ACTIVE", 1500)
            else:
                self._btn_abs.setStyleSheet(
                    f"QPushButton {{ background:{C_CARD}; border:1px solid {C_RED}; "
                    f"border-radius:5px; padding:6px 8px; color:{C_RED}; "
                    f"font-weight:bold; min-height:28px; font-size:11px; }}"
                    f"QPushButton:hover  {{ background:{C_RED}; color:#fff; }}"
                    f"QPushButton:pressed {{ background:{C_RED}; color:#fff; }}"
                    f"QPushButton:disabled {{ background:{C_PANEL}; color:{C_MUTED}; border-color:{C_BORDER}; }}"
                )
                self._warn_abs.set_active(False)
                self._status.showMessage("CAN 0x679 → ABS INACTIVE", 1500)

        elif cmd == CMD_CAN_401:
            self._window_left_notification_active = not self._window_left_notification_active
            if self._window_left_notification_active:
                self._btn_window_left.setStyleSheet(
                    f"QPushButton {{ background:{C_BLUE}; border:1px solid {C_BLUE}; "
                    f"border-radius:5px; padding:6px 8px; color:#fff; "
                    f"font-weight:bold; min-height:28px; font-size:11px; }}"
                )
                self._warn_window_left.set_active(True)
                self._status.showMessage("CAN 0x401 → Window LEFT ACTIVE", 1500)
            else:
                self._btn_window_left.setStyleSheet(
                    f"QPushButton {{ background:{C_CARD}; border:1px solid {C_BLUE}; "
                    f"border-radius:5px; padding:6px 8px; color:{C_BLUE}; "
                    f"font-weight:bold; min-height:28px; font-size:11px; }}"
                    f"QPushButton:hover  {{ background:{C_BLUE}; color:#fff; }}"
                    f"QPushButton:pressed {{ background:{C_BLUE}; color:#fff; }}"
                    f"QPushButton:disabled {{ background:{C_PANEL}; color:{C_MUTED}; border-color:{C_BORDER}; }}"
                )
                self._warn_window_left.set_active(False)
                self._status.showMessage("CAN 0x401 → Window LEFT INACTIVE", 1500)

        # ── New sensors ─────────────────────────────────────────────────
        elif cmd == CMD_CAN_321:
            vitesse = int(value) & 0xFF
            self._gauge_vitesse.set_value(vitesse)
            self._status.showMessage(f"CAN 0x321 → Vitesse = {vitesse} km/h", 1500)

        elif cmd == CMD_CAN_390:
            debit =int(value)  & 0xFF
            self._gauge_debit_air.set_value(debit)
            self._status.showMessage(f"CAN 0x390 → Débit Air = {debit} g/s", 1500)

        elif cmd == CMD_CAN_790:
            # Porte Droite notification — toggle on each reception
            self._porte_droite_notification_active = not self._porte_droite_notification_active
            if self._porte_droite_notification_active:
                self._warn_porte_droite.set_active(True)
                self._status.showMessage("CAN 0x790 → Porte Droite OUVERTE", 1500)
            else:
                self._warn_porte_droite.set_active(False)
                self._status.showMessage("CAN 0x790 → Porte Droite FERMÉE", 1500)

        elif cmd == CMD_CAN_791:
            # Porte Gauche notification — toggle on each reception
            self._porte_gauche_notification_active = not self._porte_gauche_notification_active
            if self._porte_gauche_notification_active:
                self._btn_porte_gauche.setStyleSheet(
                    f"QPushButton {{ background:{C_GREEN}; border:1px solid {C_GREEN}; "
                    f"border-radius:5px; padding:6px 8px; color:#000; "
                    f"font-weight:bold; min-height:28px; font-size:11px; }}"
                )
                self._warn_porte_gauche.set_active(True)
                self._status.showMessage("CAN 0x791 → Porte Gauche OUVERTE", 1500)
            else:
                self._btn_porte_gauche.setStyleSheet(
                    f"QPushButton {{ background:{C_CARD}; border:1px solid {C_GREEN}; "
                    f"border-radius:5px; padding:6px 8px; color:{C_GREEN}; "
                    f"font-weight:bold; min-height:28px; font-size:11px; }}"
                    f"QPushButton:hover  {{ background:{C_GREEN}; color:#000; }}"
                    f"QPushButton:pressed {{ background:{C_GREEN}; color:#000; }}"
                    f"QPushButton:disabled {{ background:{C_PANEL}; color:{C_MUTED}; border-color:{C_BORDER}; }}"
                )
                self._warn_porte_gauche.set_active(False)
                self._status.showMessage("CAN 0x791 → Porte Gauche FERMÉE", 1500)

    # ──────────────────────────────────────
    def _on_raw(self, frame: bytes):
        if not self._chk_log_rx.isChecked(): return
        ts      = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hex_str = " ".join(f"{b:02X}" for b in frame)
        cmd     = frame[1]
        name    = RX_SENSOR_NAMES.get(cmd, f"0x{cmd:02X}")
        self._log.append(
            f'<span style="color:{C_MUTED};">{ts}</span>  '
            f'<span style="color:{C_CYAN};">RX</span>  '
            f'<span style="color:#5AFF8A;">{hex_str}</span>  '
            f'<span style="color:{C_AMBER};">{name}</span>'
        )

    def _tx_log(self, cmd: int, frame: bytes):
        ts      = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hex_str = " ".join(f"{b:02X}" for b in frame)
        self._log.append(
            f'<span style="color:{C_MUTED};">{ts}</span>  '
            f'<span style="color:{C_GREEN};">TX</span>  '
            f'<span style="color:#5AFF8A;">{hex_str}</span>  '
            f'<span style="color:{C_AMBER};">CMD 0x{cmd:02X}</span>'
        )

    def _sys_log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log.append(
            f'<span style="color:{C_MUTED};">{ts}</span>  '
            f'<span style="color:{C_AMBER};">SYS</span>  '
            f'<span style="color:{C_MUTED};">{msg}</span>'
        )

    def _on_serial_error(self, msg: str):
        self._sys_log(f"Serial Link dropped: {msg}")
        self._disconnect()

    def _update_clock(self):
        self._lbl_clock.setText(datetime.now().strftime("%H:%M:%S"))

    def closeEvent(self, event):
        self._disconnect(); event.accept()

    def _update_window_btn(self):
        if self._window_frame_active:
            self._btn_window_frame.setStyleSheet(
                f"QPushButton {{ background:{C_AMBER}; border:1px solid {C_AMBER}; "
                f"border-radius:5px; padding:6px 8px; color:#000; "
                f"font-weight:bold; min-height:28px; font-size:11px; }}"
            )
            self._warn_window_right.set_active(True)
        else:
            self._btn_window_frame.setStyleSheet(
                f"QPushButton {{ background:{C_CARD}; border:1px solid {C_AMBER}; "
                f"border-radius:5px; padding:6px 8px; color:{C_AMBER}; "
                f"font-weight:bold; min-height:28px; font-size:11px; }}"
                f"QPushButton:hover  {{ background:{C_AMBER}; color:#000; }}"
                f"QPushButton:pressed {{ background:{C_AMBER}; color:#000; }}"
                f"QPushButton:disabled {{ background:{C_PANEL}; color:{C_MUTED}; border-color:{C_BORDER}; }}"
            )
            self._warn_window_right.set_active(False)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = STM32Dashboard()
    win.show()
    sys.exit(app.exec_())