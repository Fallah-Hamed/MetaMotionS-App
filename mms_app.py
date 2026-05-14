#!/usr/bin/env python3
"""
MetaMotionS (MMS) High-Level Motion App - v0.1

Focus (current): Speed Measurement tab.
Other tabs are scaffolded placeholders to keep the UI structure stable.
"""

import sys
import csv
import time
import math
import statistics
import threading
from typing import Optional, Tuple, List
from dataclasses import dataclass
from datetime import datetime, timezone, timedelta
from collections import deque
from ctypes import byref, c_uint

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QComboBox,
    QTabWidget,
    QGroupBox,
    QGridLayout,
    QMessageBox,
    QFileDialog,
    QListWidget,
    QListWidgetItem,
    QDialog,
    QProgressBar,
    QMenu,
    QAction,
    QSizePolicy,
    QScrollArea,
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, pyqtSlot, QObject, QPointF
from PyQt5.QtGui import QFont, QPainter, QPen, QColor, QPolygonF

import pyqtgraph as pg

try:
    import numpy as _np  # optional, used for fast FFT if available
except Exception:
    _np = None

try:
    from scipy.signal import find_peaks as _scipy_find_peaks
except Exception:
    _scipy_find_peaks = None

from mbientlab.warble import BleScanner
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import (
    LedPattern,
    LedColor,
    LedPreset,
    FnVoid_VoidP_DataP,
    FnVoid_VoidP_VoidP_Int,
    Const,
    MagBmm150Preset,
    MagBmm150Odr,
    GyroBoschOdr,
    GyroBoschRange,
    AccBoschActivity,
    AccBoschMotion,
    AccBoschDoubleTapWindow,
    SensorOrientation,
    AccBmi270Odr,
    Module,
)

_HKT = timezone(timedelta(hours=8))

FONT_FAMILY = "Arial"
PLOT_SAMPLES = 500
SCAN_TIMEOUT = 3
UI_REFRESH_MS = 200

STYLE_SHEET = """
QMainWindow, QWidget {
    background-color: #1e1e2e; color: #cdd6f4;
    font-family: Arial; font-size: 13px;
}
QTabWidget::pane { border: 1px solid #45475a; border-radius: 6px; }
QTabBar::tab {
    background: #313244; color: #cdd6f4;
    padding: 8px 22px; border-radius: 4px; margin-right: 3px;
    min-width: 140px;
}
QTabBar::tab:selected { background: #89b4fa; color: #1e1e2e; font-weight: bold; }
QPushButton {
    background-color: #313244; color: #cdd6f4;
    border: 1px solid #45475a; border-radius: 6px;
    padding: 7px 16px;
}
QPushButton:hover { background-color: #45475a; }
QPushButton:pressed { background-color: #89b4fa; color: #1e1e2e; }
QPushButton:disabled{ background-color: #181825; color: #585b70; }
QGroupBox {
    border: 1px solid #45475a; border-radius: 8px;
    margin-top: 12px; padding-top: 6px;
    font-weight: bold; color: #89b4fa;
}
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
QLabel { color: #cdd6f4; }
QComboBox {
    background: #313244; color: #cdd6f4;
    border: 1px solid #45475a; border-radius: 5px; padding: 4px 8px;
}
QComboBox QAbstractItemView { background: #313244; color: #cdd6f4; }
QListWidget {
    background: #181825; color: #cdd6f4;
    border: 1px solid #45475a; border-radius: 5px;
}
QListWidget::item:selected { background: #89b4fa; color: #1e1e2e; }
QStatusBar { background: #181825; color: #6c7086; font-size: 12px; }
QProgressBar {
    border: 1px solid #45475a; border-radius: 4px;
    background: #181825; text-align: center;
}
QProgressBar::chunk { background: #89b4fa; border-radius: 3px; }
QMenu { background: #313244; color: #cdd6f4; border: 1px solid #45475a; }
QMenu::item:selected { background: #89b4fa; color: #1e1e2e; }
"""


def _host_ms() -> int:
    return time.time_ns() // 1_000_000


def _perf_ns() -> int:
    return time.perf_counter_ns()


def save_file_dialog(parent, title, default_name):
    dlg = QFileDialog(parent)
    dlg.setWindowTitle(title)
    dlg.setAcceptMode(QFileDialog.AcceptSave)
    dlg.setNameFilter("CSV Files (*.csv)")
    dlg.setDefaultSuffix("csv")
    dlg.selectFile(default_name)
    dlg.setOption(QFileDialog.DontUseNativeDialog, True)
    if dlg.exec_() == QFileDialog.Accepted:
        files = dlg.selectedFiles()
        return files[0] if files else ""
    return ""


def _save_rows(parent, title, default_name, headers, rows):
    if not rows:
        QMessageBox.information(parent, "No Data", "No data collected yet.\nStart measurement first, then save.")
        return
    path = save_file_dialog(parent, title, default_name)
    if not path:
        return
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(headers)
        w.writerows(rows)
    QMessageBox.information(parent, "Saved", f"Saved {len(rows)} samples to:\n{path}")


def _extract_epoch_ms(ptr):
    try:
        return float(ptr.contents.epoch)
    except Exception:
        return None


def _xyz_samples_from_value(value, epoch_ms=None):
    if hasattr(value, "x") and hasattr(value, "y") and hasattr(value, "z"):
        return [{"epoch_ms": epoch_ms, "values": [float(value.x), float(value.y), float(value.z)]}]
    try:
        items = list(value)
    except Exception:
        items = None
    if items and all(hasattr(v, "x") and hasattr(v, "y") and hasattr(v, "z") for v in items):
        return [{"epoch_ms": epoch_ms, "values": [float(v.x), float(v.y), float(v.z)]} for v in items]
    if items and len(items) % 3 == 0 and all(isinstance(v, (int, float)) for v in items):
        out = []
        for i in range(0, len(items), 3):
            out.append({"epoch_ms": epoch_ms, "values": [float(items[i]), float(items[i + 1]), float(items[i + 2])]})
        return out
    return []


class Signals(QObject):
    scan_result = pyqtSignal(list)
    connection_state = pyqtSignal(str)
    error = pyqtSignal(str)


signals = Signals()


class GestureStreamSignals(QObject):
    """Firmware + heuristic gesture events (callbacks may run on BLE thread; signals are thread-safe)."""

    tap = pyqtSignal(str)
    activity = pyqtSignal(str)
    any_motion = pyqtSignal(str)
    step_count = pyqtSignal(int)
    step_pulse = pyqtSignal()
    orientation = pyqtSignal(str)
    heuristic = pyqtSignal(str, str)
    # Module.ACCELEROMETER implementation: 1=BMI160, 4=BMI270 (Mbient constants); -1 = unknown / stopped
    accel_implementation = pyqtSignal(int)


gesture_signals = GestureStreamSignals()


class ScanThread(QThread):
    def __init__(self, timeout=SCAN_TIMEOUT):
        super().__init__()
        self._timeout = timeout

    def run(self):
        devices = {}

        def handler(result):
            try:
                if result.has_service_uuid(MetaWear.GATT_SERVICE):
                    devices[result.mac] = result.name or "MetaWear"
            except Exception:
                pass

        BleScanner.set_handler(handler)
        BleScanner.start()
        time.sleep(self._timeout)
        BleScanner.stop()
        signals.scan_result.emit(list(devices.items()))


class ScanDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Scan for MetaMotionS Devices")
        self.setMinimumSize(520, 380)
        self.selected_mac = None
        self.selected_name = None
        self._build_ui()

    def _build_ui(self):
        lay = QVBoxLayout(self)
        title = QLabel("🔍 Scanning for MetaMotionS Devices…")
        title.setFont(QFont(FONT_FAMILY, 13, QFont.Bold))
        title.setStyleSheet("color:#89b4fa;")
        lay.addWidget(title)
        self.status_lbl = QLabel(f"Scanning for {SCAN_TIMEOUT} seconds…")
        self.status_lbl.setStyleSheet("color:#a6adc8; font-size:11px;")
        lay.addWidget(self.status_lbl)
        self.progress = QProgressBar()
        self.progress.setRange(0, 0)
        lay.addWidget(self.progress)
        self.list_widget = QListWidget()
        self.list_widget.itemDoubleClicked.connect(self.accept)
        lay.addWidget(self.list_widget)

        btn_row = QHBoxLayout()
        self.rescan_btn = QPushButton("🔄 Rescan")
        self.rescan_btn.clicked.connect(self._start_scan)
        self.connect_btn = QPushButton("✅ Connect")
        self.connect_btn.clicked.connect(self.accept)
        self.cancel_btn = QPushButton("✖ Cancel")
        self.cancel_btn.clicked.connect(self.reject)
        btn_row.addWidget(self.rescan_btn)
        btn_row.addStretch()
        btn_row.addWidget(self.connect_btn)
        btn_row.addWidget(self.cancel_btn)
        lay.addLayout(btn_row)

        self._start_scan()

    def _start_scan(self):
        self.list_widget.clear()
        self.progress.setRange(0, 0)
        self.rescan_btn.setEnabled(False)
        self.status_lbl.setText(f"Scanning for {SCAN_TIMEOUT} seconds…")
        self._scanner = ScanThread(SCAN_TIMEOUT)
        signals.scan_result.connect(self._on_result)
        self._scanner.start()

    @pyqtSlot(list)
    def _on_result(self, devices):
        try:
            signals.scan_result.disconnect(self._on_result)
        except Exception:
            pass
        self.progress.setRange(0, 1)
        self.progress.setValue(1)
        self.rescan_btn.setEnabled(True)
        self.list_widget.clear()
        if not devices:
            self.status_lbl.setText("No devices found — tap MMS button and Rescan.")
            self.list_widget.addItem(QListWidgetItem(" No devices found"))
        else:
            self.status_lbl.setText(f"Found {len(devices)} device(s). Select one and Connect.")
            for mac, name in devices:
                item = QListWidgetItem(f" {name} [{mac}]")
                item.setData(Qt.UserRole, mac)
                item.setData(Qt.UserRole + 1, name)
                self.list_widget.addItem(item)
            self.list_widget.setCurrentRow(0)

    def accept(self):
        sel = self.list_widget.currentItem()
        if sel and sel.data(Qt.UserRole):
            self.selected_mac = sel.data(Qt.UserRole)
            self.selected_name = sel.data(Qt.UserRole + 1)
            super().accept()
        else:
            QMessageBox.warning(self, "No device selected", "Please select a device first.")


def make_config_row(pairs):
    w = QWidget()
    grid = QGridLayout(w)
    grid.setContentsMargins(6, 2, 6, 2)
    grid.setHorizontalSpacing(6)
    grid.setVerticalSpacing(4)
    for col, (text, ctrl) in enumerate(pairs):
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        grid.addWidget(lbl, 0, col * 2)
        grid.addWidget(ctrl, 0, col * 2 + 1)
        grid.setColumnStretch(col * 2, 0)
        grid.setColumnStretch(col * 2 + 1, 0)
    grid.setColumnStretch(len(pairs) * 2, 1)
    return w


class RollingPlot(QWidget):
    COLORS = ["#f38ba8", "#a6e3a1", "#89b4fa", "#fab387"]

    def __init__(self, title, ylabel, legend_labels=("X", "Y", "Z"), n=PLOT_SAMPLES):
        super().__init__()
        self.n = n
        self.labels = legend_labels
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.pw = pg.PlotWidget(title=title)
        self.pw.setBackground("#181825")
        self.pw.setLabel("left", ylabel, color="#cdd6f4")
        self.pw.setLabel("bottom", "Sample Number", color="#cdd6f4")
        self.pw.addLegend()
        self.pw.showGrid(x=True, y=True, alpha=0.3)
        self.pw.setContextMenuPolicy(Qt.CustomContextMenu)
        self.pw.customContextMenuRequested.connect(self._ctx_menu)
        self.curves = []
        for i, lbl in enumerate(legend_labels):
            pen = pg.mkPen(color=self.COLORS[i % len(self.COLORS)], width=1.5)
            self.curves.append(self.pw.plot(pen=pen, name=lbl))
        layout.addWidget(self.pw)

    def set_series(self, x_data, *series):
        for i, data in enumerate(series):
            self.curves[i].setData(x_data, data)
        if x_data:
            self.pw.setXRange(x_data[0], x_data[-1], padding=0.0)

    def clear_plot(self):
        for c in self.curves:
            c.setData([], [])

    def _ctx_menu(self, pos):
        menu = QMenu(self)
        act = QAction("🗑 Clear / Reset Plot", self)
        act.triggered.connect(self.clear_plot)
        menu.addAction(act)
        menu.exec_(self.pw.mapToGlobal(pos))


class SpectrumPlot(QWidget):
    COLORS = ["#f38ba8", "#a6e3a1", "#89b4fa", "#fab387"]

    def __init__(self, title, ylabel, legend_labels=("X", "Y", "Z")):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.pw = pg.PlotWidget(title=title)
        self.pw.setBackground("#181825")
        self.pw.setLabel("left", ylabel, color="#cdd6f4")
        self.pw.setLabel("bottom", "Frequency (Hz)", color="#cdd6f4")
        self.pw.addLegend()
        self.pw.showGrid(x=True, y=True, alpha=0.3)
        self.pw.setContextMenuPolicy(Qt.CustomContextMenu)
        self.pw.customContextMenuRequested.connect(self._ctx_menu)
        self.curves = []
        for i, lbl in enumerate(legend_labels):
            pen = pg.mkPen(color=self.COLORS[i % len(self.COLORS)], width=1.5)
            self.curves.append(self.pw.plot(pen=pen, name=lbl))
        layout.addWidget(self.pw)

    def set_spectrum(self, freqs, *mags):
        for i, mag in enumerate(mags):
            self.curves[i].setData(freqs, mag)
        if freqs:
            self.pw.setXRange(freqs[0], freqs[-1], padding=0.0)

    def clear_plot(self):
        for c in self.curves:
            c.setData([], [])

    def _ctx_menu(self, pos):
        menu = QMenu(self)
        act = QAction("🗑 Clear / Reset Plot", self)
        act.triggered.connect(self.clear_plot)
        menu.addAction(act)
        menu.exec_(self.pw.mapToGlobal(pos))


def _hann_window(n: int) -> List[float]:
    if n <= 1:
        return [1.0] * max(n, 0)
    return [0.5 - 0.5 * math.cos(2.0 * math.pi * i / (n - 1)) for i in range(n)]


def _rfft_magnitude(x: List[float], fs_hz: float) -> Tuple[List[float], List[float]]:
    """
    Returns (freqs, mags) for 0..Nyquist using a Hann window and mean removal.
    Magnitude is linear (not dB).
    """
    n = len(x)
    if n < 8 or fs_hz <= 0:
        return [], []

    mean_x = sum(x) / n
    w = _hann_window(n)

    if _np is not None:
        arr = _np.asarray(x, dtype=_np.float64) - float(mean_x)
        arr = arr * _np.asarray(w, dtype=_np.float64)
        fft = _np.fft.rfft(arr)
        mags = _np.abs(fft) / n
        freqs = _np.fft.rfftfreq(n, d=1.0 / fs_hz)
        return freqs.tolist(), mags.tolist()

    # Fallback (no numpy): naive DFT for first half (OK for n=250 and UI refresh ~5 Hz)
    xw = [(x[i] - mean_x) * w[i] for i in range(n)]
    k_max = n // 2
    freqs = [fs_hz * k / n for k in range(k_max + 1)]
    mags = []
    for k in range(k_max + 1):
        re = 0.0
        im = 0.0
        ang = -2.0 * math.pi * k / n
        for i in range(n):
            a = ang * i
            re += xw[i] * math.cos(a)
            im += xw[i] * math.sin(a)
        mags.append(math.sqrt(re * re + im * im) / n)
    return freqs, mags


def _linear_mag_to_db(mags: List[float], floor_db: float = -120.0) -> List[float]:
    """Convert linear FFT magnitudes to 20·log10(|·|), clamped to floor_db."""
    eps = 10.0 ** (floor_db / 20.0)
    out: List[float] = []
    for m in mags:
        v = max(float(m), eps)
        db = 20.0 * math.log10(v)
        out.append(db if db > floor_db else floor_db)
    return out


def _activity_label(code: int) -> str:
    m = {
        AccBoschActivity.STILL: "Still",
        AccBoschActivity.WALKING: "Walking",
        AccBoschActivity.RUNNING: "Running",
        AccBoschActivity.UNKNOWN: "Unknown",
    }
    return m.get(int(code), f"Code {int(code)}")


def _orientation_label(code: int) -> str:
    names = {
        SensorOrientation.FACE_UP_PORTRAIT_UPRIGHT: "Face-up · portrait upright",
        SensorOrientation.FACE_UP_PORTRAIT_UPSIDE_DOWN: "Face-up · portrait upside-down",
        SensorOrientation.FACE_UP_LANDSCAPE_LEFT: "Face-up · landscape left",
        SensorOrientation.FACE_UP_LANDSCAPE_RIGHT: "Face-up · landscape right",
        SensorOrientation.FACE_DOWN_PORTRAIT_UPRIGHT: "Face-down · portrait upright",
        SensorOrientation.FACE_DOWN_PORTRAIT_UPSIDE_DOWN: "Face-down · portrait upside-down",
        SensorOrientation.FACE_DOWN_LANDSCAPE_LEFT: "Face-down · landscape left",
        SensorOrientation.FACE_DOWN_LANDSCAPE_RIGHT: "Face-down · landscape right",
    }
    return names.get(int(code), f"Orientation {int(code)}")


def _tap_label(tap_type: int, sign: int) -> str:
    tmap = {0: "Tap", 1: "Single tap", 2: "Double tap"}
    kind = tmap.get(int(tap_type), f"Tap type {int(tap_type)}")
    return f"{kind} (sign={int(sign)})"


# Mbient MetaWear C API: mbl_mw_metawearboard_get_module_info (accelerometer) -> implementation
_ACC_IMPL_BMI160 = 1
_ACC_IMPL_BMI270 = 4


def _accelerometer_implementation(board) -> Optional[int]:
    try:
        mid = c_uint(Module.ACCELEROMETER)
        inf = libmetawear.mbl_mw_metawearboard_get_module_info(board, byref(mid))
        if inf:
            return int(inf.contents.implementation)
    except Exception:
        pass
    return None


def _bmi270_motion_status_label(u: int) -> str:
    """
    On BMI270 the motion data signal is UINT32 (SDK), typically a feature-interrupt status bitmask.
    Decimal 4 == 0x04 == bit 2, commonly the any-motion line — not an arbitrary 'magic number'.
    """
    v = int(u) & 0xFFFFFFFF
    parts = []
    if v & 0x01:
        parts.append("sig_motion")
    if v & 0x02:
        parts.append("no_motion")
    if v & 0x04:
        parts.append("any_motion")
    rest = v & ~0x07
    if rest:
        parts.append(f"other=0x{rest:x}")
    desc = ", ".join(parts) if parts else "no known flags"
    return f"IRQ status 0x{v:02x} ({desc})"


def _gravity_orientation_text(gx: float, gy: float, gz: float) -> str:
    axa, aya, aza = abs(gx), abs(gy), abs(gz)
    m = max(axa, aya, aza)
    if m < 0.35:
        return "Unclear — hold steady or reduce vibration"
    if aza >= axa and aza >= aya:
        return "Flat · screen/top face up (Z ≈ +1g)" if gz > 0 else "Flat · screen/top face down (Z ≈ −1g)"
    if axa >= aya:
        return "Edge-on · +X up" if gx > 0 else "Edge-on · −X up"
    return "Edge-on · +Y up" if gy > 0 else "Edge-on · −Y up"


def _heading_cardinal(deg: float) -> str:
    names = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    idx = int(((deg % 360.0) + 22.5) // 45.0) % 8
    return names[idx]


class GestureHeuristics:
    """
    Lightweight motion heuristics on raw accel (g): free-fall, impact, shake, stationary,
    coarse cadence / stairs hint. Not a substitute for trained models.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._mag_win = deque(maxlen=125)
        self._vz_buf = deque(maxlen=150)
        self._gyro_mag_win = deque(maxlen=80)
        self._ff_run = 0
        self._last_shake_t = 0.0
        self._last_cadence_t = 0.0
        self._last_stairs_t = 0.0
        self._last_stationary_t = 0.0
        self._last_activity = ""
        self._g_lp = [0.0, 0.0, 0.0]
        self._g_lp_init = False
        self._prev_vec = (0.0, 0.0, 0.0)
        self._have_prev = False
        self._last_tap_t = 0.0
        self._last_orient_t = 0.0
        self._last_orient_text = ""
        self._last_step_t = 0.0
        self._bmi270 = False

    def set_accel_implementation(self, impl: Optional[int]):
        with self._lock:
            self._bmi270 = impl == _ACC_IMPL_BMI270

    def set_activity_hint(self, label: str):
        with self._lock:
            self._last_activity = label or ""

    def _emit_h(self, key: str, text: str):
        gesture_signals.heuristic.emit(key, text)

    def feed_gyro(self, gx_dps: float, gy_dps: float, gz_dps: float, t_perf: float):
        with self._lock:
            gmag = math.sqrt(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps)
            self._gyro_mag_win.append(gmag)

    def feed(self, ax: float, ay: float, az: float, t_perf: float):
        mag = math.sqrt(ax * ax + ay * ay + az * az)
        with self._lock:
            self._mag_win.append(mag)
            # crude vertical high-pass for cadence (body Z ≈ az in device frame)
            prev = self._vz_buf[-1] if self._vz_buf else az
            self._vz_buf.append(az - prev)

            if mag < 0.22:
                self._ff_run += 1
            else:
                if self._ff_run >= 4:
                    self._emit_h("free_fall", f"Low |a|≈{mag:.2f} g (possible free-fall)")
                if self._ff_run >= 4 and mag > 1.8:
                    self._emit_h("impact", f"High |a|≈{mag:.2f} g after low-g phase")
                self._ff_run = 0

            if mag > 2.8:
                self._emit_h("impact", f"Spike |a|≈{mag:.2f} g")

            if len(self._mag_win) >= 25:
                chunk = list(self._mag_win)[-25:]
                sd = statistics.pstdev(chunk) if len(chunk) > 1 else 0.0
                if sd > 0.42 and (t_perf - self._last_shake_t) > 0.35:
                    self._last_shake_t = t_perf
                    self._emit_h("shake", f"High |a| variance (σ≈{sd:.2f})")

            if len(self._mag_win) >= 100:
                chunk = list(self._mag_win)[-100:]
                sd = statistics.pstdev(chunk) if len(chunk) > 1 else 0.0
                if sd < 0.04 and (t_perf - self._last_stationary_t) > 2.0:
                    self._last_stationary_t = t_perf
                    self._emit_h("stationary", "Stable |a| (device likely still)")

            if (t_perf - self._last_cadence_t) > 0.75 and len(self._vz_buf) >= 80:
                self._last_cadence_t = t_perf
                z = list(self._vz_buf)
                mu = sum(z) / len(z)
                zc = [v - mu for v in z]
                fs = 50.0
                peaks = []
                if _scipy_find_peaks is not None and _np is not None:
                    try:
                        arr = _np.asarray(zc, dtype=_np.float64)
                        dist = int(0.28 * fs)
                        pk, _ = _scipy_find_peaks(arr, distance=max(8, dist), prominence=0.08)
                        peaks = list(pk)
                    except Exception:
                        peaks = []
                if not peaks:
                    for i in range(2, len(zc) - 2):
                        if zc[i] > zc[i - 1] and zc[i] >= zc[i + 1] and zc[i] > 0.12:
                            if not peaks or (i - peaks[-1]) > int(0.28 * fs):
                                peaks.append(i)
                if len(peaks) >= 2:
                    dt = (peaks[-1] - peaks[0]) / (len(peaks) - 1) / fs
                    if dt > 0:
                        cadence = 60.0 / dt
                        if 40 < cadence < 200:
                            act = self._last_activity.lower()
                            if "walk" in act or "run" in act or not act:
                                self._emit_h("cadence", f"~{cadence:.0f} steps/min (accel peaks)")

            if (t_perf - self._last_stairs_t) > 2.5 and len(self._mag_win) >= 80:
                self._last_stairs_t = t_perf
                act = self._last_activity.lower()
                if "walk" in act:
                    z = list(self._vz_buf)[-80:]
                    if len(z) > 5:
                        step = max(1, len(z) // 4)
                        jerks = [abs(z[i + step] - z[i]) for i in range(0, len(z) - step, step)]
                        jm = sum(jerks) / len(jerks) if jerks else 0.0
                        if jm > 0.30:
                            self._emit_h("stairs_hint", "Likely stairs / elevation change")
                        elif jm > 0.18:
                            self._emit_h(
                                "stairs_hint",
                                "Possible stairs / ramp / uneven elevation",
                            )

            alpha = 0.08
            if not self._g_lp_init:
                self._g_lp = [ax, ay, az]
                self._g_lp_init = True
            else:
                self._g_lp[0] = (1.0 - alpha) * self._g_lp[0] + alpha * ax
                self._g_lp[1] = (1.0 - alpha) * self._g_lp[1] + alpha * ay
                self._g_lp[2] = (1.0 - alpha) * self._g_lp[2] + alpha * az

            if (t_perf - self._last_orient_t) > 0.20:
                txt = _gravity_orientation_text(self._g_lp[0], self._g_lp[1], self._g_lp[2])
                self._last_orient_t = t_perf
                if txt != self._last_orient_text:
                    self._last_orient_text = txt
                    gesture_signals.orientation.emit(f"{txt} (heuristic)")

            if self._have_prev:
                px, py, pz = self._prev_vec
                dj = math.sqrt((ax - px) ** 2 + (ay - py) ** 2 + (az - pz) ** 2)
                recent_gyro = max(self._gyro_mag_win) if self._gyro_mag_win else 0.0
                if (dj > 0.224 or (dj > 0.144 and recent_gyro > 140.0)) and (t_perf - self._last_tap_t) > 0.18:
                    dt = t_perf - self._last_tap_t
                    self._last_tap_t = t_perf
                    if 0.08 < dt < 0.42:
                        gesture_signals.tap.emit(f"Double tap (heuristic, Δa≈{dj:.2f} g)")
                    else:
                        gesture_signals.tap.emit(f"Single tap (heuristic, Δa≈{dj:.2f} g)")
            self._prev_vec = (ax, ay, az)
            self._have_prev = True

    def reset(self):
        with self._lock:
            self._mag_win.clear()
            self._vz_buf.clear()
            self._gyro_mag_win.clear()
            self._ff_run = 0
            self._g_lp_init = False
            self._have_prev = False
            self._last_orient_text = ""


class DeviceManager:
    def __init__(self):
        self.device = None
        self._cbs = []
        self._signals = {}
        self._active = set()
        self._fast_conn_set = False
        self._gyro_chip = "bmi270"
        self._fast_conn_lock = threading.Lock()
        self._gesture_poll_steps = False

    def connect(self, mac: str):
        signals.connection_state.emit("connecting")
        self._fast_conn_set = False
        for attempt in range(1, 4):
            d = None
            try:
                signals.connection_state.emit(f"connecting (attempt {attempt}/3…)")
                d = MetaWear(mac)
                d.connect()
                self.device = d
                self.device.on_disconnect = lambda status: signals.connection_state.emit("disconnected")
                # Negotiate connection parameters ONCE immediately after connect.
                # Starting multiple sensors in parallel can race this call and cause disconnects.
                self._set_fast_connection()
                signals.connection_state.emit("connected")
                return
            except Exception:
                self.device = None
                try:
                    if d:
                        d.disconnect()
                except Exception:
                    pass
                if attempt < 3:
                    signals.connection_state.emit(f"connecting (retry {attempt}/3…)")
                    time.sleep(5.0)
        signals.error.emit("Failed after 3 attempts.\nToggle Bluetooth OFF/ON, tap MMS button, try again.")
        signals.connection_state.emit("disconnected")

    def disconnect(self):
        self.stop_all()
        self._fast_conn_set = False
        if self.device:
            try:
                libmetawear.mbl_mw_debug_disconnect(self.device.board)
                self.device.disconnect()
            except Exception:
                pass
            self.device = None
        signals.connection_state.emit("disconnected")

    def _unsub(self, key):
        sig = self._signals.pop(key, None)
        if sig is not None and hasattr(libmetawear, "mbl_mw_datasignal_unsubscribe"):
            try:
                libmetawear.mbl_mw_datasignal_unsubscribe(sig)
            except Exception:
                pass

    def _set_fast_connection(self):
        if not self.device:
            return
        with self._fast_conn_lock:
            if self._fast_conn_set:
                return
            fn = getattr(libmetawear, "mbl_mw_settings_set_connection_parameters", None)
            if fn is None:
                self._fast_conn_set = True
                return
            try:
                # Mark as set early to avoid racing threads re-entering while we sleep.
                self._fast_conn_set = True
                fn(self.device.board, 7.5, 7.5, 0, 6000)
                time.sleep(1.0)
            except Exception:
                # If negotiation fails, keep going with default params.
                self._fast_conn_set = True

    def _led_color(self, s):
        return {"Red": LedColor.RED, "Green": LedColor.GREEN, "Blue": LedColor.BLUE}.get(s, LedColor.GREEN)

    def led_blink(self, color, reps):
        if not self.device:
            return
        p = LedPattern(repeat_count=reps + 1)
        libmetawear.mbl_mw_led_load_preset_pattern(byref(p), LedPreset.BLINK)
        libmetawear.mbl_mw_led_write_pattern(self.device.board, byref(p), self._led_color(color))
        libmetawear.mbl_mw_led_play(self.device.board)

    def led_off(self):
        if not self.device:
            return
        libmetawear.mbl_mw_led_stop_and_clear(self.device.board)

    def start_accel_50hz(self, sink, rng_g=2.0):
        if not self.device or "accel" in self._active:
            return
        if "gesture" in self._active:
            self.stop_gesture_detection()
            time.sleep(0.15)
        self._active.add("accel")
        board = self.device.board
        self._set_fast_connection()
        libmetawear.mbl_mw_acc_set_odr(board, 50.0)
        libmetawear.mbl_mw_acc_set_range(board, float(rng_g))
        libmetawear.mbl_mw_acc_write_acceleration_config(board)
        sig = None
        packed = False
        for name in (
            "mbl_mw_acc_get_packed_acceleration_data_signal",
            "mbl_mw_acc_bosch_get_packed_acceleration_data_signal",
            "mbl_mw_acc_get_acceleration_data_signal",
        ):
            fn = getattr(libmetawear, name, None)
            if fn is None:
                continue
            try:
                sig = fn(board)
                packed = "packed" in name
                if sig:
                    break
            except Exception:
                sig = None
        if sig is None:
            self._active.discard("accel")
            return
        self._signals["accel"] = sig

        def cb(ctx, ptr):
            epoch_ms = _extract_epoch_ms(ptr)
            val = parse_value(ptr)
            samples = _xyz_samples_from_value(val, epoch_ms)
            if samples:
                sink(samples=samples, packed=packed, packet_epoch_ms=epoch_ms)

        fn_cb = FnVoid_VoidP_DataP(cb)
        self._cbs.append(fn_cb)
        libmetawear.mbl_mw_datasignal_subscribe(sig, None, fn_cb)
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(board)
        libmetawear.mbl_mw_acc_start(board)

    def stop_accel(self):
        if not self.device or "accel" not in self._active:
            return
        self._active.discard("accel")
        self._unsub("accel")
        try:
            libmetawear.mbl_mw_acc_stop(self.device.board)
            libmetawear.mbl_mw_acc_disable_acceleration_sampling(self.device.board)
        except Exception:
            pass

    def start_gyro_50hz(self, sink, rng_key="2000"):
        if not self.device or "gyro" in self._active:
            return
        if "gesture" in self._active:
            self.stop_gesture_detection()
            time.sleep(0.15)
        self._active.add("gyro")
        board = self.device.board
        self._set_fast_connection()

        odr = GyroBoschOdr._50Hz
        range_map = {
            "125": GyroBoschRange._125dps,
            "250": GyroBoschRange._250dps,
            "500": GyroBoschRange._500dps,
            "1000": GyroBoschRange._1000dps,
            "2000": GyroBoschRange._2000dps,
        }
        rng = range_map.get(str(rng_key), GyroBoschRange._2000dps)

        try:
            libmetawear.mbl_mw_gyro_bmi270_set_odr(board, odr)
            libmetawear.mbl_mw_gyro_bmi270_set_range(board, rng)
            libmetawear.mbl_mw_gyro_bmi270_write_config(board)
            self._gyro_chip = "bmi270"
            sig = None
            for name in (
                "mbl_mw_gyro_bmi270_get_packed_rotation_data_signal",
                "mbl_mw_gyro_bmi270_get_rotation_data_signal",
            ):
                fn = getattr(libmetawear, name, None)
                if fn is None:
                    continue
                try:
                    sig = fn(board)
                    if sig:
                        break
                except Exception:
                    sig = None
            if sig is None:
                raise RuntimeError("No BMI270 gyro signal")
            self._signals["gyro"] = sig

            def cb(ctx, ptr):
                epoch_ms = _extract_epoch_ms(ptr)
                val = parse_value(ptr)
                samples = _xyz_samples_from_value(val, epoch_ms)
                if samples:
                    sink(samples=samples)

            fn_cb = FnVoid_VoidP_DataP(cb)
            self._cbs.append(fn_cb)
            libmetawear.mbl_mw_datasignal_subscribe(sig, None, fn_cb)
            libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(board)
            libmetawear.mbl_mw_gyro_bmi270_start(board)
            return
        except Exception:
            pass

        try:
            libmetawear.mbl_mw_gyro_bmi160_set_odr(board, odr)
            libmetawear.mbl_mw_gyro_bmi160_set_range(board, rng)
            libmetawear.mbl_mw_gyro_bmi160_write_config(board)
            self._gyro_chip = "bmi160"
            sig = None
            for name in (
                "mbl_mw_gyro_bmi160_get_packed_rotation_data_signal",
                "mbl_mw_gyro_bmi160_get_rotation_data_signal",
            ):
                fn = getattr(libmetawear, name, None)
                if fn is None:
                    continue
                try:
                    sig = fn(board)
                    if sig:
                        break
                except Exception:
                    sig = None
            if sig is None:
                raise RuntimeError("No BMI160 gyro signal")
            self._signals["gyro"] = sig

            def cb(ctx, ptr):
                epoch_ms = _extract_epoch_ms(ptr)
                val = parse_value(ptr)
                samples = _xyz_samples_from_value(val, epoch_ms)
                if samples:
                    sink(samples=samples)

            fn_cb = FnVoid_VoidP_DataP(cb)
            self._cbs.append(fn_cb)
            libmetawear.mbl_mw_datasignal_subscribe(sig, None, fn_cb)
            libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(board)
            libmetawear.mbl_mw_gyro_bmi160_start(board)
        except Exception:
            self._active.discard("gyro")

    def stop_gyro(self):
        if not self.device or "gyro" not in self._active:
            return
        self._active.discard("gyro")
        self._unsub("gyro")
        board = self.device.board
        try:
            if getattr(self, "_gyro_chip", "bmi270") == "bmi270":
                libmetawear.mbl_mw_gyro_bmi270_stop(board)
                libmetawear.mbl_mw_gyro_bmi270_disable_rotation_sampling(board)
            else:
                libmetawear.mbl_mw_gyro_bmi160_stop(board)
                libmetawear.mbl_mw_gyro_bmi160_disable_rotation_sampling(board)
        except Exception:
            pass

    def start_mag_20hz(self, sink):
        if not self.device or "mag" in self._active:
            return
        if "gesture" in self._active:
            self.stop_gesture_detection()
            time.sleep(0.15)
        self._active.add("mag")
        board = self.device.board
        try:
            libmetawear.mbl_mw_mag_bmm150_set_preset(board, MagBmm150Preset.HIGH_ACCURACY)
            # Per Mbient/BMM150 docs, HIGH_ACCURACY corresponds to 20 Hz with xy/z repetitions 47/83.
            if hasattr(libmetawear, "mbl_mw_mag_bmm150_configure"):
                libmetawear.mbl_mw_mag_bmm150_configure(board, 47, 83, MagBmm150Odr._20Hz)
        except Exception:
            pass
        sig = libmetawear.mbl_mw_mag_bmm150_get_b_field_data_signal(board)
        self._signals["mag"] = sig

        def cb(ctx, ptr):
            epoch_ms = _extract_epoch_ms(ptr)
            val = parse_value(ptr)
            samples = _xyz_samples_from_value(val, epoch_ms)
            if samples:
                sink(samples=samples)

        fn_cb = FnVoid_VoidP_DataP(cb)
        self._cbs.append(fn_cb)
        libmetawear.mbl_mw_datasignal_subscribe(sig, None, fn_cb)
        libmetawear.mbl_mw_mag_bmm150_enable_b_field_sampling(board)
        libmetawear.mbl_mw_mag_bmm150_start(board)

    def stop_mag(self):
        if not self.device or "mag" not in self._active:
            return
        self._active.discard("mag")
        self._unsub("mag")
        try:
            libmetawear.mbl_mw_mag_bmm150_stop(self.device.board)
            libmetawear.mbl_mw_mag_bmm150_disable_b_field_sampling(self.device.board)
        except Exception:
            pass

    def reset_step_counter_hw(self):
        if not self.device:
            return
        board = self.device.board
        for nm in ("mbl_mw_acc_bmi270_reset_step_counter", "mbl_mw_acc_bmi160_reset_step_counter"):
            fn = getattr(libmetawear, nm, None)
            if fn:
                try:
                    fn(board)
                    return
                except Exception:
                    pass

    def stop_gesture_detection(self):
        if "gesture" not in self._active:
            return
        board = self.device.board if self.device else None
        if board:
            for fn_name, args in (
                ("mbl_mw_acc_bosch_disable_tap_detection", (board,)),
                ("mbl_mw_acc_bosch_disable_orientation_detection", (board,)),
                ("mbl_mw_acc_bosch_disable_motion_detection", (board, AccBoschMotion.ANYMOTION)),
                ("mbl_mw_acc_bmi270_disable_activity_detection", (board,)),
                ("mbl_mw_acc_bmi270_disable_step_counter", (board,)),
                ("mbl_mw_acc_bmi270_disable_step_detector", (board,)),
                ("mbl_mw_acc_bmi160_disable_step_counter", (board,)),
                ("mbl_mw_acc_bmi160_disable_step_detector", (board,)),
            ):
                fn = getattr(libmetawear, fn_name, None)
                if fn:
                    try:
                        fn(*args)
                    except Exception:
                        pass
        for key in (
            "gesture_tap",
            "gesture_activity",
            "gesture_motion",
            "gesture_step_cnt",
            "gesture_step_det",
            "gesture_orient",
            "gesture_gyro",
            "accel",
        ):
            self._unsub(key)
        self._active.discard("gesture")
        self._active.discard("accel")
        if board:
            try:
                libmetawear.mbl_mw_acc_stop(board)
                libmetawear.mbl_mw_acc_disable_acceleration_sampling(board)
            except Exception:
                pass
            try:
                libmetawear.mbl_mw_gyro_bmi270_stop(board)
                libmetawear.mbl_mw_gyro_bmi270_disable_rotation_sampling(board)
            except Exception:
                pass
            try:
                libmetawear.mbl_mw_gyro_bmi160_stop(board)
                libmetawear.mbl_mw_gyro_bmi160_disable_rotation_sampling(board)
            except Exception:
                pass
        self._gesture_poll_steps = False
        try:
            gesture_signals.accel_implementation.emit(-1)
        except Exception:
            pass

    def poll_step_counter_read(self):
        """BMI270 step count must be read via this API; streaming the step-counter signal is unsupported."""
        if not self.device or "gesture" not in self._active or not getattr(self, "_gesture_poll_steps", False):
            return
        board = self.device.board
        fn = getattr(libmetawear, "mbl_mw_acc_bmi270_read_step_counter", None)
        if not fn:
            return

        def handler(ctx, ptr, value):
            try:
                gesture_signals.step_count.emit(int(value))
            except Exception:
                pass

        cb = FnVoid_VoidP_VoidP_Int(handler)
        self._cbs.append(cb)
        try:
            fn(board, None, cb)
        except Exception:
            pass

    def start_gesture_detection(self, accel_sink, gyro_sink=None, rng_g=8.0):
        """
        Enables Bosch/BMI270 features where the hardware supports them, plus 50 Hz accel heuristics.
        MetaMotion S (BMI270): Bosch tap + smartphone orientation are not supported by the chip (Mbient docs);
        we use heuristics for those and poll the step counter.
        """
        if not self.device or "gesture" in self._active:
            return
        self.stop_gyro()
        self.stop_mag()
        self.stop_accel()
        time.sleep(0.2)
        self._active.add("gesture")
        board = self.device.board
        self._set_fast_connection()

        acc_impl = _accelerometer_implementation(board)
        if acc_impl is None and self.device and getattr(self.device, "info", None):
            hw = str(self.device.info.get("hardware", "")).upper()
            if "270" in hw:
                acc_impl = _ACC_IMPL_BMI270
            elif "160" in hw:
                acc_impl = _ACC_IMPL_BMI160
        is_bmi270 = acc_impl == _ACC_IMPL_BMI270
        self._gesture_poll_steps = bool(is_bmi270)

        libmetawear.mbl_mw_acc_set_odr(board, 50.0)
        libmetawear.mbl_mw_acc_set_range(board, float(rng_g))
        libmetawear.mbl_mw_acc_write_acceleration_config(board)

        def _safe(name, call):
            fn = getattr(libmetawear, name, None)
            if not fn:
                return
            try:
                call(fn)
            except Exception:
                pass

        if is_bmi270:
            _safe("mbl_mw_acc_bmi270_set_odr", lambda f: f(board, AccBmi270Odr._50Hz))

        if not is_bmi270:
            _safe("mbl_mw_acc_bosch_set_double_tap_window", lambda f: f(board, AccBoschDoubleTapWindow._250ms))
            _safe("mbl_mw_acc_bosch_write_tap_config", lambda f: f(board))
            _safe("mbl_mw_acc_bosch_enable_tap_detection", lambda f: f(board, 1, 1))
            _safe("mbl_mw_acc_bosch_write_orientation_config", lambda f: f(board))
            _safe("mbl_mw_acc_bosch_enable_orientation_detection", lambda f: f(board))

        _safe("mbl_mw_acc_bmi270_write_step_counter_config", lambda f: f(board))
        _safe("mbl_mw_acc_bmi270_enable_step_counter", lambda f: f(board))
        _safe("mbl_mw_acc_bmi270_enable_step_detector", lambda f: f(board))
        _safe("mbl_mw_acc_bmi270_enable_activity_detection", lambda f: f(board))

        def _get_sig(name):
            fn = getattr(libmetawear, name, None)
            if not fn:
                return None
            try:
                return fn(board)
            except Exception:
                return None

        tap_sig = None if is_bmi270 else _get_sig("mbl_mw_acc_bosch_get_tap_data_signal")
        act_sig = _get_sig("mbl_mw_acc_bmi270_get_activity_detector_data_signal")
        sc_sig = None
        if not is_bmi270:
            sc_sig = _get_sig("mbl_mw_acc_bmi270_get_step_counter_data_signal")
            if sc_sig is None:
                _safe("mbl_mw_acc_bmi160_write_step_counter_config", lambda f: f(board))
                _safe("mbl_mw_acc_bmi160_enable_step_counter", lambda f: f(board))
                sc_sig = _get_sig("mbl_mw_acc_bmi160_get_step_counter_data_signal")
        sd_sig = _get_sig("mbl_mw_acc_bmi270_get_step_detector_data_signal")
        ori_sig = None if is_bmi270 else _get_sig("mbl_mw_acc_bosch_get_orientation_detection_data_signal")

        def tap_cb(ctx, ptr):
            try:
                v = parse_value(ptr)
                if hasattr(v, "type") and hasattr(v, "sign"):
                    gesture_signals.tap.emit(_tap_label(v.type, v.sign))
                else:
                    gesture_signals.tap.emit(str(v))
            except Exception as e:
                gesture_signals.tap.emit(f"(parse error: {e})")

        def activity_cb(ctx, ptr):
            try:
                v = parse_value(ptr)
                gesture_signals.activity.emit(_activity_label(int(v)))
            except Exception as e:
                gesture_signals.activity.emit(f"(error: {e})")

        def step_cnt_cb(ctx, ptr):
            try:
                v = parse_value(ptr)
                gesture_signals.step_count.emit(int(v))
            except Exception:
                pass

        def step_det_cb(ctx, ptr):
            gesture_signals.step_pulse.emit()

        def orient_cb(ctx, ptr):
            try:
                v = parse_value(ptr)
                gesture_signals.orientation.emit(_orientation_label(int(v)))
            except Exception as e:
                gesture_signals.orientation.emit(f"(error: {e})")

        if tap_sig:
            self._signals["gesture_tap"] = tap_sig
            fn_t = FnVoid_VoidP_DataP(tap_cb)
            self._cbs.append(fn_t)
            libmetawear.mbl_mw_datasignal_subscribe(tap_sig, None, fn_t)
        if act_sig:
            self._signals["gesture_activity"] = act_sig
            fn_a = FnVoid_VoidP_DataP(activity_cb)
            self._cbs.append(fn_a)
            libmetawear.mbl_mw_datasignal_subscribe(act_sig, None, fn_a)
        if sc_sig:
            self._signals["gesture_step_cnt"] = sc_sig
            fn_s = FnVoid_VoidP_DataP(step_cnt_cb)
            self._cbs.append(fn_s)
            libmetawear.mbl_mw_datasignal_subscribe(sc_sig, None, fn_s)
        if sd_sig:
            self._signals["gesture_step_det"] = sd_sig
            fn_d = FnVoid_VoidP_DataP(step_det_cb)
            self._cbs.append(fn_d)
            libmetawear.mbl_mw_datasignal_subscribe(sd_sig, None, fn_d)
        if ori_sig:
            self._signals["gesture_orient"] = ori_sig
            fn_o = FnVoid_VoidP_DataP(orient_cb)
            self._cbs.append(fn_o)
            libmetawear.mbl_mw_datasignal_subscribe(ori_sig, None, fn_o)

        if gyro_sink is not None:
            odr = GyroBoschOdr._50Hz
            rng = GyroBoschRange._500dps
            try:
                libmetawear.mbl_mw_gyro_bmi270_set_odr(board, odr)
                libmetawear.mbl_mw_gyro_bmi270_set_range(board, rng)
                libmetawear.mbl_mw_gyro_bmi270_write_config(board)
                self._gyro_chip = "bmi270"
                gsig = None
                for name in (
                    "mbl_mw_gyro_bmi270_get_packed_rotation_data_signal",
                    "mbl_mw_gyro_bmi270_get_rotation_data_signal",
                ):
                    fn = getattr(libmetawear, name, None)
                    if fn is None:
                        continue
                    try:
                        gsig = fn(board)
                        if gsig:
                            break
                    except Exception:
                        gsig = None
                if gsig:
                    self._signals["gesture_gyro"] = gsig

                    def gyro_cb(ctx, ptr):
                        epoch_ms = _extract_epoch_ms(ptr)
                        val = parse_value(ptr)
                        samples = _xyz_samples_from_value(val, epoch_ms)
                        if samples:
                            gyro_sink(samples=samples)

                    fn_g = FnVoid_VoidP_DataP(gyro_cb)
                    self._cbs.append(fn_g)
                    libmetawear.mbl_mw_datasignal_subscribe(gsig, None, fn_g)
                    libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(board)
                    libmetawear.mbl_mw_gyro_bmi270_start(board)
            except Exception:
                try:
                    libmetawear.mbl_mw_gyro_bmi160_set_odr(board, odr)
                    libmetawear.mbl_mw_gyro_bmi160_set_range(board, rng)
                    libmetawear.mbl_mw_gyro_bmi160_write_config(board)
                    self._gyro_chip = "bmi160"
                    gsig = None
                    for name in (
                        "mbl_mw_gyro_bmi160_get_packed_rotation_data_signal",
                        "mbl_mw_gyro_bmi160_get_rotation_data_signal",
                    ):
                        fn = getattr(libmetawear, name, None)
                        if fn is None:
                            continue
                        try:
                            gsig = fn(board)
                            if gsig:
                                break
                        except Exception:
                            gsig = None
                    if gsig:
                        self._signals["gesture_gyro"] = gsig

                        def gyro_cb(ctx, ptr):
                            epoch_ms = _extract_epoch_ms(ptr)
                            val = parse_value(ptr)
                            samples = _xyz_samples_from_value(val, epoch_ms)
                            if samples:
                                gyro_sink(samples=samples)

                        fn_g = FnVoid_VoidP_DataP(gyro_cb)
                        self._cbs.append(fn_g)
                        libmetawear.mbl_mw_datasignal_subscribe(gsig, None, fn_g)
                        libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(board)
                        libmetawear.mbl_mw_gyro_bmi160_start(board)
                except Exception:
                    pass

        sig = None
        packed = False
        for name in (
            "mbl_mw_acc_get_packed_acceleration_data_signal",
            "mbl_mw_acc_bosch_get_packed_acceleration_data_signal",
            "mbl_mw_acc_get_acceleration_data_signal",
        ):
            fn = getattr(libmetawear, name, None)
            if fn is None:
                continue
            try:
                sig = fn(board)
                packed = "packed" in name
                if sig:
                    break
            except Exception:
                sig = None
        if sig is None:
            self.stop_gesture_detection()
            return
        self._signals["accel"] = sig
        self._active.add("accel")

        def accel_cb(ctx, ptr):
            epoch_ms = _extract_epoch_ms(ptr)
            val = parse_value(ptr)
            samples = _xyz_samples_from_value(val, epoch_ms)
            if samples:
                accel_sink(samples=samples, packed=packed, packet_epoch_ms=epoch_ms)

        fn_acc = FnVoid_VoidP_DataP(accel_cb)
        self._cbs.append(fn_acc)
        libmetawear.mbl_mw_datasignal_subscribe(sig, None, fn_acc)
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(board)
        libmetawear.mbl_mw_acc_start(board)

        try:
            gesture_signals.accel_implementation.emit(acc_impl if acc_impl is not None else -1)
        except Exception:
            pass
    def stop_all(self):
        if "gesture" in self._active:
            self.stop_gesture_detection()
        for s in list(self._active):
            if s == "accel":
                self.stop_accel()
            elif s == "gyro":
                self.stop_gyro()
            elif s == "mag":
                self.stop_mag()


device_mgr = DeviceManager()


def _inv_sqrt(x: float) -> float:
    return 1.0 / math.sqrt(x) if x > 0 else 0.0


def _quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def _quat_norm(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n == 0:
        return (1.0, 0.0, 0.0, 0.0)
    inv = 1.0 / n
    return (w * inv, x * inv, y * inv, z * inv)


def _rotate_vector_by_quat(v, q):
    # v' = q * (0,v) * q_conj
    vx, vy, vz = v
    qq = _quat_mul(_quat_mul(q, (0.0, vx, vy, vz)), _quat_conj(q))
    return (qq[1], qq[2], qq[3])


@dataclass
class SpeedSample:
    sample_index: int
    pc_ms: int
    vx: float
    vy: float
    vz: float


class SpeedRecorder:
    def __init__(self, plot_samples=PLOT_SAMPLES):
        self.plot_samples = plot_samples
        self._lock = threading.Lock()
        self.clear()

    def clear(self):
        with self._lock:
            self.rows: list[SpeedSample] = []
            self._plot_buffers = [deque(maxlen=self.plot_samples) for _ in range(3)]
            self._latest = (None, None, None)
            self._sample_count = 0
            self._packet_rate_hz = 0.0
            self._last_perf_ns = None

    def append(self, vx, vy, vz):
        pc_ms = _host_ms()
        perf_ns = _perf_ns()
        with self._lock:
            if self._last_perf_ns is not None and perf_ns > self._last_perf_ns:
                self._packet_rate_hz = 1e9 / (perf_ns - self._last_perf_ns)
            self._last_perf_ns = perf_ns
            self._sample_count += 1
            s = SpeedSample(self._sample_count, pc_ms, float(vx), float(vy), float(vz))
            self.rows.append(s)
            self._latest = (s.vx, s.vy, s.vz)
            self._plot_buffers[0].append(s.vx)
            self._plot_buffers[1].append(s.vy)
            self._plot_buffers[2].append(s.vz)

    def snapshot(self):
        with self._lock:
            points = len(self._plot_buffers[0])
            start = self._sample_count - points + 1 if points else 1
            plot_x = list(range(start, self._sample_count + 1)) if points else []
            return {
                "latest": self._latest,
                "plot_x": plot_x,
                "plot_data": [list(b) for b in self._plot_buffers],
                "rate_hz": self._packet_rate_hz,
                "samples": self._sample_count,
            }

    def csv_rows(self):
        with self._lock:
            return [[r.sample_index, r.pc_ms, r.vx, r.vy, r.vz, math.sqrt(r.vx * r.vx + r.vy * r.vy + r.vz * r.vz)] for r in self.rows]


class MadgwickAHRS:
    """
    Minimal Madgwick AHRS (IMU+MAG).

    Notes:
    - Gyro in rad/s.
    - Accel in m/s^2 (or any unit; it is normalized internally).
    - Mag in uT (or any unit; it is normalized internally).
    """

    def __init__(self, beta=0.08):
        self.beta = float(beta)
        self.q = (1.0, 0.0, 0.0, 0.0)  # w,x,y,z

    def update(self, gx, gy, gz, ax, ay, az, mx=None, my=None, mz=None, dt=0.02):
        q1, q2, q3, q4 = self.q

        # Rate of change of quaternion from gyroscope
        q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz)
        q_dot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy)
        q_dot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx)
        q_dot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx)

        # Normalize accelerometer
        norm_a = ax * ax + ay * ay + az * az
        if norm_a > 0.0:
            inv_norm_a = _inv_sqrt(norm_a)
            ax *= inv_norm_a
            ay *= inv_norm_a
            az *= inv_norm_a

            if mx is None or my is None or mz is None:
                # IMU-only correction (no mag)
                # Objective function f(q) for gravity direction
                f1 = 2.0 * (q2 * q4 - q1 * q3) - ax
                f2 = 2.0 * (q1 * q2 + q3 * q4) - ay
                f3 = 2.0 * (0.5 - q2 * q2 - q3 * q3) - az

                # Jacobian transpose times f (gradient)
                s1 = -2.0 * q3 * f1 + 2.0 * q2 * f2
                s2 = 2.0 * q4 * f1 + 2.0 * q1 * f2 - 4.0 * q2 * f3
                s3 = -2.0 * q1 * f1 + 2.0 * q4 * f2 - 4.0 * q3 * f3
                s4 = 2.0 * q2 * f1 + 2.0 * q3 * f2
            else:
                # Normalize magnetometer
                norm_m = mx * mx + my * my + mz * mz
                if norm_m > 0.0:
                    inv_norm_m = _inv_sqrt(norm_m)
                    mx *= inv_norm_m
                    my *= inv_norm_m
                    mz *= inv_norm_m
                else:
                    mx = my = mz = None

                if mx is None:
                    s1 = s2 = s3 = s4 = 0.0
                else:
                    # Reference direction of Earth's magnetic field
                    # h = q * m * q_conj
                    h = _quat_mul(_quat_mul((q1, q2, q3, q4), (0.0, mx, my, mz)), _quat_conj((q1, q2, q3, q4)))
                    hx, hy, hz = h[1], h[2], h[3]
                    bx = math.sqrt(hx * hx + hy * hy)
                    bz = hz

                    # Objective function and Jacobian (Madgwick 2010)
                    f1 = 2.0 * (q2 * q4 - q1 * q3) - ax
                    f2 = 2.0 * (q1 * q2 + q3 * q4) - ay
                    f3 = 2.0 * (0.5 - q2 * q2 - q3 * q3) - az
                    f4 = 2.0 * bx * (0.5 - q3 * q3 - q4 * q4) + 2.0 * bz * (q2 * q4 - q1 * q3) - mx
                    f5 = 2.0 * bx * (q2 * q3 - q1 * q4) + 2.0 * bz * (q1 * q2 + q3 * q4) - my
                    f6 = 2.0 * bx * (q1 * q3 + q2 * q4) + 2.0 * bz * (0.5 - q2 * q2 - q3 * q3) - mz

                    # Gradient (pre-computed partials)
                    s1 = (
                        -2.0 * q3 * f1
                        + 2.0 * q2 * f2
                        - 2.0 * bz * q3 * f4
                        + (-2.0 * bx * q4 + 2.0 * bz * q2) * f5
                        + 2.0 * bx * q3 * f6
                    )
                    s2 = (
                        2.0 * q4 * f1
                        + 2.0 * q1 * f2
                        - 4.0 * q2 * f3
                        + 2.0 * bz * q4 * f4
                        + (2.0 * bx * q3 + 2.0 * bz * q1) * f5
                        + (2.0 * bx * q4 - 4.0 * bz * q2) * f6
                    )
                    s3 = (
                        -2.0 * q1 * f1
                        + 2.0 * q4 * f2
                        - 4.0 * q3 * f3
                        + (-4.0 * bx * q3 - 2.0 * bz * q1) * f4
                        + (2.0 * bx * q2 + 2.0 * bz * q4) * f5
                        + (2.0 * bx * q1 - 4.0 * bz * q3) * f6
                    )
                    s4 = (
                        2.0 * q2 * f1
                        + 2.0 * q3 * f2
                        + (-4.0 * bx * q4 + 2.0 * bz * q2) * f4
                        + (-2.0 * bx * q1 + 2.0 * bz * q3) * f5
                        + 2.0 * bx * q2 * f6
                    )

            norm_s = s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4
            if norm_s > 0.0:
                inv_norm_s = _inv_sqrt(norm_s)
                s1 *= inv_norm_s
                s2 *= inv_norm_s
                s3 *= inv_norm_s
                s4 *= inv_norm_s

                q_dot1 -= self.beta * s1
                q_dot2 -= self.beta * s2
                q_dot3 -= self.beta * s3
                q_dot4 -= self.beta * s4

        q1 += q_dot1 * dt
        q2 += q_dot2 * dt
        q3 += q_dot3 * dt
        q4 += q_dot4 * dt
        self.q = _quat_norm((q1, q2, q3, q4))


class SpeedEstimator:
    """
    Practical speed estimator for demo/measurement UI:
    - Uses Madgwick to estimate orientation (gyro + accel + mag).
    - Rotates accel into Earth frame, subtracts gravity.
    - Integrates to velocity with fixed dt = 1/50.
    - Warmup: no integration for N seconds; AHRS settles; then bias/zero if stationary.
    - ZUPT when gyro + accel + magnetometer-stability all indicate stationary.

    Note: Constant translational speed implies ~0 linear acceleration; integrated velocity
    stays flat between acceleration impulses — magnetometer cannot measure speed.

    Post-warmup settle: after warmup, fusion continues for a short window with v held at 0
    so attitude/bias can finish converging without a false velocity step.
    """

    # Local gravity for Hong Kong used for unit conversion and gravity subtraction (m/s^2).
    G = 9.78778
    # Extra time after warmup with no velocity integration (reduces post-warmup spike).
    POST_WARMUP_SETTLE_SEC = 0.5
    # Use last fraction of warmup samples for bias stats (early samples are noisier).
    WARMUP_TAIL_FRAC = 0.4

    def __init__(self, sample_hz=50.0, warmup_sec=2.0):
        self.sample_hz = float(sample_hz)
        self.dt = 1.0 / self.sample_hz
        self.warmup_sec = float(warmup_sec)
        self.ahrs = MadgwickAHRS(beta=0.08)
        self._latest_gyro_dps = (0.0, 0.0, 0.0)
        self._latest_mag = (None, None, None)
        self._mag_unit_prev = None
        self._mag_angle_deg_window = deque(maxlen=6)
        self.v = [0.0, 0.0, 0.0]  # m/s in Earth frame
        self._acc_bias = [0.0, 0.0, 0.0]  # m/s^2 bias in sensor frame (slowly updated)
        self._stationary_count = 0
        self._warmup_samples_left = 0
        self._warmup_gyro_norms = []
        self._warmup_acc_g = []  # (ax_g, ay_g, az_g) per accel sample during warmup
        self._settle_samples_left = 0
        self.phase = "idle"  # idle | warmup | settle | run

    @property
    def warmup_samples_remaining(self) -> int:
        return self._warmup_samples_left

    @property
    def settle_samples_remaining(self) -> int:
        return self._settle_samples_left

    def reset(self, warmup_sec=None):
        ws = self.warmup_sec if warmup_sec is None else float(warmup_sec)
        self.warmup_sec = ws
        self.ahrs = MadgwickAHRS(beta=0.08)
        self._latest_gyro_dps = (0.0, 0.0, 0.0)
        self._latest_mag = (None, None, None)
        self._mag_unit_prev = None
        self._mag_angle_deg_window.clear()
        self.v = [0.0, 0.0, 0.0]
        self._acc_bias = [0.0, 0.0, 0.0]
        self._stationary_count = 0
        self._warmup_samples_left = int(round(ws * self.sample_hz))
        self._warmup_gyro_norms = []
        self._warmup_acc_g = []
        self._settle_samples_left = 0
        self.phase = "warmup" if self._warmup_samples_left > 0 else "run"

    def update_gyro(self, gx_dps, gy_dps, gz_dps):
        self._latest_gyro_dps = (float(gx_dps), float(gy_dps), float(gz_dps))

    def update_mag(self, mx, my, mz):
        mx, my, mz = float(mx), float(my), float(mz)
        self._latest_mag = (mx, my, mz)
        n = math.sqrt(mx * mx + my * my + mz * mz)
        if n <= 1e-9:
            return
        ux, uy, uz = mx / n, my / n, mz / n
        if self._mag_unit_prev is not None:
            px, py, pz = self._mag_unit_prev
            c = max(-1.0, min(1.0, px * ux + py * uy + pz * uz))
            ang = math.degrees(math.acos(c))
            self._mag_angle_deg_window.append(ang)
        self._mag_unit_prev = (ux, uy, uz)

    def _mag_stable_for_zupt(self) -> bool:
        if not self._mag_angle_deg_window:
            return True
        return max(self._mag_angle_deg_window) < 6.0

    def _finalize_warmup(self):
        """After warmup: if device was still, lock bias and v=0; else start integration with v=0."""
        if len(self._warmup_gyro_norms) < 3 or len(self._warmup_acc_g) < 3:
            self._settle_samples_left = int(round(self.POST_WARMUP_SETTLE_SEC * self.sample_hz))
            self.phase = "settle" if self._settle_samples_left > 0 else "run"
            return
        n = len(self._warmup_acc_g)
        i0 = max(0, int((1.0 - self.WARMUP_TAIL_FRAC) * n))
        tail_acc = self._warmup_acc_g[i0:]
        tail_gyro = self._warmup_gyro_norms[i0:]

        mean_gyro = statistics.mean(tail_gyro)
        acc_norms = [
            math.sqrt(ax * ax + ay * ay + az * az) * self.G for ax, ay, az in tail_acc
        ]
        st_acc = statistics.stdev(acc_norms) if len(acc_norms) > 1 else 0.0
        stationary_like = (mean_gyro < 10.0) and (st_acc < 0.15 * self.G)

        if stationary_like:
            # Mean accel in m/s^2 (uncorrected) over tail of warmup (matches final attitude better)
            mx = statistics.mean([t[0] for t in tail_acc]) * self.G
            my = statistics.mean([t[1] for t in tail_acc]) * self.G
            mz = statistics.mean([t[2] for t in tail_acc]) * self.G
            gravity_s = _rotate_vector_by_quat((0.0, 0.0, self.G), _quat_conj(self.ahrs.q))
            self._acc_bias[0] = mx - gravity_s[0]
            self._acc_bias[1] = my - gravity_s[1]
            self._acc_bias[2] = mz - gravity_s[2]
            self.v = [0.0, 0.0, 0.0]
        else:
            # Already moving during warmup — keep bias near zero; user sees motion without startup spike.
            self._acc_bias = [0.0, 0.0, 0.0]
            self.v = [0.0, 0.0, 0.0]

        self._settle_samples_left = int(round(self.POST_WARMUP_SETTLE_SEC * self.sample_hz))
        self.phase = "settle" if self._settle_samples_left > 0 else "run"

    def update_accel_and_step(self, ax_g, ay_g, az_g):
        # Convert accel to m/s^2 (sensor frame)
        ax = float(ax_g) * self.G
        ay = float(ay_g) * self.G
        az = float(az_g) * self.G

        # Gyro in rad/s
        gx_dps, gy_dps, gz_dps = self._latest_gyro_dps
        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)

        mx, my, mz = self._latest_mag
        self.ahrs.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt=self.dt)

        # Warmup: fuse attitude only; do not integrate velocity; record stats for bias decision.
        if self._warmup_samples_left > 0:
            gyro_norm = math.sqrt(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps)
            self._warmup_gyro_norms.append(gyro_norm)
            self._warmup_acc_g.append((float(ax_g), float(ay_g), float(az_g)))
            self._warmup_samples_left -= 1
            if self._warmup_samples_left == 0:
                self._finalize_warmup()
            return (0.0, 0.0, 0.0)

        # Bias correction (sensor frame) after warmup
        ax -= self._acc_bias[0]
        ay -= self._acc_bias[1]
        az -= self._acc_bias[2]

        # Post-warmup settle: keep reporting v=0 while refining bias (avoids transient right after warmup)
        if self._settle_samples_left > 0:
            gravity_s = _rotate_vector_by_quat((0.0, 0.0, self.G), _quat_conj(self.ahrs.q))
            ex = ax - gravity_s[0]
            ey = ay - gravity_s[1]
            ez = az - gravity_s[2]
            beta = 0.02
            self._acc_bias[0] += beta * ex
            self._acc_bias[1] += beta * ey
            self._acc_bias[2] += beta * ez
            self._settle_samples_left -= 1
            if self._settle_samples_left == 0:
                self.v = [0.0, 0.0, 0.0]
                self.phase = "run"
            return (0.0, 0.0, 0.0)

        # Rotate accel to Earth frame and subtract gravity
        ax_e, ay_e, az_e = _rotate_vector_by_quat((ax, ay, az), self.ahrs.q)
        ax_lin = ax_e
        ay_lin = ay_e
        az_lin = az_e - self.G

        # Stationary detection: accel ~1g, low gyro, stable mag direction (rotation would swing mag in sensor frame)
        acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
        gyro_norm_dps = math.sqrt(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps)
        mag_ok = self._mag_stable_for_zupt()
        is_stationary = (
            # Slightly wider thresholds so ZUPT engages more often.
            (abs(acc_norm - self.G) < 0.25 * self.G)
            and (gyro_norm_dps < 10.0)
            and mag_ok
        )

        if is_stationary:
            self._stationary_count += 1
        else:
            self._stationary_count = 0

        # If stationary for ~0.18s, ZUPT: zero velocity and slowly refine bias
        if self._stationary_count >= int(0.18 / self.dt):
            self.v[0] = 0.0
            self.v[1] = 0.0
            self.v[2] = 0.0
            gravity_s = _rotate_vector_by_quat((0.0, 0.0, self.G), _quat_conj(self.ahrs.q))
            ex = ax - gravity_s[0]
            ey = ay - gravity_s[1]
            ez = az - gravity_s[2]
            alpha = 0.008
            self._acc_bias[0] += alpha * ex
            self._acc_bias[1] += alpha * ey
            self._acc_bias[2] += alpha * ez
            return tuple(self.v)

        # Integrate (no per-sample velocity damping — it falsely kills constant-speed segments)
        self.v[0] += ax_lin * self.dt
        self.v[1] += ay_lin * self.dt
        self.v[2] += az_lin * self.dt
        return tuple(self.v)


class SpeedMeasurementTab(QWidget):
    def __init__(self):
        super().__init__()
        self._active = False
        self._ui_timer = QTimer(self)
        self._ui_timer.setInterval(UI_REFRESH_MS)
        self._ui_timer.timeout.connect(self._refresh_ui)

        self.recorder = SpeedRecorder()
        # Warmup disabled: start integrating immediately.
        self.est = SpeedEstimator(sample_hz=50.0, warmup_sec=0.0)

        self._lock = threading.Lock()
        self._last_mag_update_ms = 0
        self._last_packet_perf_ns = None
        self._packet_rate_hz = 0.0
        # For this tab we target the SDK high-speed packed format: 3 samples per packet.
        # We display and compute Data Rate accordingly.
        self._samples_per_packet = 3
        self._last_packet_epoch_ms = None

        self._build_ui()
        signals.connection_state.connect(self._on_conn)

    def _build_ui(self):
        lay = QVBoxLayout(self)

        cfg = QGroupBox("Speed Measurement Configuration")
        cfg_lay = QVBoxLayout(cfg)
        cfg_lay.setContentsMargins(8, 4, 8, 6)

        self.acc_range_combo = QComboBox()
        self.acc_range_combo.addItems(["2", "4", "8", "16"])
        self.acc_range_combo.setCurrentText("2")

        self.gyro_range_combo = QComboBox()
        self.gyro_range_combo.addItems(["125", "250", "500", "1000", "2000"])
        self.gyro_range_combo.setCurrentText("2000")

        odr_lbl = QLabel("Fixed ODR: Accel 50 Hz | Gyro 50 Hz | Mag HIGH_ACCURACY (~20 Hz)")
        odr_lbl.setStyleSheet("color:#a6adc8; font-size:11px;")

        cfg_lay.addWidget(make_config_row([
            ("Accel Range (G):", self.acc_range_combo),
            ("Gyro Range (°/s):", self.gyro_range_combo),
        ]))
        cfg_lay.addWidget(odr_lbl)
        lay.addWidget(cfg)

        self.plot = RollingPlot("Velocity (Earth frame)", "m/s", ("Vx", "Vy", "Vz"))
        lay.addWidget(self.plot, stretch=1)

        val_box = QGroupBox("Live Values")
        vlay = QHBoxLayout(val_box)
        self.vx_lbl = QLabel("Vx: -- m/s")
        self.vy_lbl = QLabel("Vy: -- m/s")
        self.vz_lbl = QLabel("Vz: -- m/s")
        self.vmag_lbl = QLabel("Speed: -- m/s")
        for l in (self.vx_lbl, self.vy_lbl, self.vz_lbl, self.vmag_lbl):
            l.setFont(QFont(FONT_FAMILY, 14, QFont.Bold))
            vlay.addWidget(l)
        self.stats_lbl = QLabel(
            "Phase: idle | Packet Rate: 0.00 Hz | Data Rate: 0.00 Hz | Samples/Packet: 3 | Samples: 0"
        )
        self.stats_lbl.setStyleSheet("color:#a6adc8; font-size:11px;")
        vlay.addStretch()
        vlay.addWidget(self.stats_lbl)
        lay.addWidget(val_box)

        btn_row = QHBoxLayout()
        self.start_btn = QPushButton("▶ Start Measurement")
        self.start_btn.setEnabled(False)
        self.start_btn.clicked.connect(self.toggle)
        clear_btn = QPushButton("🧹 Clear Data")
        clear_btn.clicked.connect(self.clear_data)
        save_btn = QPushButton("💾 Save CSV")
        save_btn.clicked.connect(self.save_csv)
        btn_row.addWidget(self.start_btn)
        btn_row.addWidget(clear_btn)
        btn_row.addWidget(save_btn)
        lay.addLayout(btn_row)

        info = QLabel(
            "Velocity is computed by integrating gravity-removed linear acceleration.\n"
            "ZUPT (zero-velocity update) tries to keep velocity near 0 when the board is stationary.\n"
            "Physics: constant speed ⇒ ~0 linear acceleration ⇒ velocity from integration stays flat between pushes — "
            "magnetometer helps heading and ZUPT (stationary detection), not speed.\n"
            "Pure IMU speed drifts without GPS or other references."
        )
        info.setStyleSheet("color:#a6adc8; font-size:11px;")
        lay.addWidget(info)

    def clear_data(self):
        self.recorder.clear()
        # Warmup disabled: integrate immediately.
        self.est.reset(warmup_sec=0.0)
        self.plot.clear_plot()
        self._refresh_ui()

    def save_csv(self):
        default = f"Speed_{datetime.now(_HKT).strftime('%Y%m%dT%H%M%S')}.csv"
        headers = ["Sample Index", "PC Timestamp (ms)", "Vx (m/s)", "Vy (m/s)", "Vz (m/s)", "Speed (m/s)"]
        _save_rows(self, "Save Speed CSV", default, headers, self.recorder.csv_rows())

    def _gyro_sink(self, samples):
        # samples can contain 1..N; we just keep latest for fusion.
        for s in samples:
            gx, gy, gz = s["values"]
            with self._lock:
                self.est.update_gyro(gx, gy, gz)

    def _mag_sink(self, samples):
        # Only latest is needed.
        for s in samples:
            mx, my, mz = s["values"]
            with self._lock:
                self.est.update_mag(mx, my, mz)
                self._last_mag_update_ms = _host_ms()

    def _acc_sink(self, samples, packed=False, packet_epoch_ms=None):
        # Assume fixed ODR 50Hz and no packet loss as requested.
        with self._lock:
            # Packet rate: prefer device epoch deltas (stable, not affected by OS callback burstiness).
            if packet_epoch_ms is not None and self._last_packet_epoch_ms is not None:
                dt_ms = float(packet_epoch_ms) - float(self._last_packet_epoch_ms)
                if dt_ms > 0.0:
                    self._packet_rate_hz = 1000.0 / dt_ms
            else:
                # Fallback: PC-side timing (can be bursty on Windows).
                perf_ns = _perf_ns()
                if self._last_packet_perf_ns is not None and perf_ns > self._last_packet_perf_ns:
                    self._packet_rate_hz = 1e9 / (perf_ns - self._last_packet_perf_ns)
                self._last_packet_perf_ns = perf_ns

            if packet_epoch_ms is not None:
                self._last_packet_epoch_ms = float(packet_epoch_ms)
            # Speed tab assumption: packed 3-sample packets.
            # (Some SDK decode paths still deliver `samples` length 1 even when the packet cadence
            # corresponds to 3 samples/packet, so we keep this fixed.)
            self._samples_per_packet = 3
        for s in samples:
            ax, ay, az = s["values"]
            with self._lock:
                vx, vy, vz = self.est.update_accel_and_step(ax, ay, az)
            self.recorder.append(vx, vy, vz)

    def toggle(self):
        if not self._active:
            self.clear_data()
            self._ui_timer.start()
            self._active = True
            self.start_btn.setText("⏹ Stop Measurement")

            acc_rng = float(self.acc_range_combo.currentText())
            gyro_rng = self.gyro_range_combo.currentText()

            threading.Thread(
                target=device_mgr.start_gyro_50hz,
                kwargs={"sink": self._gyro_sink, "rng_key": gyro_rng},
                daemon=True,
            ).start()
            threading.Thread(
                target=device_mgr.start_mag_20hz,
                kwargs={"sink": self._mag_sink},
                daemon=True,
            ).start()
            threading.Thread(
                target=device_mgr.start_accel_50hz,
                kwargs={"sink": self._acc_sink, "rng_g": acc_rng},
                daemon=True,
            ).start()
        else:
            self._ui_timer.stop()
            self._active = False
            self.start_btn.setText("▶ Start Measurement")
            threading.Thread(target=device_mgr.stop_all, daemon=True).start()

    def _refresh_ui(self):
        snap = self.recorder.snapshot()
        self.plot.set_series(snap["plot_x"], *snap["plot_data"])
        vx, vy, vz = snap["latest"]
        if vx is not None:
            self.vx_lbl.setText(f"Vx: {vx:+.4f} m/s")
            self.vy_lbl.setText(f"Vy: {vy:+.4f} m/s")
            self.vz_lbl.setText(f"Vz: {vz:+.4f} m/s")
            self.vmag_lbl.setText(f"Speed: {math.sqrt(vx*vx + vy*vy + vz*vz):.4f} m/s")
        with self._lock:
            packet_rate = float(self._packet_rate_hz or 0.0)
            spp = 3
        data_rate = 3.0 * packet_rate
        self.stats_lbl.setText(
            f"Phase: {self.est.phase} | "
            f"Packet Rate: {packet_rate:.2f} Hz | "
            f"Data Rate: {data_rate:.2f} Hz | "
            f"Samples/Packet: {spp} | "
            f"Samples: {snap['samples']}"
        )

    @pyqtSlot(str)
    def _on_conn(self, state):
        self.start_btn.setEnabled(state == "connected")
        if state == "disconnected":
            self._ui_timer.stop()
            self._active = False
            self.start_btn.setText("▶ Start Measurement")


@dataclass
class AccelSample:
    sample_index: int
    packet_index: int
    device_epoch_ms: Optional[float]
    pc_ms: int
    ax_g: float
    ay_g: float
    az_g: float


class VibrationRecorder:
    """
    Stores:
    - Full time series for CSV export
    - Rolling buffers for time plot
    - Sliding 5s window for spectrum
    """

    def __init__(self, plot_samples=PLOT_SAMPLES, fs_hz=50.0, spectrum_window_sec=5.0):
        self.plot_samples = int(plot_samples)
        self.fs_hz = float(fs_hz)
        self.window_len = max(1, int(round(float(spectrum_window_sec) * self.fs_hz)))
        self._lock = threading.Lock()
        self.clear()

    def clear(self):
        with self._lock:
            self.rows: List[AccelSample] = []
            self.sample_count = 0
            self.packet_count = 0
            self.packet_rate_hz = 0.0
            self._last_packet_epoch_ms = None
            self._plot_buffers = [deque(maxlen=self.plot_samples) for _ in range(3)]
            self._latest = (None, None, None)
            self._win_buffers = [deque(maxlen=self.window_len) for _ in range(3)]

    def append_packet(self, samples: List[dict], packet_epoch_ms: Optional[float] = None):
        pc_ms = _host_ms()
        with self._lock:
            # Packet rate from device epoch
            if packet_epoch_ms is not None and self._last_packet_epoch_ms is not None:
                dt_ms = float(packet_epoch_ms) - float(self._last_packet_epoch_ms)
                if dt_ms > 0:
                    self.packet_rate_hz = 1000.0 / dt_ms
            if packet_epoch_ms is not None:
                self._last_packet_epoch_ms = float(packet_epoch_ms)

            self.packet_count += 1
            pkt_idx = self.packet_count

            for s in samples:
                self.sample_count += 1
                ax, ay, az = (float(s["values"][0]), float(s["values"][1]), float(s["values"][2]))
                epoch_ms = s.get("epoch_ms", packet_epoch_ms)
                self.rows.append(
                    AccelSample(
                        sample_index=self.sample_count,
                        packet_index=pkt_idx,
                        device_epoch_ms=epoch_ms,
                        pc_ms=pc_ms,
                        ax_g=ax,
                        ay_g=ay,
                        az_g=az,
                    )
                )
                self._latest = (ax, ay, az)
                self._plot_buffers[0].append(ax)
                self._plot_buffers[1].append(ay)
                self._plot_buffers[2].append(az)
                self._win_buffers[0].append(ax)
                self._win_buffers[1].append(ay)
                self._win_buffers[2].append(az)

    def snapshot_time(self):
        with self._lock:
            points = len(self._plot_buffers[0]) if self._plot_buffers else 0
            start = self.sample_count - points + 1 if points else 1
            plot_x = list(range(start, self.sample_count + 1)) if points else []
            return {
                "latest": self._latest,
                "plot_x": plot_x,
                "plot_data": [list(b) for b in self._plot_buffers],
                "samples": self.sample_count,
                "packets": self.packet_count,
                "packet_rate_hz": self.packet_rate_hz,
            }

    def snapshot_window(self):
        with self._lock:
            return [list(b) for b in self._win_buffers], self.fs_hz

    def csv_rows(self):
        with self._lock:
            return [
                [
                    r.sample_index,
                    r.packet_index,
                    "" if r.device_epoch_ms is None else r.device_epoch_ms,
                    r.pc_ms,
                    r.ax_g,
                    r.ay_g,
                    r.az_g,
                ]
                for r in self.rows
            ]


class VibrationMeasurementTab(QWidget):
    def __init__(self):
        super().__init__()
        self._active = False
        self._ui_timer = QTimer(self)
        self._ui_timer.setInterval(UI_REFRESH_MS)
        self._ui_timer.timeout.connect(self._refresh_ui)

        self.recorder = VibrationRecorder(plot_samples=PLOT_SAMPLES, fs_hz=50.0, spectrum_window_sec=5.0)

        self._lock = threading.Lock()
        self._build_ui()
        signals.connection_state.connect(self._on_conn)

    def _build_ui(self):
        lay = QVBoxLayout(self)

        cfg = QGroupBox("Vibration Measurement Configuration")
        cfg_lay = QVBoxLayout(cfg)
        cfg_lay.setContentsMargins(8, 4, 8, 6)

        self.acc_range_combo = QComboBox()
        self.acc_range_combo.addItems(["2", "4", "8", "16"])
        self.acc_range_combo.setCurrentText("2")

        odr_lbl = QLabel("Fixed ODR: Accel 50 Hz | Packed format: 3 samples/packet | Spectrum window: 5 s")
        odr_lbl.setStyleSheet("color:#a6adc8; font-size:11px;")

        cfg_lay.addWidget(make_config_row([
            ("Accel Range (G):", self.acc_range_combo),
        ]))
        cfg_lay.addWidget(odr_lbl)
        lay.addWidget(cfg)

        plots_row = QHBoxLayout()
        self.time_plot = RollingPlot("Accelerometer (time domain)", "Acceleration (g)", ("X", "Y", "Z"))
        self.spec_plot = SpectrumPlot("Frequency spectrum (sliding 5 s)", "Magnitude (dB)", ("X", "Y", "Z"))
        plots_row.addWidget(self.time_plot, stretch=1)
        plots_row.addWidget(self.spec_plot, stretch=1)
        lay.addLayout(plots_row, stretch=1)

        val_box = QGroupBox("Live Values")
        vlay = QHBoxLayout(val_box)
        self.ax_lbl = QLabel("X: -- g")
        self.ay_lbl = QLabel("Y: -- g")
        self.az_lbl = QLabel("Z: -- g")
        for l in (self.ax_lbl, self.ay_lbl, self.az_lbl):
            l.setFont(QFont(FONT_FAMILY, 14, QFont.Bold))
            vlay.addWidget(l)
        self.stats_lbl = QLabel("Packet Rate: 0.00 Hz | Data Rate: 0.00 Hz | Samples: 0 | Packets: 0")
        self.stats_lbl.setStyleSheet("color:#a6adc8; font-size:11px;")
        vlay.addStretch()
        vlay.addWidget(self.stats_lbl)
        lay.addWidget(val_box)

        btn_row = QHBoxLayout()
        self.start_btn = QPushButton("▶ Start Measurement")
        self.start_btn.setEnabled(False)
        self.start_btn.clicked.connect(self.toggle)
        clear_btn = QPushButton("🧹 Clear Data")
        clear_btn.clicked.connect(self.clear_data)
        save_btn = QPushButton("💾 Save CSV")
        save_btn.clicked.connect(self.save_csv)
        btn_row.addWidget(self.start_btn)
        btn_row.addWidget(clear_btn)
        btn_row.addWidget(save_btn)
        lay.addLayout(btn_row)

    def clear_data(self):
        self.recorder.clear()
        self.time_plot.clear_plot()
        self.spec_plot.clear_plot()
        self._refresh_ui()

    def save_csv(self):
        default = f"Vibration_Accel_{datetime.now(_HKT).strftime('%Y%m%dT%H%M%S')}.csv"
        headers = ["Sample Index", "Packet Index", "Device Epoch (ms)", "PC Timestamp (ms)", "Ax (g)", "Ay (g)", "Az (g)"]
        _save_rows(self, "Save Vibration CSV", default, headers, self.recorder.csv_rows())

    def _acc_sink(self, samples, packed=False, packet_epoch_ms=None):
        # We request packed; if SDK doesn't decode as 3, packet epoch cadence still reflects packing.
        self.recorder.append_packet(samples=samples, packet_epoch_ms=packet_epoch_ms)

    def toggle(self):
        if not self._active:
            self.clear_data()
            self._ui_timer.start()
            self._active = True
            self.start_btn.setText("⏹ Stop Measurement")

            acc_rng = float(self.acc_range_combo.currentText())
            threading.Thread(
                target=device_mgr.start_accel_50hz,
                kwargs={"sink": self._acc_sink, "rng_g": acc_rng},
                daemon=True,
            ).start()
        else:
            self._ui_timer.stop()
            self._active = False
            self.start_btn.setText("▶ Start Measurement")
            threading.Thread(target=device_mgr.stop_accel, daemon=True).start()

    def _refresh_ui(self):
        snap = self.recorder.snapshot_time()
        self.time_plot.set_series(snap["plot_x"], *snap["plot_data"])

        ax, ay, az = snap["latest"]
        if ax is not None:
            self.ax_lbl.setText(f"X: {ax:+.4f} g")
            self.ay_lbl.setText(f"Y: {ay:+.4f} g")
            self.az_lbl.setText(f"Z: {az:+.4f} g")

        packet_rate = float(snap["packet_rate_hz"] or 0.0)
        data_rate = 3.0 * packet_rate
        self.stats_lbl.setText(
            f"Packet Rate: {packet_rate:.2f} Hz | "
            f"Data Rate: {data_rate:.2f} Hz | "
            f"Samples: {snap['samples']} | "
            f"Packets: {snap['packets']}"
        )

        # Spectrum: compute from sliding 5s window at 50 Hz.
        (win_xyz, fs_hz) = self.recorder.snapshot_window()
        if win_xyz and len(win_xyz[0]) >= int(0.8 * 5.0 * fs_hz):
            freqs, mag_x = _rfft_magnitude(win_xyz[0], fs_hz)
            _, mag_y = _rfft_magnitude(win_xyz[1], fs_hz)
            _, mag_z = _rfft_magnitude(win_xyz[2], fs_hz)
            self.spec_plot.set_spectrum(
                freqs,
                _linear_mag_to_db(mag_x),
                _linear_mag_to_db(mag_y),
                _linear_mag_to_db(mag_z),
            )

    @pyqtSlot(str)
    def _on_conn(self, state):
        self.start_btn.setEnabled(state == "connected")
        if state == "disconnected":
            self._ui_timer.stop()
            self._active = False
            self.start_btn.setText("▶ Start Measurement")


class GestureMotionTab(QWidget):
    """
    Combines MetaWear Bosch/BMI270 firmware detectors with simple on-device heuristics on 50 Hz accel.
    Mutually exclusive with Speed/Vibration streaming (same accelerometer pipeline).
    """

    _ROW_SPECS = (
        ("tap", "👆", "Tap"),
        ("orient", "📱", "Orientation"),
        ("activity", "🧍", "Activity classifier"),
        ("steps", "👟", "Step counter"),
        ("stepdet", "👣", "Step detector pulses"),
        ("free_fall", "🪂", "Free-fall (heuristic)"),
        ("impact", "💥", "Impact / landing"),
        ("shake", "📳", "Shake"),
        ("stationary", "🧘", "Idle / stationary"),
        ("cadence", "🚶", "Cadence estimate (steps/min)"),
        ("stairs_hint", "🪜", "Stairs hint"),
    )

    def __init__(self):
        super().__init__()
        self._active = False
        self._heur = GestureHeuristics()
        self._step_det_total = 0
        self._step_count_total = 0
        self._pulse_streak = 0
        self._pair_pulses = 0
        self._walk_run_active = False
        self._last_pulse_ms = None
        self._run_interval_ms = 430
        self._status_labels = {}
        self._transient_keys = {
            "tap",
            "free_fall",
            "impact",
            "shake",
            "stationary",
            "cadence",
            "stairs_hint",
        }
        self._expiry_deadlines = {}
        self._log_rows = []
        self._log_lock = threading.Lock()
        self._step_poll_timer = QTimer(self)
        self._step_poll_timer.setInterval(450)
        self._step_poll_timer.timeout.connect(device_mgr.poll_step_counter_read)
        self._clear_timer = QTimer(self)
        self._clear_timer.setInterval(250)
        self._clear_timer.timeout.connect(self._clear_expired_statuses)
        self._clear_timer.start()
        self._build_ui()
        signals.connection_state.connect(self._on_conn)
        gesture_signals.accel_implementation.connect(self._on_accel_impl)
        gesture_signals.tap.connect(self._on_tap)
        gesture_signals.activity.connect(self._on_activity)
        gesture_signals.step_count.connect(self._on_step_count)
        gesture_signals.step_pulse.connect(self._on_step_pulse)
        gesture_signals.orientation.connect(self._on_orient)
        gesture_signals.heuristic.connect(self._on_heuristic)

    def _build_ui(self):
        lay = QVBoxLayout(self)

        info = QLabel(
            "MetaMotion S uses a BMI270 accelerometer: Bosch smartphone tap/orientation APIs are not supported on that "
            "chip (Mbient docs) — tap and coarse orientation here use heuristics from 50 Hz accel. "
            "Cadence estimate is a rough steps-per-minute estimate from accelerometer peak timing. "
            "The step counter is polled (SDK: do not stream the step-counter signal)."
        )
        info.setWordWrap(True)
        info.setStyleSheet("color:#a6adc8; font-size:11px;")
        lay.addWidget(info)

        cfg = QGroupBox("Configuration")
        cfg_lay = QVBoxLayout(cfg)
        self.acc_range_combo = QComboBox()
        self.acc_range_combo.addItems(["2", "4", "8", "16"])
        self.acc_range_combo.setCurrentText("8")
        cfg_lay.addWidget(make_config_row([("Accel range (g):", self.acc_range_combo)]))
        lay.addWidget(cfg)

        det = QGroupBox("Detections")
        det_lay = QVBoxLayout(det)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        inner = QWidget()
        grid = QGridLayout(inner)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)
        for row, (key, icon, title) in enumerate(self._ROW_SPECS):
            ic = QLabel(icon)
            ic.setFont(QFont(FONT_FAMILY, 16))
            ic.setMinimumWidth(36)
            tl = QLabel(title)
            tl.setStyleSheet("color:#89b4fa; font-weight:bold;")
            st = QLabel("—")
            st.setStyleSheet("color:#cdd6f4;")
            st.setWordWrap(True)
            self._status_labels[key] = st
            grid.addWidget(ic, row, 0)
            grid.addWidget(tl, row, 1)
            grid.addWidget(st, row, 2)
        grid.setColumnStretch(2, 1)
        scroll.setWidget(inner)
        scroll.setMinimumHeight(220)
        det_lay.addWidget(scroll)
        lay.addWidget(det, stretch=1)

        log_box = QGroupBox("Event log")
        log_lay = QVBoxLayout(log_box)
        self.log_list = QListWidget()
        self.log_list.setMaximumHeight(160)
        log_lay.addWidget(self.log_list)
        lay.addWidget(log_box)

        btn_row = QHBoxLayout()
        self.start_btn = QPushButton("▶ Start detection")
        self.start_btn.setEnabled(False)
        self.start_btn.clicked.connect(self.toggle)
        clr = QPushButton("🧹 Clear UI / log")
        clr.clicked.connect(self.clear_ui)
        rst = QPushButton("↺ Reset HW step counter")
        rst.clicked.connect(self._reset_hw_steps)
        sav = QPushButton("💾 Save log CSV")
        sav.clicked.connect(self.save_log_csv)
        btn_row.addWidget(self.start_btn)
        btn_row.addWidget(clr)
        btn_row.addWidget(rst)
        btn_row.addWidget(sav)
        lay.addLayout(btn_row)

    def _append_log(self, text: str):
        ts = datetime.now(_HKT).strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{ts.split()[1]}] {text}"
        with self._log_lock:
            self._log_rows.append([ts, text])
        self.log_list.addItem(line)
        self.log_list.scrollToBottom()
        while self.log_list.count() > 400:
            self.log_list.takeItem(0)

    def _accel_sink(self, samples, packed=False, packet_epoch_ms=None):
        for s in samples:
            ax, ay, az = s["values"]
            self._heur.feed(ax, ay, az, time.perf_counter())

    def _gyro_sink(self, samples):
        for s in samples:
            gx, gy, gz = s["values"]
            self._heur.feed_gyro(gx, gy, gz, time.perf_counter())

    def _set_status(self, key: str, text: str, auto_clear: bool = True):
        if key in self._status_labels:
            self._status_labels[key].setText(text)
        if auto_clear and key in self._transient_keys:
            self._expiry_deadlines[key] = _host_ms() + 2000

    def _clear_expired_statuses(self):
        now = _host_ms()
        expired = [k for k, deadline in self._expiry_deadlines.items() if deadline <= now]
        for key in expired:
            if key in self._status_labels:
                self._status_labels[key].setText("—")
            self._expiry_deadlines.pop(key, None)
        if self._last_pulse_ms is not None and (now - self._last_pulse_ms) > 2500 and (self._pulse_streak or self._walk_run_active):
            self._reset_pulse_sequence(set_activity=True)

    def toggle(self):
        if not self._active:
            self._active = True
            self.start_btn.setText("⏹ Stop detection")
            rng = float(self.acc_range_combo.currentText())

            def _run():
                device_mgr.start_gesture_detection(self._accel_sink, gyro_sink=self._gyro_sink, rng_g=rng)

            threading.Thread(target=_run, daemon=True).start()
        else:
            self._step_poll_timer.stop()
            self._active = False
            self.start_btn.setText("▶ Start detection")
            threading.Thread(target=device_mgr.stop_gesture_detection, daemon=True).start()

    def clear_ui(self):
        self._heur.reset()
        self._step_det_total = 0
        self._step_count_total = 0
        self._reset_pulse_sequence(set_activity=False)
        self._expiry_deadlines.clear()
        with self._log_lock:
            self._log_rows.clear()
        self.log_list.clear()
        for key in self._status_labels:
            self._status_labels[key].setText("—")

    def _reset_pulse_sequence(self, set_activity: bool):
        self._pulse_streak = 0
        self._pair_pulses = 0
        self._walk_run_active = False
        self._last_pulse_ms = None
        if "stepdet" in self._status_labels:
            self._status_labels["stepdet"].setText("0 pulse(s)")
        if set_activity:
            self._set_status("activity", "Still (pulse timeout)", auto_clear=False)

    def _reset_hw_steps(self):
        threading.Thread(target=device_mgr.reset_step_counter_hw, daemon=True).start()
        self._append_log("Requested hardware step counter reset.")

    def save_log_csv(self):
        default = f"Gesture_Log_{datetime.now(_HKT).strftime('%Y%m%dT%H%M%S')}.csv"
        with self._log_lock:
            rows = [list(r) for r in self._log_rows]
        _save_rows(self, "Save gesture log", default, ["Timestamp (HKT)", "Event"], rows)

    @pyqtSlot(str)
    def _on_tap(self, t):
        self._set_status("tap", t)
        self._append_log(f"Tap: {t}")

    @pyqtSlot(str)
    def _on_activity(self, t):
        self._set_status("activity", t)
        self._heur.set_activity_hint(t)

    @pyqtSlot(int)
    def _on_accel_impl(self, impl: int):
        self._heur.set_accel_implementation(impl if impl >= 0 else None)

    @pyqtSlot(int)
    def _on_step_count(self, n):
        pass

    @pyqtSlot()
    def _on_step_pulse(self):
        now = _host_ms()
        dt_ms = None if self._last_pulse_ms is None else (now - self._last_pulse_ms)
        if dt_ms is None or dt_ms > 2500:
            self._pulse_streak = 0
            self._pair_pulses = 0
            self._walk_run_active = False
        self._last_pulse_ms = now
        self._step_det_total += 1
        self._pulse_streak += 1
        self._set_status("stepdet", f"{self._pulse_streak} pulse(s)", auto_clear=False)
        if not self._walk_run_active:
            if self._pulse_streak >= 10:
                self._walk_run_active = True
                self._pair_pulses = 0
                gait = "Running" if (dt_ms is not None and dt_ms < self._run_interval_ms) else "Walking"
                self._set_status("activity", f"{gait} (pulse cadence)", auto_clear=False)
            self._append_log("Step detector pulse")
            return
        gait = "Running" if (dt_ms is not None and dt_ms < self._run_interval_ms) else "Walking"
        self._set_status("activity", f"{gait} (pulse cadence)", auto_clear=False)
        self._pair_pulses += 1
        if self._pair_pulses >= 2:
            self._pair_pulses = 0
            self._step_count_total += 1
            self._set_status("steps", str(self._step_count_total), auto_clear=False)
        self._append_log("Step detector pulse")

    @pyqtSlot(str)
    def _on_orient(self, t):
        self._set_status("orient", t, auto_clear=False)
        self._append_log(f"Orientation: {t}")

    @pyqtSlot(str, str)
    def _on_heuristic(self, key: str, text: str):
        self._set_status(key, text)
        self._append_log(f"{key}: {text}")

    @pyqtSlot(str)
    def _on_conn(self, state):
        self.start_btn.setEnabled(state == "connected")
        if state == "disconnected":
            self._step_poll_timer.stop()
            self._active = False
            self.start_btn.setText("▶ Start detection")


class CompassGauge(QWidget):
    def __init__(self):
        super().__init__()
        self._heading_deg = 0.0
        self._calibrated = False
        self.setMinimumSize(320, 320)

    def set_heading(self, heading_deg: float):
        self._heading_deg = float(heading_deg) % 360.0
        self.update()

    def set_calibrated(self, calibrated: bool):
        self._calibrated = bool(calibrated)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        w = self.width()
        h = self.height()
        side = min(w, h)
        cx = w / 2.0
        cy = h / 2.0
        radius = side * 0.44

        p.fillRect(self.rect(), QColor("#1e1e2e"))
        p.setPen(QPen(QColor("#89b4fa"), 3))
        p.setBrush(QColor("#181825"))
        p.drawEllipse(QPointF(cx, cy), radius, radius)

        for deg in range(0, 360, 10):
            ang = math.radians(deg - 90.0)
            outer = QPointF(cx + radius * math.cos(ang), cy + radius * math.sin(ang))
            inner_r = radius * (0.78 if deg % 90 == 0 else 0.84 if deg % 30 == 0 else 0.89)
            inner = QPointF(cx + inner_r * math.cos(ang), cy + inner_r * math.sin(ang))
            p.setPen(QPen(QColor("#cdd6f4"), 2 if deg % 30 == 0 else 1))
            p.drawLine(inner, outer)

        font = QFont(FONT_FAMILY, 13, QFont.Bold)
        p.setFont(font)
        labels = {"N": 0, "E": 90, "S": 180, "W": 270}
        for text, deg in labels.items():
            ang = math.radians(deg - 90.0)
            lr = radius * 0.64
            lx = cx + lr * math.cos(ang)
            ly = cy + lr * math.sin(ang)
            p.setPen(QColor("#f38ba8") if text == "N" else QColor("#cdd6f4"))
            p.drawText(int(lx - 10), int(ly + 6), text)

        p.setPen(QPen(QColor("#45475a"), 2))
        p.drawEllipse(QPointF(cx, cy), radius * 0.08, radius * 0.08)

        if self._calibrated:
            ang = math.radians(self._heading_deg - 90.0)
            tip = QPointF(cx + radius * 0.68 * math.cos(ang), cy + radius * 0.68 * math.sin(ang))
            left = QPointF(cx + radius * 0.14 * math.cos(ang + 2.45), cy + radius * 0.14 * math.sin(ang + 2.45))
            right = QPointF(cx + radius * 0.14 * math.cos(ang - 2.45), cy + radius * 0.14 * math.sin(ang - 2.45))
            tail = QPointF(cx - radius * 0.24 * math.cos(ang), cy - radius * 0.24 * math.sin(ang))
            p.setPen(QPen(QColor("#f38ba8"), 2))
            p.setBrush(QColor("#f38ba8"))
            p.drawPolygon(QPolygonF([tip, left, tail, right]))
        else:
            p.setPen(QColor("#a6adc8"))
            p.setFont(QFont(FONT_FAMILY, 11, QFont.Bold))
            p.drawText(int(cx - 48), int(cy + 6), "Calibrate")


class DigitalCompassTab(QWidget):
    mag_sample = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        self._active = False
        self._calibrating = False
        self._cal_samples = []
        self._offset_x = 0.0
        self._offset_y = 0.0
        self._calibrated = False
        self._heading_vec = None
        self._build_ui()
        self.mag_sample.connect(self._on_mag_sample)
        signals.connection_state.connect(self._on_conn)

    def _build_ui(self):
        lay = QVBoxLayout(self)

        info = QLabel(
            "Keep the module flat. Use 5 s calibration first: rotate it slowly on a horizontal surface "
            "for about two or three turns. Calibration removes XY hard-iron offset; Z is ignored."
        )
        info.setWordWrap(True)
        info.setStyleSheet("color:#a6adc8; font-size:11px;")
        lay.addWidget(info)

        top = QHBoxLayout()
        self.gauge = CompassGauge()
        top.addWidget(self.gauge, stretch=1)

        side = QGroupBox("Compass Status")
        grid = QGridLayout(side)
        self.state_lbl = QLabel("Not started")
        self.heading_lbl = QLabel("--°")
        self.raw_lbl = QLabel("X: -- uT | Y: -- uT | Z: -- uT")
        self.corr_lbl = QLabel("Xc: -- uT | Yc: -- uT")
        self.offset_lbl = QLabel("Offset X: 0.00 uT | Offset Y: 0.00 uT")
        self.instr_lbl = QLabel("Press '5 s Calibration' and rotate the module slowly on a flat surface.")
        self.instr_lbl.setWordWrap(True)
        self.instr_lbl.setStyleSheet("color:#a6adc8; font-size:11px;")
        rows = [
            ("State:", self.state_lbl),
            ("Heading:", self.heading_lbl),
            ("Raw B-field:", self.raw_lbl),
            ("Corrected XY:", self.corr_lbl),
            ("Offset:", self.offset_lbl),
            ("Instructions:", self.instr_lbl),
        ]
        for r, (name, ctrl) in enumerate(rows):
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignRight | Qt.AlignTop)
            grid.addWidget(lbl, r, 0)
            grid.addWidget(ctrl, r, 1)
        grid.setColumnStretch(1, 1)
        top.addWidget(side, stretch=1)
        lay.addLayout(top, stretch=1)

        btn_row = QHBoxLayout()
        self.start_btn = QPushButton("▶ Start Compass")
        self.start_btn.setEnabled(False)
        self.start_btn.clicked.connect(self.toggle_stream)
        self.cal_btn = QPushButton("🧭 5 s Calibration")
        self.cal_btn.setEnabled(False)
        self.cal_btn.clicked.connect(self.start_calibration)
        self.reset_btn = QPushButton("↺ Reset Calibration")
        self.reset_btn.setEnabled(False)
        self.reset_btn.clicked.connect(self.reset_calibration)
        btn_row.addWidget(self.start_btn)
        btn_row.addWidget(self.cal_btn)
        btn_row.addWidget(self.reset_btn)
        lay.addLayout(btn_row)

    def _mag_sink(self, samples):
        for s in samples:
            x, y, z = s["values"]
            self.mag_sample.emit(float(x), float(y), float(z))

    def _start_stream_bg(self):
        device_mgr.stop_all()
        time.sleep(0.15)
        device_mgr.start_mag_20hz(self._mag_sink)

    def toggle_stream(self):
        if not self._active:
            self._active = True
            self.start_btn.setText("⏹ Stop Compass")
            self.state_lbl.setText("Starting magnetometer stream...")
            threading.Thread(target=self._start_stream_bg, daemon=True).start()
        else:
            self._active = False
            self._calibrating = False
            self.start_btn.setText("▶ Start Compass")
            self.state_lbl.setText("Stopped")
            self.instr_lbl.setText("Press '5 s Calibration' and rotate the module slowly on a flat surface.")
            threading.Thread(target=device_mgr.stop_mag, daemon=True).start()

    def start_calibration(self):
        if not self._active:
            self.toggle_stream()
        self._calibrating = True
        self._cal_samples = []
        self.cal_btn.setEnabled(False)
        self.state_lbl.setText("Calibrating (5 s)")
        self.instr_lbl.setText(
            "Calibration running: keep the module flat and rotate it slowly two or three times "
            "for about ten seconds."
        )
        QTimer.singleShot(5000, self.finish_calibration)

    def finish_calibration(self):
        self._calibrating = False
        self.cal_btn.setEnabled(True)
        if len(self._cal_samples) < 20:
            self.state_lbl.setText("Calibration failed")
            self.instr_lbl.setText("Not enough samples collected. Try again and rotate more slowly for the full 5 s.")
            return
        xs = [p[0] for p in self._cal_samples]
        ys = [p[1] for p in self._cal_samples]
        self._offset_x = 0.5 * (max(xs) + min(xs))
        self._offset_y = 0.5 * (max(ys) + min(ys))
        self._calibrated = True
        self._heading_vec = None
        self.gauge.set_calibrated(True)
        self.offset_lbl.setText(f"Offset X: {self._offset_x:+.2f} uT | Offset Y: {self._offset_y:+.2f} uT")
        self.state_lbl.setText("Calibrated")
        self.instr_lbl.setText("Calibration complete. Keep the module flat for best heading accuracy.")
        self.reset_btn.setEnabled(True)

    def reset_calibration(self):
        self._calibrated = False
        self._offset_x = 0.0
        self._offset_y = 0.0
        self._heading_vec = None
        self.gauge.set_calibrated(False)
        self.heading_lbl.setText("--°")
        self.offset_lbl.setText("Offset X: 0.00 uT | Offset Y: 0.00 uT")
        self.state_lbl.setText("Calibration reset")
        self.instr_lbl.setText("Press '5 s Calibration' and rotate the module slowly on a flat surface.")
        self.reset_btn.setEnabled(False)

    @pyqtSlot(float, float, float)
    def _on_mag_sample(self, mx: float, my: float, mz: float):
        self.raw_lbl.setText(f"X: {mx:+.2f} uT | Y: {my:+.2f} uT | Z: {mz:+.2f} uT")
        if self._calibrating:
            self._cal_samples.append((mx, my))
        cx = mx - self._offset_x
        cy = my - self._offset_y
        self.corr_lbl.setText(f"Xc: {cx:+.2f} uT | Yc: {cy:+.2f} uT")
        if not self._calibrated:
            return
        if abs(cx) + abs(cy) < 1e-6:
            return
        heading = (math.degrees(math.atan2(cy, cx)) + 360.0) % 360.0
        ux = math.cos(math.radians(heading))
        uy = math.sin(math.radians(heading))
        if self._heading_vec is None:
            self._heading_vec = [ux, uy]
        else:
            alpha = 0.20
            self._heading_vec[0] = (1.0 - alpha) * self._heading_vec[0] + alpha * ux
            self._heading_vec[1] = (1.0 - alpha) * self._heading_vec[1] + alpha * uy
            n = math.hypot(self._heading_vec[0], self._heading_vec[1])
            if n > 0:
                self._heading_vec[0] /= n
                self._heading_vec[1] /= n
        heading_smooth = (math.degrees(math.atan2(self._heading_vec[1], self._heading_vec[0])) + 360.0) % 360.0
        self.gauge.set_heading(heading_smooth)
        self.heading_lbl.setText(f"{heading_smooth:6.1f}° ({_heading_cardinal(heading_smooth)})")
        if not self._calibrating:
            self.state_lbl.setText("Heading active")

    @pyqtSlot(str)
    def _on_conn(self, state):
        enabled = state == "connected"
        self.start_btn.setEnabled(enabled)
        self.cal_btn.setEnabled(enabled and not self._calibrating)
        if state == "disconnected":
            self._active = False
            self._calibrating = False
            self.start_btn.setText("▶ Start Compass")
            self.state_lbl.setText("Not connected")
            self.instr_lbl.setText("Connect to the module first.")


class InclinationGauge(QWidget):
    def __init__(self):
        super().__init__()
        self._tilt_x_deg = 0.0
        self._tilt_y_deg = 0.0
        self._signed_incl_deg = 0.0
        # Keep increasing separation up to +/-180 deg; circles may partially leave the widget.
        self._max_angle_deg = 180.0
        self.setMinimumSize(360, 360)

    def set_tilt(self, tilt_x_deg: float, tilt_y_deg: float, signed_incl_deg: float):
        self._tilt_x_deg = float(tilt_x_deg)
        self._tilt_y_deg = float(tilt_y_deg)
        self._signed_incl_deg = float(signed_incl_deg)
        self.update()

    def _interp_color(self, t: float) -> QColor:
        t = max(0.0, min(1.0, float(t)))
        g = QColor("#a6e3a1")
        r = QColor("#f38ba8")
        rr = int(g.red() + t * (r.red() - g.red()))
        gg = int(g.green() + t * (r.green() - g.green()))
        bb = int(g.blue() + t * (r.blue() - g.blue()))
        return QColor(rr, gg, bb)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor("#1e1e2e"))

        w = self.width()
        h = self.height()
        cx = w / 2.0
        cy = h / 2.0
        side = min(w, h)
        circle_r = side * 0.16
        max_sep = side * 1.25

        mag = math.hypot(self._tilt_x_deg, self._tilt_y_deg)
        t = min(mag / self._max_angle_deg, 1.0)
        color = self._interp_color(t)

        sx = max(-1.0, min(1.0, self._tilt_x_deg / self._max_angle_deg))
        sy = max(-1.0, min(1.0, -self._tilt_y_deg / self._max_angle_deg))
        dx = sx * max_sep
        dy = sy * max_sep

        # Keep the circle that moves to the left when +X tilt is applied.
        c1 = QPointF(cx - dx / 2.0, cy - dy / 2.0)

        pen = QPen(color, 4)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(c1, circle_r, circle_r)

        p.setPen(QColor("#cdd6f4"))
        p.setFont(QFont(FONT_FAMILY, 18, QFont.Bold))
        p.drawText(int(cx - 44), int(cy + 7), f"{self._signed_incl_deg:+.1f}°")


class InclinationMeasurementTab(QWidget):
    def __init__(self):
        super().__init__()
        self._active = False
        self._g_lp = None
        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        self._zero_roll_deg = 0.0
        self._zero_pitch_deg = 0.0
        self._pending_cal = False
        self._build_ui()
        signals.connection_state.connect(self._on_conn)

    def _build_ui(self):
        lay = QVBoxLayout(self)

        info = QLabel(
            "Uses low-pass accelerometer gravity to estimate inclination. "
            "Press calibration to treat the current module pose as zero. "
            "The two hollow circles stay centered symmetrically about the origin."
        )
        info.setWordWrap(True)
        info.setStyleSheet("color:#a6adc8; font-size:11px;")
        lay.addWidget(info)

        top = QHBoxLayout()
        self.gauge = InclinationGauge()
        top.addWidget(self.gauge, stretch=1)

        side = QGroupBox("Inclination Status")
        grid = QGridLayout(side)
        self.state_lbl = QLabel("Not started")
        self.incl_lbl = QLabel("+0.0°")
        self.xy_lbl = QLabel("Rel X: +0.0° | Rel Y: +0.0°")
        self.zero_lbl = QLabel("Zero X: +0.0° | Zero Y: +0.0°")
        self.instr_lbl = QLabel("Start the inclination stream, then press calibration to set the current pose as zero.")
        self.instr_lbl.setWordWrap(True)
        self.instr_lbl.setStyleSheet("color:#a6adc8; font-size:11px;")
        rows = [
            ("State:", self.state_lbl),
            ("Inclination:", self.incl_lbl),
            ("Relative tilt:", self.xy_lbl),
            ("Zero reference:", self.zero_lbl),
            ("Instructions:", self.instr_lbl),
        ]
        for r, (name, ctrl) in enumerate(rows):
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignRight | Qt.AlignTop)
            grid.addWidget(lbl, r, 0)
            grid.addWidget(ctrl, r, 1)
        grid.setColumnStretch(1, 1)
        top.addWidget(side, stretch=1)
        lay.addLayout(top, stretch=1)

        btn_row = QHBoxLayout()
        self.start_btn = QPushButton("▶ Start Inclination")
        self.start_btn.setEnabled(False)
        self.start_btn.clicked.connect(self.toggle_stream)
        self.cal_btn = QPushButton("🎯 Set Current as Zero")
        self.cal_btn.setEnabled(False)
        self.cal_btn.clicked.connect(self.calibrate_zero)
        btn_row.addWidget(self.start_btn)
        btn_row.addWidget(self.cal_btn)
        lay.addLayout(btn_row)

    def _start_stream_bg(self):
        device_mgr.stop_all()
        time.sleep(0.15)
        device_mgr.start_accel_50hz(self._accel_sink, rng_g=2.0)

    def toggle_stream(self):
        if not self._active:
            self._active = True
            self.start_btn.setText("⏹ Stop Inclination")
            self.state_lbl.setText("Starting accelerometer stream...")
            threading.Thread(target=self._start_stream_bg, daemon=True).start()
        else:
            self._active = False
            self.start_btn.setText("▶ Start Inclination")
            self.state_lbl.setText("Stopped")
            threading.Thread(target=device_mgr.stop_accel, daemon=True).start()

    def calibrate_zero(self):
        if not self._active:
            self._pending_cal = True
            self.toggle_stream()
            self.instr_lbl.setText("Inclination stream starting. Current pose will be set as zero on the first valid sample.")
            return
        self._apply_zero_calibration()

    def _apply_zero_calibration(self):
        self._zero_roll_deg = self._roll_deg
        self._zero_pitch_deg = self._pitch_deg
        self.zero_lbl.setText(f"Zero X: {self._zero_pitch_deg:+.1f}° | Zero Y: {self._zero_roll_deg:+.1f}°")
        self.state_lbl.setText("Calibrated")
        self.instr_lbl.setText("Current inclination has been set as zero.")
        self._pending_cal = False

    def _accel_sink(self, samples, packed=False, packet_epoch_ms=None):
        for s in samples:
            ax, ay, az = s["values"]
            if self._g_lp is None:
                self._g_lp = [ax, ay, az]
            else:
                alpha = 0.08
                self._g_lp[0] = (1.0 - alpha) * self._g_lp[0] + alpha * ax
                self._g_lp[1] = (1.0 - alpha) * self._g_lp[1] + alpha * ay
                self._g_lp[2] = (1.0 - alpha) * self._g_lp[2] + alpha * az

            gx, gy, gz = self._g_lp
            self._roll_deg = math.degrees(math.atan2(gy, gz))
            self._pitch_deg = math.degrees(math.atan2(-gx, math.sqrt(gy * gy + gz * gz)))

            if self._pending_cal:
                self._apply_zero_calibration()

            rel_x_deg = self._pitch_deg - self._zero_pitch_deg
            rel_y_deg = self._roll_deg - self._zero_roll_deg
            mag_deg = math.hypot(rel_x_deg, rel_y_deg)
            dominant = rel_x_deg if abs(rel_x_deg) >= abs(rel_y_deg) else rel_y_deg
            signed_deg = mag_deg if dominant >= 0 else -mag_deg

            self.gauge.set_tilt(rel_x_deg, rel_y_deg, signed_deg)
            self.incl_lbl.setText(f"{signed_deg:+.1f}°")
            self.xy_lbl.setText(f"Rel X: {rel_x_deg:+.1f}° | Rel Y: {rel_y_deg:+.1f}°")
            if self._active:
                self.state_lbl.setText("Inclination active")

    @pyqtSlot(str)
    def _on_conn(self, state):
        enabled = state == "connected"
        self.start_btn.setEnabled(enabled)
        self.cal_btn.setEnabled(enabled)
        if state == "disconnected":
            self._active = False
            self._pending_cal = False
            self._g_lp = None
            self.start_btn.setText("▶ Start Inclination")
            self.state_lbl.setText("Not connected")
            self.instr_lbl.setText("Connect to the module first.")


class PlaceholderTab(QWidget):
    def __init__(self, title: str):
        super().__init__()
        lay = QVBoxLayout(self)
        t = QLabel(title)
        t.setFont(QFont(FONT_FAMILY, 16, QFont.Bold))
        t.setStyleSheet("color:#89b4fa;")
        lay.addWidget(t)
        msg = QLabel("Scaffolded tab. Implementation coming next.")
        msg.setStyleSheet("color:#a6adc8; font-size:12px;")
        lay.addWidget(msg)
        lay.addStretch()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MetaMotionS High-Level Motion App v0.1")
        self.resize(1280, 900)
        self.setFont(QFont(FONT_FAMILY, 11))
        self._build_ui()
        signals.connection_state.connect(self._on_conn)
        signals.error.connect(self._on_error)

    def _build_ui(self):
        toolbar = self.addToolBar("Main")
        toolbar.setMovable(False)
        toolbar.setStyleSheet(
            "QToolBar { background: #181825; border-bottom: 1px solid #45475a; spacing: 8px; padding: 4px; }"
        )

        self.scan_btn = QPushButton("🔌 Scan && Connect")
        self.scan_btn.setFixedHeight(34)
        self.scan_btn.clicked.connect(self.scan_and_connect)
        toolbar.addWidget(self.scan_btn)

        sp = QWidget()
        sp.setFixedWidth(12)
        toolbar.addWidget(sp)

        self.conn_label = QLabel("Status: Not Connected")
        self.conn_label.setFont(QFont(FONT_FAMILY, 11, QFont.Bold))
        self.conn_label.setStyleSheet("color: #f38ba8; padding: 0 8px;")
        toolbar.addWidget(self.conn_label)

        self.info_label = QLabel("")
        self.info_label.setStyleSheet("color: #a6adc8; padding: 0 8px;")
        toolbar.addWidget(self.info_label)

        stretch = QWidget()
        stretch.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        toolbar.addWidget(stretch)

        self.disc_btn = QPushButton("⏏ Disconnect")
        self.disc_btn.setFixedHeight(34)
        self.disc_btn.setEnabled(False)
        self.disc_btn.clicked.connect(lambda: threading.Thread(target=device_mgr.disconnect, daemon=True).start())
        toolbar.addWidget(self.disc_btn)

        self.tabs = QTabWidget()
        self.tabs.addTab(SpeedMeasurementTab(), "🏎️ Speed Measurement")
        self.tabs.addTab(VibrationMeasurementTab(), "📳 Vibration Measurement")
        self.tabs.addTab(GestureMotionTab(), "🖐️ Gesture/Motion Detection")
        self.tabs.addTab(DigitalCompassTab(), "🧭 Digital Compass")
        self.tabs.addTab(InclinationMeasurementTab(), "📐 Inclination Measurement")

        self.setCentralWidget(self.tabs)
        self.statusBar().showMessage("Ready — Click 'Scan & Connect' to find your MetaMotionS device.")

    def scan_and_connect(self):
        dlg = ScanDialog(self)
        if dlg.exec_() != QDialog.Accepted:
            return
        self.scan_btn.setEnabled(False)
        self.statusBar().showMessage(f"Connecting to {dlg.selected_name} [{dlg.selected_mac}] …")
        threading.Thread(target=device_mgr.connect, args=(dlg.selected_mac,), daemon=True).start()

    @pyqtSlot(str)
    def _on_conn(self, state):
        if state == "connected":
            self.scan_btn.setEnabled(True)
            self.disc_btn.setEnabled(True)
            info = device_mgr.device.info if device_mgr.device else {}
            fw = info.get("firmware", "?")
            hw = info.get("hardware", "?")
            mac = device_mgr.device.address if device_mgr.device else "?"
            self.conn_label.setText(f"✅ Connected [{mac}]")
            self.conn_label.setStyleSheet("color: #a6e3a1; padding: 0 8px;")
            self.info_label.setText(f"FW: {fw} HW: {hw}")
            self.statusBar().showMessage(f"Connected — FW {fw} | HW {hw}")
        elif "connecting" in state:
            self.conn_label.setText(f"⏳ {state}")
            self.conn_label.setStyleSheet("color: #f9e2af; padding: 0 8px;")
            self.statusBar().showMessage(state)
        else:
            self.scan_btn.setEnabled(True)
            self.disc_btn.setEnabled(False)
            self.conn_label.setText("Status: Not Connected")
            self.conn_label.setStyleSheet("color: #f38ba8; padding: 0 8px;")
            self.info_label.setText("")
            self.statusBar().showMessage("Disconnected.")

    @pyqtSlot(str)
    def _on_error(self, msg):
        self.scan_btn.setEnabled(True)
        self.statusBar().showMessage(f"Error: {msg}")
        QMessageBox.critical(self, "Connection Error", msg)

    def closeEvent(self, event):
        threading.Thread(target=device_mgr.disconnect, daemon=True).start()
        event.accept()


def main():
    pg.setConfigOptions(antialias=True, foreground="#cdd6f4", background="#181825")
    app = QApplication(sys.argv)
    app.setFont(QFont(FONT_FAMILY, 11))
    app.setStyleSheet(STYLE_SHEET)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

