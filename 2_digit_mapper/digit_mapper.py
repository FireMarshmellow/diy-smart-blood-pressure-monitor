#!/usr/bin/env python3
"""
Step 2: Digit Mapper.

Drop in the frame saved in step 1, drag the digit boxes onto the digits until
every row reads right, then press "Write into bp_reader.ino" (or copy the code and
paste it over the DIGIT MAP block in 3_reader/bp_reader/bp_reader.ino yourself).

The live readout uses the same maths as the sketch, so if every digit shows the
right number here, the reader sees the same thing.

Needs: Python 3.9+ and PySide6      pip install -r requirements.txt
Run:   python digit_mapper.py [image] [bp_reader.ino or layout.json]

Mouse:  drag a box to move it, drag its edges or corners to resize,
        drag empty space to pan, wheel to zoom.
Keys:   arrows nudge the selected box (Shift = 5px), Ctrl+arrows resize it,
        Tab picks the next box, F fits the image, Ctrl+Z / Ctrl+Y undo / redo.
"""

import copy
import json
import math
import re
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import (QAction, QColor, QFont, QFontDatabase, QGuiApplication,
                           QImage, QKeySequence, QPainter, QPen, QPixmap)
from PySide6.QtWidgets import (QApplication, QCheckBox, QDoubleSpinBox, QFileDialog,
                               QFrame, QGridLayout, QHBoxLayout, QLabel, QMainWindow,
                               QMessageBox, QPlainTextEdit, QPushButton, QScrollArea,
                               QSizePolicy, QSpinBox, QVBoxLayout, QWidget)

APP_NAME = "Digit Mapper"
READER_INO = Path(__file__).resolve().parent.parent / "3_reader" / "bp_reader" / "bp_reader.ino"
BLOCK_BEGIN = "// ---- BEGIN DIGIT MAP"
BLOCK_END = "// ---- END DIGIT MAP ----"
IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".gif", ".tif", ".tiff", ".webp"}
FRAME_W, FRAME_H = 320, 240          # what the reader works on

# ---------------------------------------------------------------------------
# Geometry and decoding: a direct copy of the sketch's logic
# ---------------------------------------------------------------------------

# Sample points inside a digit box, as % of width/height. Order a..g.
SEG_PTS = [(50, 8), (85, 28), (85, 72), (50, 92), (15, 72), (15, 28), (50, 50)]
SEG_HORIZ = [True, False, False, True, False, False, True]
SEG_NAMES = "abcdefg"
HOLE_X = [35, 50, 65]
HOLE_Y = [28, 72]
SEG_HOLE = [0, 0, 1, 1, 1, 0, 2]     # 0 top hole, 1 bottom hole, 2 both

MASK_TO_DIGIT = {0x7E: 0, 0x30: 1, 0x6D: 2, 0x79: 3, 0x33: 4, 0x5B: 5, 0x5F: 6,
                 0x1F: 6, 0x72: 7, 0x70: 7, 0x7F: 8, 0x7B: 9, 0x73: 9}
BLANK, UNKNOWN = -1, -2
SEG_OFF, SEG_ON, SEG_UNSURE = 0, 1, 2


@dataclass
class Box:
    x: int
    y: int
    w: int
    h: int


@dataclass
class Row:
    name: str
    pct: int
    boxes: list


@dataclass
class Settings:
    rotate_180: bool = False
    slant: float = 0.08
    search_px: int = 2
    guard_pct: int = 2
    min_thresh: int = 10
    dark_on_light: bool = True


def default_rows():
    # A starting point that fits docs/example_frame.png. Drag them onto your own display.
    return [
        Row("sys", 25, [Box(131, 32, 29, 52), Box(162, 32, 29, 52), Box(193, 32, 29, 52)]),
        Row("dia", 25, [Box(130, 89, 29, 56), Box(163, 89, 29, 56), Box(196, 89, 29, 56)]),
        Row("pul", 20, [Box(164, 152, 19, 39), Box(186, 152, 19, 39), Box(208, 152, 19, 39)]),
    ]


def f32(v):
    return struct.unpack("f", struct.pack("f", v))[0]


def slant_offset(slant, h, py):
    # (int)lroundf(SLANT * b.h * (50 - py) / 100.0f), in float32 like the ESP32
    v = f32(f32(f32(f32(slant) * h) * (50 - py)) / 100.0)
    r = math.floor(abs(v) + 0.5)
    return int(r if v >= 0 else -r)


class Frame:
    """Greyscale pixels of the loaded image."""

    def __init__(self, w, h, data):
        self.w, self.h, self.data = w, h, data

    @staticmethod
    def from_qimage(img):
        g = img.convertToFormat(QImage.Format.Format_Grayscale8)
        w, h, bpl = g.width(), g.height(), g.bytesPerLine()
        try:
            raw = bytes(g.constBits())
            data = b"".join(raw[y * bpl:y * bpl + w] for y in range(h))
        except Exception:
            data = bytes(g.pixel(x, y) & 0xFF for y in range(h) for x in range(w))
        return Frame(w, h, data)


class Decoder:
    def __init__(self, frame, s):
        self.f, self.s = frame, s

    def ptx(self, b, px, py):
        return b.x + (b.w * px) // 100 + slant_offset(self.s.slant, b.h, py)

    def pty(self, b, py):
        return b.y + (b.h * py) // 100

    def avg3(self, cx, cy):
        total = n = 0
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                x, y = cx + dx, cy + dy
                if 0 <= x < self.f.w and 0 <= y < self.f.h:
                    total += self.f.data[y * self.f.w + x]
                    n += 1
        return total // n if n else 0

    def line5(self, cx, cy, horiz):
        total = n = 0
        for k in range(-2, 3):
            x, y = (cx + k, cy) if horiz else (cx, cy + k)
            if 0 <= x < self.f.w and 0 <= y < self.f.h:
                total += self.f.data[y * self.f.w + x]
                n += 1
        return total // n if n else 0

    def segment_level(self, b, i):
        cx, cy = self.ptx(b, *SEG_PTS[i]), self.pty(b, SEG_PTS[i][1])
        best = None
        for o in range(-self.s.search_px, self.s.search_px + 1):
            v = self.line5(cx, cy + o, True) if SEG_HORIZ[i] else self.line5(cx + o, cy, False)
            if best is None or (v < best if self.s.dark_on_light else v > best):
                best = v
        return best

    def hole_level(self, b, hole):
        best = None
        for px in HOLE_X:
            v = self.avg3(self.ptx(b, px, HOLE_Y[hole]), self.pty(b, HOLE_Y[hole]))
            if best is None or (v > best if self.s.dark_on_light else v < best):
                best = v
        return best

    def classify(self, diff, ref, pct):
        if ref * pct // 100 < self.s.min_thresh:
            return SEG_UNSURE
        if diff * 100 > ref * (pct + self.s.guard_pct):
            return SEG_ON
        if diff * 100 < ref * (pct - self.s.guard_pct):
            return SEG_OFF
        return SEG_UNSURE

    def digit(self, b, pct):
        holes = [self.hole_level(b, 0), self.hole_level(b, 1)]
        holes.append((holes[0] + holes[1]) // 2)
        segs = []
        mask = 0
        unsure = False
        for i in range(7):
            ref = holes[SEG_HOLE[i]]
            lvl = self.segment_level(b, i)
            diff = (ref - lvl) if self.s.dark_on_light else (lvl - ref)
            state = self.classify(diff, ref, pct)
            segs.append({"state": state, "pct": int(diff * 100 / ref) if ref > 0 else 0})   # truncates like the sketch's /debug
            unsure |= state == SEG_UNSURE
            mask = (mask << 1) | (1 if state == SEG_ON else 0)
        if unsure:
            d = UNKNOWN
        elif mask == 0:
            d = BLANK
        else:
            d = MASK_TO_DIGIT.get(mask, UNKNOWN)
        return {"segs": segs, "digit": d, "mask": None if unsure else mask, "bg": holes[2]}

    def row(self, r):
        digits = [self.digit(b, r.pct) for b in r.boxes]
        value, started = 0, False
        for d in digits:
            if d["digit"] == UNKNOWN:
                return digits, None
            if d["digit"] == BLANK:
                if started:
                    return digits, None
                continue
            started = True
            value = value * 10 + d["digit"]
        return digits, (value if started else None)


# ---------------------------------------------------------------------------
# Import / export
# ---------------------------------------------------------------------------

def cbool(v):
    return "true" if v else "false"


def arduino_code(rows, s):
    out = [BLOCK_BEGIN + " (made by the digit mapper, replace the whole block) ----",
           "const bool  CALIBRATED    = true;",
           f"const bool  ROTATE_180    = {cbool(s.rotate_180)};",
           f"const bool  DARK_ON_LIGHT = {cbool(s.dark_on_light)};",
           f"const float SLANT         = {s.slant:.2f};",
           f"const int   SEARCH_PX     = {s.search_px};",
           f"const int   GUARD_PCT     = {s.guard_pct};",
           f"const int   MIN_THRESH    = {s.min_thresh};",
           "",
           "Field fields[] = {"]
    for r in rows:
        boxes = ", ".join("{%3d, %3d, %2d, %2d}" % (b.x, b.y, b.w, b.h) for b in r.boxes)
        out.append(f'  {{"{r.name}", {r.pct}, {len(r.boxes)}, {{{boxes}}}}},')
    out += ["};", BLOCK_END]
    return "\n".join(out)


def replace_block(sketch, code):
    """Swaps the DIGIT MAP block in the sketch text for code. Returns None if there is no block."""
    start = sketch.find(BLOCK_BEGIN)
    end = sketch.find(BLOCK_END, start)
    if start < 0 or end < 0:
        return None
    return sketch[:start] + code + sketch[end + len(BLOCK_END):]


def parse_sketch(text):
    """Reads the fields table and tuning constants out of an .ino file."""
    rows = []
    entry = re.compile(r'\{\s*"(\w+)"\s*,\s*(\d+)\s*,(?:\s*(\d+)\s*,)?\s*\{((?:\s*\{[^{}]*\}\s*,?)+)\s*\}\s*\}')
    box = re.compile(r'\{\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\}')
    table = re.search(r"Field\s+fields\s*\[\s*\]\s*=\s*\{(.*?)\n\s*\}\s*;", text, re.S)
    for m in entry.finditer(table.group(1) if table else ""):
        name, a, b, inner = m.groups()
        pct = int(a) if b is not None else 20       # older sketches had no threshold column
        boxes = [Box(*map(int, bm.groups())) for bm in box.finditer(inner)]
        if boxes:
            rows.append(Row(name, pct, boxes))
    s = Settings()
    for attr, pat, conv in (("slant", r"float\s+SLANT\s*=\s*([-0-9.]+)", float),
                            ("search_px", r"int\s+SEARCH_PX\s*=\s*(\d+)", int),
                            ("guard_pct", r"int\s+GUARD_PCT\s*=\s*(\d+)", int),
                            ("min_thresh", r"int\s+MIN_THRESH\s*=\s*(\d+)", int)):
        m = re.search(pat, text)
        if m:
            setattr(s, attr, conv(m.group(1)))
    m = re.search(r"bool\s+DARK_ON_LIGHT\s*=\s*(true|false)", text)
    if m:
        s.dark_on_light = m.group(1) == "true"
    m = re.search(r"bool\s+ROTATE_180\s*=\s*(true|false)", text)
    if m:
        s.rotate_180 = m.group(1) == "true"
    return rows, s


def layout_to_json(rows, s):
    return json.dumps({
        "settings": s.__dict__,
        "rows": [{"name": r.name, "pct": r.pct, "boxes": [b.__dict__ for b in r.boxes]} for r in rows],
    }, indent=2)


def layout_from_json(text):
    d = json.loads(text)
    s = Settings(**d.get("settings", {}))
    rows = [Row(r["name"], r["pct"], [Box(**b) for b in r["boxes"]]) for r in d["rows"]]
    return rows, s


# ---------------------------------------------------------------------------
# Look
# ---------------------------------------------------------------------------

C_BG = QColor("#111318")
C_PANEL = QColor("#171a21")
C_CARD = QColor("#1e222b")
C_TEXT = QColor("#e7e9ef")
C_MUTED = QColor("#8a92a6")
C_LIT = QColor("#3ee08f")
C_UNLIT = QColor("#5a6378")
C_UNSURE = QColor("#ff8a3d")
ROW_COLOURS = [QColor("#ffb547"), QColor("#4cc9f0"), QColor("#f472b6"),
               QColor("#a78bfa"), QColor("#a3e635")]

STYLE = """
QMainWindow, QWidget#panel { background: #171a21; }
QWidget { color: #e7e9ef; font-size: 13px; }
QScrollArea { border: none; background: #171a21; }
QFrame#card { background: #1e222b; border: 1px solid #2a2f3a; border-radius: 10px; }
QLabel#title { font-size: 18px; font-weight: 600; }
QLabel#muted, QLabel#hint { color: #8a92a6; }
QLabel#section { color: #8a92a6; font-size: 11px; font-weight: 600; letter-spacing: 1px; }
QPushButton { background: #262b36; border: 1px solid #323846; border-radius: 7px; padding: 6px 10px; }
QPushButton:hover { background: #2e3442; border-color: #3f4757; }
QPushButton:pressed { background: #222632; }
QPushButton#primary { background: #ffb547; color: #1a1406; border: none; font-weight: 600; }
QPushButton#primary:hover { background: #ffc46b; }
QSpinBox, QDoubleSpinBox { background: #12151b; border: 1px solid #2f3542; border-radius: 6px; padding: 3px 6px; }
QSpinBox:focus, QDoubleSpinBox:focus { border-color: #ffb547; }
QCheckBox::indicator { width: 15px; height: 15px; }
QPlainTextEdit { background: #0e1015; border: 1px solid #2a2f3a; border-radius: 8px; padding: 6px; }
QPlainTextEdit#code { font-family: Consolas, "Cascadia Mono", "DejaVu Sans Mono", monospace; font-size: 12px; }
QLabel#value { font-family: Consolas, "Cascadia Mono", "DejaVu Sans Mono", monospace; font-size: 26px; font-weight: 700; }
QLabel#segval { font-family: Consolas, "Cascadia Mono", "DejaVu Sans Mono", monospace; font-size: 12px; font-weight: 600; }
QScrollBar:vertical { background: #171a21; width: 10px; margin: 0; }
QScrollBar::handle:vertical { background: #2e3442; border-radius: 5px; min-height: 30px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: none; }
QStatusBar { background: #0f1116; color: #8a92a6; }
"""


def mono_font(size=12, bold=False):
    f = QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont)
    f.setPointSize(size)
    f.setBold(bold)
    return f


def state_colour(state):
    return {SEG_ON: C_LIT, SEG_OFF: C_UNLIT}.get(state, C_UNSURE)


def digit_text(d):
    return "?" if d == UNKNOWN else ("·" if d == BLANK else str(d))


# ---------------------------------------------------------------------------
# Canvas: the image with the boxes on it
# ---------------------------------------------------------------------------

class Canvas(QWidget):
    edited = Signal()            # a box changed
    edit_started = Signal()      # about to change a box (for undo)
    selection_changed = Signal()
    file_dropped = Signal(str)
    view_changed = Signal()

    def __init__(self, win):
        super().__init__()
        self.win = win
        self.pixmap = None
        self.zoom = 1.0
        self.off = QPointF(0, 0)
        self.user_zoomed = False
        self.sel = None          # (row index, box index)
        self.drag = None
        self.setAcceptDrops(True)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setMinimumSize(480, 360)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    # -- coordinates -------------------------------------------------------

    def to_widget(self, x, y):
        return QPointF(self.off.x() + x * self.zoom, self.off.y() + y * self.zoom)

    def to_image(self, p):
        return QPointF((p.x() - self.off.x()) / self.zoom, (p.y() - self.off.y()) / self.zoom)

    def box_rect(self, b):
        tl = self.to_widget(b.x, b.y)
        return QRectF(tl.x(), tl.y(), b.w * self.zoom, b.h * self.zoom)

    def fit(self):
        if not self.pixmap:
            return
        m = 40
        z = min((self.width() - 2 * m) / self.pixmap.width(), (self.height() - 2 * m) / self.pixmap.height())
        self.zoom = max(0.2, z)
        self.off = QPointF((self.width() - self.pixmap.width() * self.zoom) / 2,
                           (self.height() - self.pixmap.height() * self.zoom) / 2)
        self.user_zoomed = False
        self.view_changed.emit()
        self.update()

    def set_image(self, pixmap):
        self.pixmap = pixmap
        self.fit()

    # -- selection ---------------------------------------------------------

    def selected_box(self):
        if self.sel is None:
            return None
        r, i = self.sel
        return self.win.rows[r].boxes[i]

    def select(self, sel):
        if sel != self.sel:
            self.sel = sel
            self.selection_changed.emit()
            self.update()

    def all_boxes(self):
        for r, row in enumerate(self.win.rows):
            for i, _ in enumerate(row.boxes):
                yield (r, i)

    def hit(self, p):
        """Returns ((row, box), mode) under the mouse. mode: 'move' or edge letters l/r/t/b."""
        tol = 6
        order = list(self.all_boxes())
        if self.sel in order:                       # the selected box wins ties
            order.remove(self.sel)
            order.append(self.sel)
        for key in reversed(order):
            rc = self.box_rect(self.win.rows[key[0]].boxes[key[1]])
            if not rc.adjusted(-tol, -tol, tol, tol).contains(p):
                continue
            mode = ""
            if abs(p.y() - rc.top()) <= tol:
                mode += "t"
            elif abs(p.y() - rc.bottom()) <= tol:
                mode += "b"
            if abs(p.x() - rc.left()) <= tol:
                mode += "l"
            elif abs(p.x() - rc.right()) <= tol:
                mode += "r"
            if mode or rc.contains(p):
                return key, (mode or "move")
        return None, None

    @staticmethod
    def cursor_for(mode):
        if mode == "move":
            return Qt.CursorShape.SizeAllCursor
        if mode in ("l", "r"):
            return Qt.CursorShape.SizeHorCursor
        if mode in ("t", "b"):
            return Qt.CursorShape.SizeVerCursor
        if mode in ("tl", "br"):
            return Qt.CursorShape.SizeFDiagCursor
        if mode in ("tr", "bl"):
            return Qt.CursorShape.SizeBDiagCursor
        return Qt.CursorShape.ArrowCursor

    # -- mouse -------------------------------------------------------------

    def mousePressEvent(self, e):
        self.setFocus()
        p = e.position()
        if e.button() == Qt.MouseButton.LeftButton and self.pixmap:
            key, mode = self.hit(p)
            if key is not None:
                self.select(key)
                b = self.selected_box()
                self.drag = {"kind": "box", "mode": mode, "start": self.to_image(p),
                             "orig": Box(b.x, b.y, b.w, b.h), "started": False}
                return
            self.select(None)
        if e.button() in (Qt.MouseButton.LeftButton, Qt.MouseButton.MiddleButton) and self.pixmap:
            self.drag = {"kind": "pan", "last": p}
            self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, e):
        p = e.position()
        if not self.drag:
            key, mode = self.hit(p) if self.pixmap else (None, None)
            self.setCursor(self.cursor_for(mode) if key else Qt.CursorShape.ArrowCursor)
            return
        if self.drag["kind"] == "pan":
            d = p - self.drag["last"]
            self.drag["last"] = p
            self.off += d
            self.user_zoomed = True
            self.update()
            return
        cur = self.to_image(p)
        dx = round(cur.x() - self.drag["start"].x())
        dy = round(cur.y() - self.drag["start"].y())
        o, mode = self.drag["orig"], self.drag["mode"]
        n = Box(o.x, o.y, o.w, o.h)
        if mode == "move":
            n.x, n.y = o.x + dx, o.y + dy
        else:
            if "l" in mode:
                dx = min(dx, o.w - 4)
                n.x, n.w = o.x + dx, o.w - dx
            if "r" in mode:
                n.w = max(4, o.w + dx)
            if "t" in mode:
                dy = min(dy, o.h - 4)
                n.y, n.h = o.y + dy, o.h - dy
            if "b" in mode:
                n.h = max(4, o.h + dy)
        b = self.selected_box()
        if (n.x, n.y, n.w, n.h) != (b.x, b.y, b.w, b.h):
            if not self.drag["started"]:
                self.drag["started"] = True
                self.edit_started.emit()
            b.x, b.y, b.w, b.h = n.x, n.y, n.w, n.h
            self.edited.emit()

    def mouseReleaseEvent(self, e):
        if self.drag and self.drag["kind"] == "pan":
            self.setCursor(Qt.CursorShape.ArrowCursor)
        self.drag = None

    def wheelEvent(self, e):
        if not self.pixmap:
            return
        p = e.position()
        before = self.to_image(p)
        self.zoom = min(40.0, max(0.2, self.zoom * (1.0015 ** e.angleDelta().y())))
        self.off = QPointF(p.x() - before.x() * self.zoom, p.y() - before.y() * self.zoom)
        self.user_zoomed = True
        self.view_changed.emit()
        self.update()

    def resizeEvent(self, e):
        if self.pixmap and not self.user_zoomed:
            self.fit()

    # -- keyboard ----------------------------------------------------------

    def keyPressEvent(self, e):
        k, mods = e.key(), e.modifiers()
        if k == Qt.Key.Key_F:
            self.fit()
            return
        if k == Qt.Key.Key_Tab:
            keys = list(self.all_boxes())
            if keys:
                i = (keys.index(self.sel) + 1) % len(keys) if self.sel in keys else 0
                self.select(keys[i])
            return
        b = self.selected_box()
        arrows = {Qt.Key.Key_Left: (-1, 0), Qt.Key.Key_Right: (1, 0),
                  Qt.Key.Key_Up: (0, -1), Qt.Key.Key_Down: (0, 1)}
        if b is None or k not in arrows:
            super().keyPressEvent(e)
            return
        step = 5 if mods & Qt.KeyboardModifier.ShiftModifier else 1
        dx, dy = arrows[k]
        self.edit_started.emit()
        if mods & Qt.KeyboardModifier.ControlModifier:
            b.w = max(4, b.w + dx * step)
            b.h = max(4, b.h + dy * step)
        else:
            b.x += dx * step
            b.y += dy * step
        self.edited.emit()

    def focusNextPrevChild(self, nxt):
        return False         # keep Tab for cycling boxes

    # -- drag and drop -----------------------------------------------------

    def dragEnterEvent(self, e):
        if e.mimeData().hasUrls():
            e.acceptProposedAction()

    def dropEvent(self, e):
        for url in e.mimeData().urls():
            if url.isLocalFile():
                self.file_dropped.emit(url.toLocalFile())

    # -- drawing -----------------------------------------------------------

    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), C_BG)
        if not self.pixmap:
            self.paint_empty(p)
            return
        p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, False)
        target = QRectF(self.off.x(), self.off.y(), self.pixmap.width() * self.zoom, self.pixmap.height() * self.zoom)
        p.drawPixmap(target, self.pixmap, QRectF(self.pixmap.rect()))
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        dec, results = self.win.decoder, self.win.results
        for r, row in enumerate(self.win.rows):
            colour = ROW_COLOURS[r % len(ROW_COLOURS)]
            res = results[r] if results and r < len(results) else None
            for i, b in enumerate(row.boxes):
                d = res[0][i] if res else None
                self.paint_box(p, dec, b, colour, d, (r, i) == self.sel)
            if res and row.boxes:
                self.paint_row_label(p, row, colour, res[1])

    def paint_empty(self, p):
        rc = QRectF(self.rect()).adjusted(60, 60, -60, -60)
        pen = QPen(C_UNLIT, 2, Qt.PenStyle.DashLine)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        p.setPen(pen)
        p.drawRoundedRect(rc, 18, 18)
        f = self.font()
        f.setPointSize(18)
        f.setBold(True)
        p.setFont(f)
        p.setPen(C_TEXT)
        p.drawText(rc.adjusted(0, -30, 0, -30), Qt.AlignmentFlag.AlignCenter, "Drop a display image here")
        f.setPointSize(12)
        f.setBold(False)
        p.setFont(f)
        p.setPen(C_MUTED)
        p.drawText(rc.adjusted(0, 30, 0, 30), Qt.AlignmentFlag.AlignCenter,
                   "or press Ctrl+O.  Use the frame saved from the step 1 preview.")

    def paint_box(self, p, dec, b, colour, d, selected):
        rc = self.box_rect(b)
        fill = QColor(colour)
        fill.setAlpha(28 if selected else 0)
        p.setBrush(fill)
        p.setPen(QPen(colour, 2.2 if selected else 1.4))
        p.drawRect(rc)
        z = self.zoom
        if dec:
            # search reach across each segment, then the sample dot itself
            if z >= 3:
                pen = QPen(QColor(colour.red(), colour.green(), colour.blue(), 110), max(1.0, z * 0.25))
                p.setPen(pen)
                s = dec.s.search_px
                for i in range(7):
                    sx, sy = dec.ptx(b, *SEG_PTS[i]), dec.pty(b, SEG_PTS[i][1])
                    if SEG_HORIZ[i]:
                        p.drawLine(self.to_widget(sx + 0.5, sy - s + 0.5), self.to_widget(sx + 0.5, sy + s + 0.5))
                    else:
                        p.drawLine(self.to_widget(sx - s + 0.5, sy + 0.5), self.to_widget(sx + s + 0.5, sy + 0.5))
            # hole points (background reference)
            hr = max(1.6, min(4.0, z * 0.3))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(QColor(255, 255, 255, 150), 1))
            for hy in HOLE_Y:
                for hx in HOLE_X:
                    p.drawEllipse(self.to_widget(dec.ptx(b, hx, hy) + 0.5, dec.pty(b, hy) + 0.5), hr, hr)
            r = max(2.6, min(7.0, z * 0.5))
            for i in range(7):
                state = d["segs"][i]["state"] if d else SEG_UNSURE
                c = self.to_widget(dec.ptx(b, *SEG_PTS[i]) + 0.5, dec.pty(b, SEG_PTS[i][1]) + 0.5)
                p.setPen(QPen(QColor(0, 0, 0, 200), 1.2))
                p.setBrush(state_colour(state))
                p.drawEllipse(c, r, r)
        # digit read from this box, above it
        if d is not None:
            txt = digit_text(d["digit"])
            f = mono_font(11, True)
            p.setFont(f)
            tag = QRectF(rc.center().x() - 11, rc.top() - 24, 22, 19)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor(17, 19, 24, 220))
            p.drawRoundedRect(tag, 5, 5)
            p.setPen(C_UNSURE if d["digit"] == UNKNOWN else colour)
            p.drawText(tag, Qt.AlignmentFlag.AlignCenter, txt)
        if selected:
            p.setPen(QPen(QColor(0, 0, 0), 1))
            p.setBrush(colour)
            for pt in (rc.topLeft(), rc.topRight(), rc.bottomLeft(), rc.bottomRight(),
                       QPointF(rc.center().x(), rc.top()), QPointF(rc.center().x(), rc.bottom()),
                       QPointF(rc.left(), rc.center().y()), QPointF(rc.right(), rc.center().y())):
                p.drawRect(QRectF(pt.x() - 3.5, pt.y() - 3.5, 7, 7))

    def paint_row_label(self, p, row, colour, value):
        first = self.box_rect(row.boxes[0])
        txt = f"{row.name.upper()}  {value if value is not None else '?'}"
        p.setFont(mono_font(11, True))
        w = p.fontMetrics().horizontalAdvance(txt) + 18
        tag = QRectF(first.left() - w - 10, first.center().y() - 12, w, 24)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(17, 19, 24, 230))
        p.drawRoundedRect(tag, 12, 12)
        p.setPen(colour if value is not None else C_UNSURE)
        p.drawText(tag, Qt.AlignmentFlag.AlignCenter, txt)


# ---------------------------------------------------------------------------
# Side panel pieces
# ---------------------------------------------------------------------------

def card():
    f = QFrame()
    f.setObjectName("card")
    lay = QVBoxLayout(f)
    lay.setContentsMargins(14, 12, 14, 14)
    lay.setSpacing(8)
    return f, lay


def section(text):
    l = QLabel(text.upper())
    l.setObjectName("section")
    return l


class RowCard(QFrame):
    def __init__(self, win, r):
        super().__init__()
        self.win, self.r = win, r
        self.setObjectName("card")
        colour = ROW_COLOURS[r % len(ROW_COLOURS)].name()
        lay = QGridLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setHorizontalSpacing(10)
        self.name = QLabel()
        self.name.setStyleSheet(f"color:{colour}; font-weight:600; font-size:14px;")
        self.value = QLabel("–")
        self.value.setObjectName("value")
        self.value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.pct = QSpinBox()
        self.pct.setRange(1, 95)
        self.pct.setSuffix(" %")
        self.pct.setToolTip("A segment counts as lit when it is this much darker than the background inside the digit")
        self.pct.valueChanged.connect(self.on_pct)
        match = QPushButton("Match row")
        match.setToolTip("Give this row's boxes the selected box's size and height, spaced evenly between the first and last")
        match.clicked.connect(lambda: win.match_row(self.r))
        lit = QLabel("lit above")
        lit.setObjectName("muted")
        lay.addWidget(self.name, 0, 0)
        lay.addWidget(self.value, 0, 1, 1, 2)
        lay.addWidget(lit, 1, 0)
        lay.addWidget(self.pct, 1, 1)
        lay.addWidget(match, 1, 2)

    def refresh(self, value, ok):
        row = self.win.rows[self.r]
        self.name.setText(row.name.upper())
        self.pct.blockSignals(True)
        self.pct.setValue(row.pct)
        self.pct.blockSignals(False)
        if self.win.frame is None:
            self.value.setText("–")
            self.value.setStyleSheet(f"color:{C_MUTED.name()};")
        else:
            self.value.setText(str(value) if value is not None else "?")
            self.value.setStyleSheet(f"color:{(C_TEXT if ok else C_UNSURE).name()};")

    def on_pct(self, v):
        self.win.push_undo()
        self.win.rows[self.r].pct = v
        self.win.recompute()


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(1360, 820)
        self.rows = default_rows()
        self.s = Settings()
        self.source = None           # the image as loaded, before any rotation
        self.rotated = False
        self.frame = None
        self.decoder = None
        self.results = None
        self.image_path = None
        self.undo, self.redo = [], []

        self.canvas = Canvas(self)
        self.canvas.edited.connect(self.recompute)
        self.canvas.edit_started.connect(self.push_undo)
        self.canvas.selection_changed.connect(self.refresh_selected)
        self.canvas.file_dropped.connect(self.open_path)
        self.canvas.view_changed.connect(self.refresh_status)

        central = QWidget()
        h = QHBoxLayout(central)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(0)
        h.addWidget(self.canvas, 1)
        h.addWidget(self.build_panel())
        self.setCentralWidget(central)
        self.info = QLabel()
        self.statusBar().addPermanentWidget(self.info)
        self.statusBar().showMessage("Drag boxes to move, edges to resize. Arrows nudge (Shift 5px, Ctrl resizes). "
                                     "Wheel zooms, drag empty space to pan, Tab next box, F fit.")
        self.build_actions()
        self.recompute()

    # -- layout ------------------------------------------------------------

    def build_panel(self):
        panel = QWidget()
        panel.setObjectName("panel")
        v = QVBoxLayout(panel)
        v.setContentsMargins(16, 16, 16, 16)
        v.setSpacing(12)

        title = QLabel(APP_NAME)
        title.setObjectName("title")
        sub = QLabel("Line the boxes up with the digits until every row reads right.")
        sub.setObjectName("muted")
        sub.setWordWrap(True)
        v.addWidget(title)
        v.addWidget(sub)

        files = QGridLayout()
        b_img = QPushButton("Open image…")
        b_img.clicked.connect(self.open_image_dialog)
        b_load = QPushButton("Load layout…")
        b_load.setToolTip("Read the boxes from bp_reader.ino or a saved layout")
        b_load.clicked.connect(self.open_layout_dialog)
        b_save = QPushButton("Save layout…")
        b_save.clicked.connect(self.save_layout_dialog)
        b_fit = QPushButton("Fit view")
        b_fit.clicked.connect(self.canvas.fit)
        files.addWidget(b_img, 0, 0)
        files.addWidget(b_load, 0, 1)
        files.addWidget(b_save, 1, 0)
        files.addWidget(b_fit, 1, 1)
        v.addLayout(files)

        v.addWidget(section("Readout"))
        self.row_cards = []
        for r in range(len(self.rows)):
            rc = RowCard(self, r)
            self.row_cards.append(rc)
            v.addWidget(rc)

        v.addWidget(section("Selected digit"))
        sel_card, sl = card()
        self.sel_title = QLabel("Click a box")
        self.sel_title.setObjectName("muted")
        sl.addWidget(self.sel_title)
        grid = QGridLayout()
        self.spins = {}
        for col, name in enumerate("xywh"):
            lab = QLabel(name)
            lab.setObjectName("muted")
            sp = QSpinBox()
            sp.setRange(-50, 2000)
            sp.valueChanged.connect(lambda val, n=name: self.on_spin(n, val))
            grid.addWidget(lab, 0, col)
            grid.addWidget(sp, 1, col)
            self.spins[name] = sp
        sl.addLayout(grid)
        self.seg_grid = QGridLayout()
        self.seg_grid.setHorizontalSpacing(4)
        self.seg_labels = []
        for i, ch in enumerate(SEG_NAMES):
            top = QLabel(ch)
            top.setObjectName("muted")
            top.setAlignment(Qt.AlignmentFlag.AlignCenter)
            val = QLabel("–")
            val.setObjectName("segval")
            val.setAlignment(Qt.AlignmentFlag.AlignCenter)
            val.setMinimumWidth(34)
            self.seg_grid.addWidget(top, 0, i)
            self.seg_grid.addWidget(val, 1, i)
            self.seg_labels.append(val)
        sl.addLayout(self.seg_grid)
        legend = QLabel(f"<span style='color:{C_LIT.name()}'>●</span> lit &nbsp; "
                        f"<span style='color:{C_UNLIT.name()}'>●</span> unlit &nbsp; "
                        f"<span style='color:{C_UNSURE.name()}'>●</span> too close to call &nbsp; "
                        "<span style='color:#cfd3dc'>○</span> background")
        legend.setObjectName("hint")
        sl.addWidget(legend)
        v.addWidget(sel_card)

        v.addWidget(section("Tuning"))
        tune, tl = card()
        g = QGridLayout()
        self.slant = QDoubleSpinBox()
        self.slant.setRange(-0.4, 0.4)
        self.slant.setSingleStep(0.01)
        self.slant.setDecimals(2)
        self.slant.setToolTip("How far the digits lean: top offset to the right, as a fraction of height")
        self.search = QSpinBox()
        self.search.setRange(0, 4)
        self.search.setSuffix(" px")
        self.search.setToolTip("How far either side each segment is looked for")
        self.guard = QSpinBox()
        self.guard.setRange(0, 15)
        self.guard.setSuffix(" %")
        self.guard.setToolTip("Segments this close to the threshold count as unsure, and the frame is skipped")
        self.minth = QSpinBox()
        self.minth.setRange(0, 80)
        self.minth.setToolTip("Frames too dark for a threshold of at least this many grey levels are skipped")
        self.led = QCheckBox("Light segments on a dark display (LED)")
        self.rot = QCheckBox("Rotate 180° (camera is mounted upside down)")
        for i, (lab, w) in enumerate((("Slant", self.slant), ("Search", self.search),
                                       ("Guard", self.guard), ("Min contrast", self.minth))):
            l = QLabel(lab)
            l.setObjectName("muted")
            g.addWidget(l, i // 2 * 2, i % 2)
            g.addWidget(w, i // 2 * 2 + 1, i % 2)
        tl.addWidget(self.rot)
        tl.addLayout(g)
        tl.addWidget(self.led)
        for w in (self.slant, self.search, self.guard, self.minth):
            w.valueChanged.connect(self.on_settings)
        self.led.toggled.connect(self.on_settings)
        self.rot.toggled.connect(self.on_settings)
        v.addWidget(tune)

        v.addWidget(section("Arduino code"))
        self.code = QPlainTextEdit()
        self.code.setObjectName("code")
        self.code.setReadOnly(True)
        self.code.setMinimumHeight(170)
        self.code.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        v.addWidget(self.code)
        copy_btn = QPushButton("Copy Arduino code")
        copy_btn.setObjectName("primary")
        copy_btn.clicked.connect(self.copy_code)
        write_btn = QPushButton("Write into bp_reader.ino")
        write_btn.setToolTip(f"Replace the DIGIT MAP block in {READER_INO}")
        write_btn.clicked.connect(self.write_reader)
        v.addWidget(write_btn)
        v.addWidget(copy_btn)
        v.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidget(panel)
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(400)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.sync_settings_widgets()
        return scroll

    def build_actions(self):
        for text, key, fn in (("Open image", QKeySequence.StandardKey.Open, self.open_image_dialog),
                              ("Load layout", "Ctrl+L", self.open_layout_dialog),
                              ("Save layout", QKeySequence.StandardKey.Save, self.save_layout_dialog),
                              ("Copy code", "Ctrl+Shift+C", self.copy_code),
                              ("Undo", QKeySequence.StandardKey.Undo, self.do_undo),
                              ("Redo", QKeySequence.StandardKey.Redo, self.do_redo)):
            a = QAction(text, self)
            a.setShortcut(QKeySequence(key))
            a.triggered.connect(fn)
            self.addAction(a)
        a = QAction("Redo", self)
        a.setShortcut(QKeySequence("Ctrl+Y"))
        a.triggered.connect(self.do_redo)
        self.addAction(a)

    # -- state -------------------------------------------------------------

    def snapshot(self):
        return (copy.deepcopy(self.rows), copy.deepcopy(self.s))

    def push_undo(self):
        self.undo.append(self.snapshot())
        self.undo = self.undo[-200:]
        self.redo.clear()

    def restore(self, snap):
        self.rows, self.s = copy.deepcopy(snap[0]), copy.deepcopy(snap[1])
        if self.canvas.sel and (self.canvas.sel[0] >= len(self.rows) or
                                self.canvas.sel[1] >= len(self.rows[self.canvas.sel[0]].boxes)):
            self.canvas.sel = None
        self.sync_settings_widgets()
        self.recompute()

    def do_undo(self):
        if self.undo:
            self.redo.append(self.snapshot())
            self.restore(self.undo.pop())

    def do_redo(self):
        if self.redo:
            self.undo.append(self.snapshot())
            self.restore(self.redo.pop())

    def apply_rotation(self):
        if self.source is None or (self.frame and self.rotated == self.s.rotate_180):
            return
        img = self.source.mirrored(True, True) if self.s.rotate_180 else self.source
        self.rotated = self.s.rotate_180
        self.frame = Frame.from_qimage(img)
        self.canvas.set_image(QPixmap.fromImage(img.convertToFormat(QImage.Format.Format_Grayscale8)))

    def recompute(self):
        self.apply_rotation()
        if self.frame:
            self.decoder = Decoder(self.frame, self.s)
            self.results = [self.decoder.row(r) for r in self.rows]
        else:
            self.decoder, self.results = None, None
        for r, rc in enumerate(self.row_cards):
            if r < len(self.rows):
                val = self.results[r][1] if self.results else None
                rc.refresh(val, self.results is not None and val is not None)
                rc.setVisible(True)
            else:
                rc.setVisible(False)
        self.code.setPlainText(arduino_code(self.rows, self.s))
        self.refresh_selected()
        self.refresh_status()
        self.canvas.update()

    def refresh_selected(self):
        b = self.canvas.selected_box()
        for sp in self.spins.values():
            sp.blockSignals(True)
            sp.setEnabled(b is not None)
        if b is None:
            self.sel_title.setText("Click a box on the image")
            for lab in self.seg_labels:
                lab.setText("–")
                lab.setStyleSheet("")
        else:
            r, i = self.canvas.sel
            for n in "xywh":
                self.spins[n].setValue(getattr(b, n))
            place = ["hundreds", "tens", "units"][i] if len(self.rows[r].boxes) == 3 else f"digit {i + 1}"
            text = f"{self.rows[r].name.upper()} {place}"
            if self.results:
                d = self.results[r][0][i]
                mask = "unsure" if d["mask"] is None else format(d["mask"], "07b")
                text += f"  reads {digit_text(d['digit'])}  ({mask}, background {d['bg']})"
                for lab, sg in zip(self.seg_labels, d["segs"]):
                    lab.setText(f"{sg['pct']}%")
                    lab.setStyleSheet(f"color:{state_colour(sg['state']).name()};")
            self.sel_title.setText(text)
        for sp in self.spins.values():
            sp.blockSignals(False)

    def refresh_status(self):
        if self.frame:
            name = Path(self.image_path).name if self.image_path else "image"
            size = f"{self.frame.w}×{self.frame.h}"
            warn = "" if (self.frame.w, self.frame.h) == (FRAME_W, FRAME_H) else \
                f"   ⚠ the reader works on {FRAME_W}×{FRAME_H} frames, use one saved from the preview"
            self.info.setText(f"{name}  {size}   zoom {self.canvas.zoom * 100:.0f}%{warn}")
        else:
            self.info.setText("no image")

    def sync_settings_widgets(self):
        widgets = (self.slant, self.search, self.guard, self.minth, self.led, self.rot)
        for w in widgets:
            w.blockSignals(True)
        self.slant.setValue(self.s.slant)
        self.search.setValue(self.s.search_px)
        self.guard.setValue(self.s.guard_pct)
        self.minth.setValue(self.s.min_thresh)
        self.led.setChecked(not self.s.dark_on_light)
        self.rot.setChecked(self.s.rotate_180)
        for w in widgets:
            w.blockSignals(False)

    # -- edits -------------------------------------------------------------

    def on_spin(self, name, val):
        b = self.canvas.selected_box()
        if b is None or getattr(b, name) == val:
            return
        self.push_undo()
        if name in "wh":
            val = max(4, val)
        setattr(b, name, val)
        self.recompute()

    def on_settings(self, *_):
        self.push_undo()
        self.s.slant = round(self.slant.value(), 2)
        self.s.search_px = self.search.value()
        self.s.guard_pct = self.guard.value()
        self.s.min_thresh = self.minth.value()
        self.s.dark_on_light = not self.led.isChecked()
        self.s.rotate_180 = self.rot.isChecked()
        self.recompute()

    def match_row(self, r):
        boxes = self.rows[r].boxes
        if len(boxes) < 2:
            return
        sel = self.canvas.sel
        ref = boxes[sel[1]] if sel and sel[0] == r else boxes[0]
        self.push_undo()
        first, last = boxes[0].x, boxes[-1].x
        for i, b in enumerate(boxes):
            b.y, b.w, b.h = ref.y, ref.w, ref.h
            b.x = round(first + (last - first) * i / (len(boxes) - 1))
        self.recompute()

    # -- files -------------------------------------------------------------

    def open_path(self, path):
        ext = Path(path).suffix.lower()
        if ext in IMAGE_EXTS:
            self.load_image(path)
        elif ext in (".ino", ".h", ".cpp", ".json", ".txt"):
            self.load_layout(path)
        else:
            QMessageBox.warning(self, APP_NAME, f"Don't know what to do with {Path(path).name}.\n"
                                                "Drop an image, a .ino sketch or a saved .json layout.")

    def load_image(self, path):
        img = QImage(path)
        if img.isNull():
            QMessageBox.warning(self, APP_NAME, f"Could not open {path}")
            return
        self.image_path = path
        self.source = img
        self.frame = None
        self.setWindowTitle(f"{APP_NAME} – {Path(path).name}")
        self.recompute()

    def load_layout(self, path):
        try:
            text = Path(path).read_text(encoding="utf-8", errors="replace")
            if Path(path).suffix.lower() == ".json":
                rows, s = layout_from_json(text)
            else:
                rows, s = parse_sketch(text)
        except Exception as ex:
            QMessageBox.warning(self, APP_NAME, f"Could not read the layout from {Path(path).name}:\n{ex}")
            return
        if not rows:
            QMessageBox.warning(self, APP_NAME, f"No fields table found in {Path(path).name}.")
            return
        self.push_undo()
        self.rows, self.s = rows, s
        self.canvas.sel = None
        self.rebuild_row_cards()
        self.sync_settings_widgets()
        self.recompute()
        self.statusBar().showMessage(f"Loaded {len(rows)} rows from {Path(path).name}", 5000)

    def rebuild_row_cards(self):
        # Row cards are made once for the default three rows; add more if a layout has more.
        lay = self.row_cards[0].parentWidget().layout()
        while len(self.row_cards) < len(self.rows):
            rc = RowCard(self, len(self.row_cards))
            lay.insertWidget(lay.indexOf(self.row_cards[-1]) + 1, rc)
            self.row_cards.append(rc)

    def open_image_dialog(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open display image", "",
                                              "Images (*.png *.jpg *.jpeg *.bmp *.gif *.tif *.tiff *.webp)")
        if path:
            self.load_image(path)

    def open_layout_dialog(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load layout", "",
                                              "Sketch or layout (*.ino *.json);;All files (*)")
        if path:
            self.load_layout(path)

    def save_layout_dialog(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save layout", "layout.json", "Layout (*.json)")
        if path:
            Path(path).write_text(layout_to_json(self.rows, self.s), encoding="utf-8")
            self.statusBar().showMessage(f"Saved {Path(path).name}", 5000)

    def write_reader(self):
        path = READER_INO
        if not path.exists():
            picked, _ = QFileDialog.getOpenFileName(self, "Find bp_reader.ino", "", "Sketch (*.ino)")
            if not picked:
                return
            path = Path(picked)
        new = replace_block(path.read_text(encoding="utf-8"), self.code.toPlainText())
        if new is None:
            QMessageBox.warning(self, APP_NAME, f"No DIGIT MAP block found in {path.name}.\n"
                                                "Copy the code and paste it in by hand instead.")
            return
        path.write_text(new, encoding="utf-8", newline="")
        self.statusBar().showMessage(f"Written into {path}. Now upload the sketch to the board.", 8000)

    def copy_code(self):
        QGuiApplication.clipboard().setText(self.code.toPlainText())
        self.statusBar().showMessage("Arduino code copied. Paste it over the DIGIT MAP block in bp_reader.ino.", 5000)


def main():
    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    w = Window()
    for arg in sys.argv[1:]:
        if Path(arg).exists():
            w.open_path(arg)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
