#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Full-screen touch targets for measuring Fermi accuracy (PyQt6).

    gui.py OUTDIR [--screen NAME] [--fast]

Shows a sequence of targets: single-finger holds over a grid, along the edges
and near the cameras, slow drags along straight lines, and two-finger holds.
Everything is logged to OUTDIR/targets.jsonl with time.time() timestamps, the
same clock record.py uses for the raw stream, so each hold can be matched to
the camera frames recorded while it was held.

Target positions are fractions of the window, which is full screen; the
touchscreen covers the whole panel, so a target at (fx, fy) should be reported
at (fx, fy) * 32767.

Steps advance on the touches the current driver reports.  Where it reports
nothing (or the wrong number of fingers), a hold falls back to a timed window
after a few seconds; Space forces that at once.  Backspace redoes the previous
step, Esc quits.
"""
import argparse, json, math, os, sys, time

from PyQt6.QtCore import QEvent, QPointF, QRectF, Qt, QTimer
from PyQt6.QtGui import QColor, QEventPoint, QFont, QPainter, QPen
from PyQt6.QtWidgets import QApplication, QWidget

HOLD_S = 1.5        # steady hold that completes a target
TIMEOUT_S = 6.0     # no (or too few) touches reported: fall back to timed
TIMED_S = 2.5       # length of a timed hold
LIFT_S = 0.3        # all fingers up for this long before the next step
TIMED_LIFT_S = 1.0  # after a timed hold, nothing tells us the finger lifted
READY_S = 0.6       # drag: pause on the start point
DRAG_S = 4.0        # drag duration
IDLE_S = 4.0        # untouched period, for the shadow baseline

BG = QColor('#101418')
FG = QColor('#e8e8e8')
DIM = QColor('#5a6470')
GO = QColor('#3ec46d')
WARN = QColor('#f0a830')
BAD = QColor('#e04848')


def build_steps():
    s = [('idle', []), ('hold', [(0.5, 0.5)])]
    xs = [0.1, 0.3, 0.5, 0.7, 0.9]
    for j, y in enumerate([0.12, 0.37, 0.63, 0.88]):
        s += [('hold', [(x, y)]) for x in (xs if j % 2 == 0 else xs[::-1])]
    e = 0.04
    for p in [(e, e), (0.5, e), (1 - e, e), (1 - e, 0.5), (1 - e, 1 - e),
              (0.5, 1 - e), (e, 1 - e), (e, 0.5)]:
        s.append(('hold', [p]))
    # close to the cameras (top corners), where shadows run off the sensor
    for p in [(0.15, 0.04), (0.85, 0.04), (0.04, 0.15), (0.96, 0.15)]:
        s.append(('hold', [p]))
    for a, b in [((0.06, 0.5), (0.94, 0.5)), ((0.94, 0.1), (0.06, 0.1)),
                 ((0.06, 0.9), (0.94, 0.9)), ((0.1, 0.06), (0.1, 0.94)),
                 ((0.5, 0.94), (0.5, 0.06)), ((0.9, 0.06), (0.9, 0.94)),
                 ((0.06, 0.06), (0.94, 0.94)), ((0.94, 0.06), (0.06, 0.94))]:
        s.append(('drag', [a, b]))
    for a, b in [((0.3, 0.5), (0.7, 0.5)), ((0.5, 0.25), (0.5, 0.75)),
                 ((0.25, 0.25), (0.75, 0.75)), ((0.75, 0.25), (0.25, 0.75)),
                 ((0.42, 0.5), (0.58, 0.5)), ((0.15, 0.15), (0.85, 0.15))]:
        s.append(('hold', [a, b]))
    s += [('hold', [(0.5, 0.5)]), ('idle', [])]   # drift check
    return s


class Capture(QWidget):
    def __init__(self, out, screen_name, fast):
        super().__init__()
        self.out = out
        self.screen_name = screen_name
        self.speed = 4.0 if fast else 1.0
        self.log = open(os.path.join(out, 'targets.jsonl'), 'a')
        self.steps = build_steps()
        self.i = -1
        self.touch = []
        self.done_at = None
        self.stream_size = -1
        self.stream_t = time.time()
        self.stream_ok = False
        self.setAttribute(Qt.WidgetAttribute.WA_AcceptTouchEvents)
        self.setCursor(Qt.CursorShape.BlankCursor)
        self.setWindowTitle('Fermi target capture')
        self.timer = QTimer(self)
        self.timer.setInterval(15)
        self.timer.timeout.connect(self.tick)

    # -- logging ---------------------------------------------------------
    def emit(self, **kw):
        kw = {'t': time.time(), **kw}
        self.log.write(json.dumps(kw) + '\n')
        self.log.flush()

    def start(self):
        scr = self.screen()
        g, ps = scr.geometry(), scr.physicalSize()
        self.emit(ev='start', screen=dict(
            name=scr.name(), w=g.width(), h=g.height(),
            mm_w=ps.width(), mm_h=ps.height(), dpr=scr.devicePixelRatio()),
            window=dict(w=self.width(), h=self.height()),
            screens=[s.name() for s in QApplication.screens()],
            speed=self.speed)
        self.goto(0)
        self.timer.start()

    def resizeEvent(self, ev):
        if self.i >= 0:
            self.emit(ev='resize', w=self.width(), h=self.height())
        super().resizeEvent(ev)

    # -- state machine ---------------------------------------------------
    def d(self, seconds):
        return seconds / self.speed

    def goto(self, i):
        self.i = i
        if i >= len(self.steps):
            self.emit(ev='end')
            self.done_at = time.time()
            return
        kind, pts = self.steps[i]
        self.t_show = time.time()
        self.set_phase('wait')
        self.timed = False
        self.emit(ev='step', i=i, kind=kind, pts=pts)

    def set_phase(self, phase):
        self.phase = phase
        self.t_phase = time.time()

    def check_stream(self, now):
        if now - self.stream_t < 1.0:
            return
        try:
            size = os.path.getsize(os.path.join(self.out, 'stream.bin'))
        except OSError:
            size = -1
        if size != self.stream_size:
            self.stream_size = size
            self.stream_ok = size > 0
            self.stream_last_change = now
        elif now - getattr(self, 'stream_last_change', now) > 3.0:
            self.stream_ok = False
        self.stream_t = now

    def tick(self):
        now = time.time()
        self.check_stream(now)
        if self.done_at is not None:
            if now - self.done_at > 1.5:
                self.close()
            self.update()
            return

        kind, pts = self.steps[self.i]
        n = len(self.touch)
        el = now - self.t_phase

        if kind == 'idle':
            if n:
                self.set_phase('wait')      # restart the untouched period
            elif el >= self.d(IDLE_S):
                self.emit(ev='idle', i=self.i, t0=self.t_phase, t1=now)
                self.goto(self.i + 1)
        elif self.phase == 'lift':
            if n and not self.timed:
                self.t_phase = now
            elif el >= self.d(TIMED_LIFT_S if self.timed else LIFT_S):
                self.goto(self.i + 1)
        elif kind == 'hold':
            need = len(pts)
            if self.phase == 'wait':
                if n >= need:
                    self.set_phase('hold')
                elif now - self.t_show >= self.d(TIMEOUT_S):
                    self.start_timed()
            elif self.phase == 'hold':
                if n < need:
                    self.emit(ev='interrupted', i=self.i, after=el)
                    self.set_phase('wait')
                elif el >= self.d(HOLD_S):
                    self.emit(ev='hold', i=self.i, t0=self.t_phase, t1=now,
                              pts=pts, timed=False)
                    self.set_phase('lift')
            elif self.phase == 'timed':
                if el >= self.d(TIMED_S):
                    self.emit(ev='hold', i=self.i, t0=self.t_phase, t1=now,
                              pts=pts, timed=True)
                    self.set_phase('lift')
        elif kind == 'drag':
            if self.phase == 'wait':
                if n or now - self.t_show >= self.d(TIMEOUT_S):
                    self.timed = not n
                    self.set_phase('ready')
            elif self.phase == 'ready':
                if el >= self.d(READY_S):
                    self.set_phase('move')
            elif self.phase == 'move':
                if el >= self.d(DRAG_S):
                    self.emit(ev='drag', i=self.i, t0=self.t_phase, t1=now,
                              a=pts[0], b=pts[1], timed=self.timed)
                    self.set_phase('lift')
        self.update()

    def start_timed(self):
        self.timed = True
        self.emit(ev='timed', i=self.i, touches=len(self.touch))
        self.set_phase('timed')

    # -- input -----------------------------------------------------------
    def event(self, ev):
        t = ev.type()
        if t in (QEvent.Type.TouchBegin, QEvent.Type.TouchUpdate,
                 QEvent.Type.TouchEnd, QEvent.Type.TouchCancel):
            w, h = max(self.width(), 1), max(self.height(), 1)
            if t in (QEvent.Type.TouchEnd, QEvent.Type.TouchCancel):
                pts = []
            else:
                pts = [(p.id(), p.position().x() / w, p.position().y() / h)
                       for p in ev.points()
                       if p.state() != QEventPoint.State.Released]
            if pts != self.touch:
                self.emit(ev='qt', pts=[[k, round(x, 5), round(y, 5)]
                                        for k, x, y in pts])
            self.touch = pts
            ev.accept()
            return True
        return super().event(ev)

    def keyPressEvent(self, ev):
        k = ev.key()
        if k == Qt.Key.Key_Escape:
            self.emit(ev='abort', i=self.i)
            self.close()
        elif self.done_at is not None:
            return
        elif k == Qt.Key.Key_Space:
            kind, _ = self.steps[self.i]
            if kind == 'hold' and self.phase in ('wait', 'hold'):
                self.start_timed()
            elif kind == 'drag' and self.phase == 'wait':
                self.timed = True
                self.set_phase('ready')
        elif k == Qt.Key.Key_Backspace:
            j = self.i - 1
            while j > 0 and self.steps[j][0] == 'idle':
                j -= 1
            if j >= 0:
                self.emit(ev='redo', i=j, from_i=self.i)
                self.goto(j)

    def closeEvent(self, ev):
        self.timer.stop()
        self.log.close()
        super().closeEvent(ev)

    # -- drawing ---------------------------------------------------------
    def px(self, p):
        return QPointF(p[0] * self.width(), p[1] * self.height())

    def drag_pos(self, a, b):
        if self.phase in ('wait', 'ready'):
            f = 0.0
        elif self.phase == 'move':
            f = min((time.time() - self.t_phase) / self.d(DRAG_S), 1.0)
        else:
            f = 1.0
        return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)

    def text_spot(self, pts):
        best, where = -1, 0.5
        for y in (0.5, 0.3, 0.7, 0.2, 0.8):
            dmin = min([math.hypot(px - 0.5, py - y) for px, py in pts] or [1])
            if dmin > 0.15:
                return y
            if dmin > best:
                best, where = dmin, y
        return where

    def paintEvent(self, ev):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        qp.fillRect(self.rect(), BG)
        w, h = self.width(), self.height()
        r = max(18.0, 0.03 * min(w, h))
        now = time.time()

        font = QFont()
        font.setPixelSize(max(16, int(h * 0.035)))
        qp.setFont(font)

        if self.i < 0:
            return
        if self.done_at is not None:
            qp.setPen(GO)
            qp.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, 'Done. Thank you!')
            return

        kind, pts = self.steps[self.i]
        el = now - self.t_phase
        avoid = list(pts)
        msg = ''

        if kind == 'idle':
            msg = "Don't touch the screen"
            frac = min(el / self.d(IDLE_S), 1.0)
            qp.setPen(QPen(DIM, 6))
            qp.drawLine(QPointF(w * 0.3, h * 0.6), QPointF(w * 0.7, h * 0.6))
            qp.setPen(QPen(GO, 6))
            qp.drawLine(QPointF(w * 0.3, h * 0.6),
                        QPointF(w * (0.3 + 0.4 * frac), h * 0.6))
        elif kind == 'hold':
            if self.phase == 'hold':
                frac, col = el / self.d(HOLD_S), GO
                msg = 'Hold still…'
            elif self.phase == 'timed':
                frac, col = el / self.d(TIMED_S), WARN
                msg = 'Touch not detected: keep holding'
            elif self.phase == 'lift':
                frac, col = 1.0, GO
                msg = 'Lift'
            else:
                frac, col = 0.0, FG
                msg = ('Touch and hold the centre of the cross' if len(pts) == 1
                       else 'Hold one finger on each cross')
            for p in pts:
                self.draw_target(qp, self.px(p), r, frac, col)
        elif kind == 'drag':
            a, b = pts
            qp.setPen(QPen(DIM, 2, Qt.PenStyle.DashLine))
            qp.drawLine(self.px(a), self.px(b))
            pos = self.drag_pos(a, b)
            col = GO if self.phase == 'lift' else (WARN if self.timed else FG)
            self.draw_target(qp, self.px(pos), r, 1.0 if self.phase == 'lift' else 0.0, col)
            avoid = [a, b, ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)]
            msg = {'wait': 'Put a finger on the cross', 'ready': 'Follow the cross',
                   'move': 'Follow the cross', 'lift': 'Lift'}[self.phase]

        if msg:
            qp.setPen(FG)
            ty = self.text_spot(avoid) * h
            qp.drawText(QRectF(0, ty - h * 0.05, w, h * 0.1),
                        Qt.AlignmentFlag.AlignCenter, msg)

        small = QFont()
        small.setPixelSize(max(12, int(h * 0.018)))
        qp.setFont(small)
        qp.setPen(DIM)
        qp.drawText(QRectF(0, h * 0.955, w, h * 0.04), Qt.AlignmentFlag.AlignCenter,
                    f'{self.i + 1}/{len(self.steps)}   touches: {len(self.touch)}   '
                    'Esc quit · Space: touch not detected · Backspace: redo previous')
        if not self.stream_ok:
            qp.setPen(BAD)
            qp.drawText(QRectF(0, h * 0.01, w, h * 0.04), Qt.AlignmentFlag.AlignCenter,
                        'Recorder is not receiving data (stream.bin not growing)')

    def draw_target(self, qp, c, r, frac, col):
        qp.setPen(QPen(FG, 2))
        qp.drawLine(QPointF(c.x() - 1.7 * r, c.y()), QPointF(c.x() + 1.7 * r, c.y()))
        qp.drawLine(QPointF(c.x(), c.y() - 1.7 * r), QPointF(c.x(), c.y() + 1.7 * r))
        qp.setPen(QPen(DIM, 3))
        qp.drawEllipse(c, r, r)
        if frac > 0:
            qp.setPen(QPen(col, 5))
            rect = QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r)
            qp.drawArc(rect, 90 * 16, -int(min(frac, 1.0) * 360 * 16))
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(col)
        qp.drawEllipse(c, 3, 3)
        qp.setBrush(Qt.BrushStyle.NoBrush)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('out')
    ap.add_argument('--screen', help='output name (default: primary)')
    ap.add_argument('--fast', action='store_true', help='4x shorter timings, for testing')
    args = ap.parse_args()

    app = QApplication(sys.argv[:1])
    scr = app.primaryScreen()
    if args.screen:
        match = [s for s in app.screens() if s.name() == args.screen]
        if not match:
            sys.exit(f'no screen {args.screen!r}; have: '
                     + ', '.join(s.name() for s in app.screens()))
        scr = match[0]

    win = Capture(args.out, args.screen, args.fast)
    win.setScreen(scr)
    win.setGeometry(scr.geometry())
    win.showFullScreen()
    QTimer.singleShot(500, win.start)   # let the fullscreen size settle
    app.exec()


if __name__ == '__main__':
    main()
