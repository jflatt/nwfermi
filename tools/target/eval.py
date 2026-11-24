#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Score the touch engine against a target capture.

    eval.py OUTDIR [--engine FILE] [--verbose]

Replays OUTDIR/stream.bin through tools/engine/replay (built if missing) and
compares what it reports during each hold and drag with the target positions
logged by gui.py.  Errors are in millimetres when the screen reported its
physical size, otherwise in % of the axis.
"""
import argparse, bisect, json, os, subprocess, sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MAX = 32767
SETTLE_S = 0.2


def load_log(path):
    ev = [json.loads(l) for l in open(path)]
    start = next((e for e in ev if e['ev'] == 'start'), None)
    if not start:
        sys.exit('no start event in targets.jsonl')
    holds, drags = {}, {}
    for e in ev:
        if e['ev'] == 'hold':
            holds[e['i']] = e       # a redo replaces the earlier attempt
        elif e['ev'] == 'drag':
            drags[e['i']] = e
    return start, holds, drags


def run_engine(stream, out):
    exe = os.path.join(ROOT, 'tools/engine/replay')
    subprocess.run(['make', '-s', '-C', ROOT, 'replay'], check=True)
    with open(out, 'w') as f:
        r = subprocess.run([exe, stream], stdout=f, stderr=subprocess.PIPE, text=True)
    sys.stderr.write(r.stderr)
    return out


def load_engine(path):
    T, P = [], []
    for l in open(path):
        f = l.split()
        n = int(f[1])
        T.append(float(f[0]))
        P.append([(int(f[3 + 3 * k]), int(f[4 + 3 * k])) for k in range(n)])
    return T, P


def frames(T, P, t0, t1):
    a, b = bisect.bisect_left(T, t0), bisect.bisect_right(T, t1)
    return T[a:b], P[a:b]


def region(p):
    x, y = p
    if max(abs(x - 0.5), abs(y - 0.5)) <= 0.42:
        return 'interior'
    if y < 0.2 and (x < 0.2 or x > 0.8):
        return 'near camera'
    return 'edge'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('out')
    ap.add_argument('--engine', help='use this replay output instead of running the engine')
    ap.add_argument('--verbose', '-v', action='store_true')
    args = ap.parse_args()

    start, holds, drags = load_log(os.path.join(args.out, 'targets.jsonl'))
    eng = args.engine or run_engine(os.path.join(args.out, 'stream.bin'),
                                    os.path.join(args.out, 'engine.txt'))
    T, P = load_engine(eng)

    s = start['screen']
    if s.get('mm_w') and s.get('mm_h'):
        sx, sy, unit = s['mm_w'] / MAX, s['mm_h'] / MAX, 'mm'
    else:
        sx = sy = 100.0 / MAX
        unit = '%'
    print(f"screen {s['name']} {s['w']}x{s['h']} px, {s.get('mm_w')}x{s.get('mm_h')} mm; "
          f"window {start['window']['w']}x{start['window']['h']}")
    if (start['window']['w'], start['window']['h']) != (s['w'], s['h']):
        print('WARNING: window was not the full screen; target positions are off')

    rows, by_region = [], {}
    print(f'\n{"step":>4} {"target":>13} {"kind":5} {"frames":>6} {"miss":>5} '
          f'{"dx":>6} {"dy":>6} {"err":>6} {"spread":>6}  ({unit})')
    for i, h in sorted(holds.items()):
        pts = [tuple(p) for p in h['pts']]
        t0 = h['t0'] + (0 if h['timed'] else SETTLE_S)
        ft, fp = frames(T, P, t0, h['t1'])
        need = len(pts)
        good = [q for q in fp if len(q) == need]
        miss = 1 - len(good) / len(fp) if fp else 1.0
        tgt = np.array([(x * MAX, y * MAX) for x, y in pts])
        if not good:
            print(f'{i:4d} {str(pts[0]):>13} {"timed" if h["timed"] else "":5} '
                  f'{len(fp):6d} {miss:5.0%}  no frames with {need} contact(s)')
            for p in pts:
                by_region.setdefault(region(p), []).append(np.nan)
            continue
        # assign reported contacts to targets by the cheaper pairing
        arr = []
        for q in good:
            q = np.array(q, float)
            if need == 2 and (np.linalg.norm(q[0] - tgt[1]) + np.linalg.norm(q[1] - tgt[0]) <
                              np.linalg.norm(q[0] - tgt[0]) + np.linalg.norm(q[1] - tgt[1])):
                q = q[::-1]
            arr.append(q)
        arr = np.array(arr)                     # frames x need x 2
        med = np.median(arr, axis=0)
        for k, p in enumerate(pts):
            dx, dy = (med[k, 0] - tgt[k, 0]) * sx, (med[k, 1] - tgt[k, 1]) * sy
            err = float(np.hypot(dx, dy))
            dev = arr[:, k, :] - med[k]
            spread = float(np.median(np.hypot(dev[:, 0] * sx, dev[:, 1] * sy)))
            kind = ('pair ' if need == 2 else '') or ('timed' if h['timed'] else '')
            print(f'{i:4d} {"(%.2f,%.2f)" % p:>13} {kind:5} {len(fp):6d} {miss:5.0%} '
                  f'{dx:6.1f} {dy:6.1f} {err:6.1f} {spread:6.1f}')
            rows.append(dict(i=i, target=p, dx=dx, dy=dy, err=err, pair=need == 2,
                             miss=miss))
            by_region.setdefault('pair' if need == 2 else region(p), []).append(err)

    print(f'\ndrags (cross-track error from the line; along-track is finger lag)')
    print(f'{"step":>4} {"from":>12} {"to":>12} {"frames":>6} {"miss":>5} '
          f'{"median":>6} {"p90":>6} {"max":>6}  ({unit})')
    drag_err = []
    for i, d in sorted(drags.items()):
        a = np.array([d['a'][0] * MAX * sx, d['a'][1] * MAX * sy])
        b = np.array([d['b'][0] * MAX * sx, d['b'][1] * MAX * sy])
        u = (b - a) / np.linalg.norm(b - a)
        ft, fp = frames(T, P, d['t0'], d['t1'] + 0.3)
        e = []
        for q in fp:
            if len(q) != 1:
                continue
            v = np.array([q[0][0] * sx, q[0][1] * sy]) - a
            e.append(abs(v[0] * u[1] - v[1] * u[0]))
        miss = 1 - len(e) / len(fp) if fp else 1.0
        if e:
            e = np.array(e)
            drag_err += list(e)
            print(f'{i:4d} {"(%.2f,%.2f)" % tuple(d["a"]):>12} {"(%.2f,%.2f)" % tuple(d["b"]):>12} '
                  f'{len(fp):6d} {miss:5.0%} {np.median(e):6.1f} {np.percentile(e, 90):6.1f} '
                  f'{e.max():6.1f}')
        else:
            print(f'{i:4d} {"(%.2f,%.2f)" % tuple(d["a"]):>12} {"(%.2f,%.2f)" % tuple(d["b"]):>12} '
                  f'{len(fp):6d} {miss:5.0%}  no single-contact frames')

    print(f'\nsummary ({unit}): region, targets, median, p90, max, missing')
    for name in ('interior', 'edge', 'near camera', 'pair'):
        v = np.array(by_region.get(name, []), float)
        if not v.size:
            continue
        ok = v[~np.isnan(v)]
        if ok.size:
            print(f'  {name:12s} {v.size:3d} {np.median(ok):6.1f} {np.percentile(ok, 90):6.1f} '
                  f'{ok.max():6.1f}  {int(np.isnan(v).sum())}')
        else:
            print(f'  {name:12s} {v.size:3d}   none detected')
    if drag_err:
        e = np.array(drag_err)
        print(f'  {"drag":12s} {len(drags):3d} {np.median(e):6.1f} {np.percentile(e, 90):6.1f} '
              f'{e.max():6.1f}')
    if rows and unit == 'mm':
        dx = np.median([r['dx'] for r in rows if not r['pair']])
        dy = np.median([r['dy'] for r in rows if not r['pair']])
        print(f'  overall offset (single holds): dx {dx:+.1f} dy {dy:+.1f} mm')


if __name__ == '__main__':
    main()
