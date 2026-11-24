#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Fit a new controller calibration to target captures and write a plan.

    fit.py PLAN.json CAPTURE_DIR...

What can be stored in the controller, and so what is fitted:
  * a new 4-point calibration record (the measured raw positions of the four
    stored targets), giving the engine's affine map and camera tilt;
  * camera 1's lens coefficients, rescaled (camera 0's needs no change).

Each capture is held out in turn to show how well the fit generalises, then
the plan is checked end to end: the new records are put into the recorded
settings packets, and the streams are replayed through tools/engine/replay.

Needs numpy.  Run it as your normal user; controller.py applies the plan.
"""
import bisect, json, os, struct, subprocess, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from settingslib import CAL_MEAS, CAL_TARGET, LENS, Settings, settings_packets

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REPLAY = os.path.join(ROOT, 'tools/engine/replay')
MAX = 32767.0
Q24 = float(1 << 24)


def replay(stream, dump=None, out=None):
    env = dict(os.environ)
    if dump:
        env['NWF_DUMP'] = dump
    with open(out or os.devnull, 'w') as f:
        subprocess.run([REPLAY, stream], stdout=f, stderr=subprocess.DEVNULL, env=env, check=True)


def load(cap):
    ev = [json.loads(l) for l in open(os.path.join(cap, 'targets.jsonl'))]
    scr = ev[0]['screen']
    holds = {e['i']: e for e in ev if e['ev'] == 'hold'}
    dump = os.path.join(cap, 'dips.txt')
    replay(os.path.join(cap, 'stream.bin'), dump)
    T, F, lens, edge = [], [], {}, {}
    for l in open(dump):
        f = l.split()
        if f[0] == 'D':
            T.append(float(f[1]))
            k, cams = 2, []
            for c in range(2):
                n = int(f[k]); k += 1
                cams.append([int(f[k + 3 * j]) / 256.0 for j in range(n)]); k += 3 * n
            F.append(cams)
        elif f[0] == 'L':
            c = int(f[1]); edge[c] = int(f[3]) / 256.0
            lens[c] = np.array([int(v) for v in f[4:]]) / Q24
    rows = []
    for i, h in sorted(holds.items()):
        if len(h['pts']) != 1:
            continue
        a, b = bisect.bisect_left(T, h['t0'] + 0.2), bisect.bisect_right(T, h['t1'])
        fr = [F[k] for k in range(a, b) if len(F[k][0]) == 1 and len(F[k][1]) == 1]
        if fr:
            rows.append((h['pts'][0][0], h['pts'][0][1],
                         np.median([x[0][0] for x in fr]), np.median([x[1][0] for x in fr])))
    return dict(name=os.path.basename(cap.rstrip('/')), R=np.array(rows), W=scr['mm_w'], H=scr['mm_h'],
                lens=lens, edge=edge)


def region(fx, fy):
    if max(abs(fx - .5), abs(fy - .5)) <= .42:
        return 'interior'
    if fy < .2 and (fx < .2 or fx > .8):
        return 'near camera'
    return 'edge'


# q = [x map (3), y map (3), camera-1 lens scale]; affine raw -> output units
def tilt(q):
    return (np.arctan2(q[3], q[4]) - np.arctan2(q[1], q[0])) / 2


def model(D, q, corner):
    L = lambda c, p: np.interp(p, np.arange(512), D['lens'][c])
    th, k1, R = tilt(q), q[6], D['R']
    a0 = corner + L(0, D['edge'][0]) + th - L(0, R[:, 2])
    a1 = corner + k1 * L(1, D['edge'][1]) - th - k1 * L(1, R[:, 3])
    den = np.sin(a0 + a1)
    rx, ry = MAX * np.sin(a1) * np.cos(a0) / den, MAX * np.sin(a1) * np.sin(a0) / den
    x = q[0] * rx + q[1] * ry + q[2]
    y = q[3] * rx + q[4] * ry + q[5]
    return np.r_[(x / MAX - R[:, 0]) * D['W'], (y / MAX - R[:, 1]) * D['H']]


def lm(f, q):
    lam, r = 1e-3, f(q)
    cost = r @ r
    for _ in range(200):
        J = np.empty((r.size, q.size))
        for j in range(q.size):
            h = 1e-6 * max(1.0, abs(q[j])); dq = q.copy(); dq[j] += h
            J[:, j] = (f(dq) - r) / h
        A, g = J.T @ J, J.T @ r
        while True:
            step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
            rn = f(q + step)
            if rn @ rn < cost:
                done = cost - rn @ rn < 1e-12 * cost
                q, r, cost, lam = q + step, rn, rn @ rn, max(lam / 3, 1e-12)
                break
            lam *= 4
            if lam > 1e12:
                return q
        if done:
            return q
    return q


def fit(caps, q0, corner):
    return lm(lambda q: np.concatenate([model(D, q, corner) for D in caps]), q0.copy())


def summary(errs):
    out = {}
    for e, reg in errs:
        for v, r in zip(e, reg):
            out.setdefault(r, []).append(v); out.setdefault('all', []).append(v)
    return '  '.join(f'{r} {np.median(out[r]):.1f}/{np.percentile(out[r], 90):.1f}/{max(out[r]):.1f}'
                     for r in ('interior', 'edge', 'near camera', 'all') if r in out)


def stored_map(S):
    """The engine's affine map from the record it uses now (same maths, in float)."""
    rec = min(S.active(k) for k in CAL_MEAS)
    m = np.array([[S.float(CAL_MEAS[2 * i], rec), S.float(CAL_MEAS[2 * i + 1], rec)] for i in range(4)])
    t = np.array([[S.float(CAL_TARGET[2 * i], 0), S.float(CAL_TARGET[2 * i + 1], 0)] for i in range(4)])
    A = np.c_[np.trunc(m), np.ones(4)]
    ax, *_ = np.linalg.lstsq(A, np.trunc(t[:, 0]), rcond=None)
    ay, *_ = np.linalg.lstsq(A, np.trunc(t[:, 1]), rcond=None)
    return np.r_[ax, ay], rec


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    plan_path, cap_dirs = sys.argv[1], sys.argv[2:]
    subprocess.run(['make', '-s', '-C', ROOT, 'replay'], check=True)

    # the controller's current settings, from the newest capture
    blob = None
    with open(os.path.join(cap_dirs[-1], 'stream.bin'), 'rb') as f:
        data = f.read()
    buf, pos = bytearray(), 0
    while pos + 12 <= len(data) and len(buf) < 1 << 20:
        t, n = struct.unpack_from('<dI', data, pos); buf += data[pos + 12:pos + 12 + n]; pos += 12 + n
    for off, p in settings_packets(buf):
        blob = p
        break
    if blob is None:
        sys.exit('no settings packet in the capture')
    S = Settings(blob)
    sw, sh = struct.unpack('>hh', S.raw(0x2022, 0) + S.raw(0x2023, 0))
    corner = np.arctan2(sh, sw)
    cur, rec = stored_map(S)

    print('loading captures (replaying each through the engine)...')
    caps = [load(c) for c in cap_dirs]
    regs = lambda D: [region(a, b) for a, b in D['R'][:, :2]]
    q0 = np.r_[cur, 1.0]
    print(f'\ncurrent (record {rec}): ' + summary([(np.hypot(*model(D, q0, corner).reshape(2, -1)), regs(D)) for D in caps]))
    if len(caps) > 1:
        errs = []
        for k in range(len(caps)):
            q = fit([c for j, c in enumerate(caps) if j != k], q0, corner)
            errs.append((np.hypot(*model(caps[k], q, corner).reshape(2, -1)), regs(caps[k])))
        print('fitted, each capture held out:  ' + summary(errs))
    q = fit(caps, q0, corner)
    print(f'fitted on all captures:         ' + summary([(np.hypot(*model(D, q, corner).reshape(2, -1)), regs(D)) for D in caps]))
    print(f'  camera tilt {np.degrees(tilt(q)):+.3f} deg, camera 1 lens scale {q[6]:.5f}   (median/p90/max mm)')

    # the record that reproduces the fitted map, and the rescaled lens terms
    t = np.array([[S.float(CAL_TARGET[2 * i], 0), S.float(CAL_TARGET[2 * i + 1], 0)] for i in range(4)])
    A = np.array([[q[0], q[1]], [q[3], q[4]]])
    meas = np.linalg.solve(A, (t - [q[2], q[5]]).T).T
    lens_set = S.active(LENS[1][1])
    writes = []
    for i in range(4):
        for j in range(2):
            writes.append(dict(key=CAL_MEAS[2 * i + j], value=float(np.float32(meas[i, j])),
                               what=f'calibration point {i} {"xy"[j]}'))
    lens_writes = []
    for k, key in enumerate(LENS[1]):
        if S.active(key) != lens_set:
            continue        # erased in the current set: stays zero
        v = S.float(key, lens_set)
        lens_writes.append(dict(key=key, value=float(np.float32(v * q[6])), old=v, what=f'camera 1 lens c{k}'))
    # the linear term last: the engine only switches sets once it is there
    lens_writes.sort(key=lambda w: w['key'] == LENS[1][1])
    writes += lens_writes
    for w in writes:
        w['index'], w['type'] = S.desc[w['key']][0], S.desc[w['key']][1]
        w['element'] = S.active(w['key']) + 1
        w['bits'] = struct.pack('>f', w['value']).hex()

    # end-to-end check: patch the settings packets and replay
    new = S.appended([(w['key'], bytes.fromhex(w['bits'])) for w in writes])
    print('\nsimulated controller with the new records, replayed:')
    errs = []
    for cap in cap_dirs:
        with open(os.path.join(cap, 'stream.bin'), 'rb') as f:
            data = f.read()
        # patch the concatenated stream (packets can straddle reads), then re-split
        recs, cat, pos = [], bytearray(), 0
        while pos + 12 <= len(data):
            t_, ln = struct.unpack_from('<dI', data, pos)
            recs.append((t_, ln)); cat += data[pos + 12:pos + 12 + ln]; pos += 12 + ln
        n = 0
        for off, p in list(settings_packets(cat)):
            if p == S.blob:
                cat[off:off + len(p)] = new.blob; n += 1
        sim = os.path.join(cap, 'stream-simulated.bin')
        with open(sim, 'wb') as f:
            pos = 0
            for t_, ln in recs:
                f.write(struct.pack('<dI', t_, ln) + cat[pos:pos + ln]); pos += ln
        eng = os.path.join(cap, 'engine-simulated.txt')
        replay(sim, out=eng)
        r = subprocess.run([sys.executable, '-I', os.path.join(ROOT, 'tools/target/eval.py'), cap, '--engine', eng],
                           capture_output=True, text=True).stdout
        summ = r[r.index('summary'):].splitlines()
        print(f'  {os.path.basename(cap)} ({n} settings packets patched)')
        for l in summ[1:4]:
            print('   ', l.strip())
        os.unlink(sim)

    plan = dict(created_from=cap_dirs, settings_before=S.blob.hex(), settings_after=new.blob.hex(),
                record_in_use=rec, tilt_deg=float(np.degrees(tilt(q))), lens_scale_cam1=float(q[6]),
                writes=writes)
    with open(plan_path, 'w') as f:
        json.dump(plan, f, indent=1)
    print(f'\nplan: {len(writes)} writes -> {plan_path}')
    for w in writes:
        cur_s = f"(was {w['old']:.6g})" if 'old' in w else ''
        print(f"  {w['key']:04x} idx {w['index']:3d} elem {w['element']}  {w['value']:14.6g} {cur_s:22s} {w['what']}")


if __name__ == '__main__':
    main()
