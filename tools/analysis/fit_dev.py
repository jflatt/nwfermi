"""Fit geometry only, using the device's own lens polynomials."""
import pickle, sys
import numpy as np
sys.path.insert(0, 'tools/analysis')
import settings

P = pickle.load(open('capture/parsed/pairs.pkl', 'rb'))
p0 = np.array([p[0] for p in P]); p1 = np.array([p[2] for p in P])
X = np.array([p[4] for p in P], float); Y = np.array([p[5] for p in P], float)
S = {k: v for k, t, n, v in settings.decode(open('capture/parsed/settings.bin', 'rb').read())}
co = {cam: np.array([0 if c != c else c for c in (S[(g << 8) | i][1] for i in range(0x29, 0x30))])
      for cam, g in ((0, 0x30), (1, 0x40))}

def lens(cam, p, umode):
    u = {'512-2p': 512 - 2 * p, '2p': 2 * p, 'p': p, '512-p': 512 - p}[umode]
    return sum(c * u ** k for k, c in enumerate(co[cam])) / 512.0

def geom_angles(q):
    A, c0x, c0y, c1x, c1y = q[:5]
    t0 = np.arctan2(Y - c0y, (X - c0x) * A)
    t1 = np.arctan2(Y - c1y, (c1x - X) * A)
    return t0, t1

def resid(q, umode, sgn):
    t0, t1 = geom_angles(q)
    r0, r1, k0, k1 = q[5:9]
    return np.concatenate([sgn * k0 * lens(0, p0, umode) + r0 - t0,
                           sgn * k1 * lens(1, p1, umode) + r1 - t1])

def lm(f, q):
    lam = 1e-3
    for it in range(300):
        r = f(q); J = np.empty((r.size, q.size))
        for j in range(q.size):
            h = 1e-6 * max(1, abs(q[j])); dq = q.copy(); dq[j] += h
            J[:, j] = (f(dq) - r) / h
        H = J.T @ J; g = J.T @ r
        while True:
            step = np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-12), -g)
            if (f(q + step) ** 2).sum() < (r ** 2).sum():
                q = q + step; lam = max(lam / 3, 1e-12); break
            lam *= 4
            if lam > 1e12: return q
        if np.abs(step).max() < 1e-10: return q
    return q

res = []
for umode in ('512-2p', '2p', 'p', '512-p'):
    for sgn in (1, -1):
        for A0 in (0.9, 1.7):
            q0 = np.array([A0, 0, 0, 32767, 0, np.pi / 4, np.pi / 4, 1.0, 1.0])
            q = lm(lambda v: resid(v, umode, sgn), q0)
            e = np.degrees(np.sqrt((resid(q, umode, sgn) ** 2).mean()))
            res.append((e, umode, sgn, A0, q))
res.sort(key=lambda r: r[0])
for e, umode, sgn, A0, q in res[:4]:
    print(f'rms {e:.3f} deg  u={umode} sign={sgn:+d}  A={q[0]:.3f} cam0=({q[1]:.0f},{q[2]:.0f}) cam1=({q[3]:.0f},{q[4]:.0f}) rot=({np.degrees(q[5]):.2f},{np.degrees(q[6]):.2f}) scale=({q[7]:.4f},{q[8]:.4f})')
pickle.dump(res[0], open('capture/parsed/model_dev.pkl', 'wb'))
