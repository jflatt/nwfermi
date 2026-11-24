"""Fit a two-camera pixel->angle model to (dip, daemon position) pairs."""
import pickle, sys
import numpy as np

P = pickle.load(open(sys.argv[1], 'rb'))
p0 = np.array([p[0] for p in P]); p1 = np.array([p[2] for p in P])
X = np.array([p[4] for p in P], float); Y = np.array([p[5] for p in P], float)
DEG = 3

def unpack(q):
    A, c0x, c0y, c1x, c1y = q[:5]
    a = q[5:5 + DEG + 1]; b = q[5 + DEG + 1:]
    return A, c0x, c0y, c1x, c1y, a, b

def poly(c, p):
    u = (p - 256.0) / 256.0
    return sum(ci * u ** i for i, ci in enumerate(c))

def resid(q):
    A, c0x, c0y, c1x, c1y, a, b = unpack(q)
    t0 = np.arctan2(Y - c0y, (X - c0x) * A)
    t1 = np.arctan2(Y - c1y, (c1x - X) * A)
    return np.concatenate([poly(a, p0) - t0, poly(b, p1) - t1])

def triangulate(q, p0, p1):
    A, c0x, c0y, c1x, c1y, a, b = unpack(q)
    k0, k1 = np.tan(poly(a, p0)), np.tan(poly(b, p1))
    # y - c0y = k0*A*(x - c0x);  y - c1y = k1*A*(c1x - x)
    x = (k1 * A * c1x + k0 * A * c0x + c1y - c0y) / (k0 * A + k1 * A)
    y = c0y + k0 * A * (x - c0x)
    return x, y

s = np.pi / 2 / 460
q = np.array([1.78, 0, 0, 32767, 0] + [s * 216, s * 256, 0, 0] + [s * 216, s * 256, 0, 0], float)
lam = 1e-3
for it in range(200):
    r = resid(q); J = np.empty((r.size, q.size))
    for j in range(q.size):
        h = 1e-6 * max(1, abs(q[j])); dq = q.copy(); dq[j] += h
        J[:, j] = (resid(dq) - r) / h
    H = J.T @ J; g = J.T @ r
    while True:
        step = np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-12), -g)
        if (resid(q + step) ** 2).sum() < (r ** 2).sum():
            q += step; lam = max(lam / 3, 1e-9); break
        lam *= 4
        if lam > 1e9: break
    if np.abs(step).max() < 1e-9 or lam > 1e9: break
print('iterations', it, 'rms angle err (deg)', np.degrees(np.sqrt((resid(q) ** 2).mean())))
A, c0x, c0y, c1x, c1y, a, b = unpack(q)
print(f'aspect A={A:.4f} cam0=({c0x:.0f},{c0y:.0f}) cam1=({c1x:.0f},{c1y:.0f})')
print('cam0 poly', np.round(a, 6)); print('cam1 poly', np.round(b, 6))
x, y = triangulate(q, p0, p1)
e = np.hypot(x - X, y - Y)
print(f'position error (units of 1/32767): median {np.median(e):.0f} p90 {np.percentile(e,90):.0f} max {e.max():.0f}')
print(f'  as % of width: median {100*np.median(e)/32767:.2f}%  p90 {100*np.percentile(e,90)/32767:.2f}%')
pickle.dump(q, open(sys.argv[2], 'wb'))
