import sys
import numpy as np
sys.path.insert(0, 'tools/analysis')
exec(open('tools/analysis/devcal.py').read().split("for idx in (1, 2):")[0])
meas = [(S[k][1], S[k + 1][1]) for k in range(0x703b, 0x7043, 2)]
H = homography(meas, targets)

def lensv(cam, p, conv):
    c = co[cam]
    if conv == 'A':   v = sum(ck * p ** k for k, ck in enumerate(c)) / 256            # 2*poly(p)/512
    elif conv == 'B': v = sum(ck * (512 - 2 * p) ** k for k, ck in enumerate(c)) / 512 # daemon non-reversed
    elif conv == 'C': v = sum(ck * (2 * p) ** k for k, ck in enumerate(c)) / 512
    elif conv == 'D': v = sum(ck * p ** k for k, ck in enumerate(c)) / 512
    return v

def run(conv, mode):
    def raw(q):
        d0p, d1p, r0, r1, s0, s1 = q
        a0 = s0 * lensv(0, p0 + d0p, conv) + r0
        a1 = s1 * lensv(1, p1 + d1p, conv) + r1
        th0, th1 = a0, np.pi - a1
        k0, k1 = np.tan(th0), -np.tan(th1)
        x = W * k1 / (k0 + k1)
        return x, k0 * x
    free = {'rot': [2, 3], 'pix': [0, 1], 'rot+scale': [2, 3, 4, 5], 'pix+scale': [0, 1, 4, 5]}[mode]
    base = np.array([0, 0, 0, 0, 1, 1], float)
    def f(v):
        q = base.copy(); q[free] = v
        return np.concatenate(np.array(apply(H, *raw(q))) - np.array([X, Y])) / 32767
    best = None
    for sgn in (1, -1):
        for r in (0.0, np.pi / 4, -np.pi / 4, np.pi / 2):
            init = base.copy(); init[4:] = sgn
            if mode.startswith('rot'): init[2:4] = r
            b0 = base.copy(); base[:] = init
            v = lm(f, init[free].copy())
            e = np.sqrt((f(v) ** 2).mean())
            if best is None or e < best[0]:
                best = (e, v.copy(), init.copy())
            base[:] = b0
    e, v, init = best
    base[:] = init; q = base.copy(); q[free] = v
    x, y = apply(H, *raw(q))
    return err(x, y), q

for conv in 'ABCD':
    for mode in ('rot', 'pix', 'rot+scale'):
        e, q = run(conv, mode)
        print(f'{conv} {mode:10s} {e}  pix=({q[0]:.2f},{q[1]:.2f}) rot=({np.degrees(q[2]):.2f},{np.degrees(q[3]):.2f}) scale=({q[4]:.4f},{q[5]:.4f})')
