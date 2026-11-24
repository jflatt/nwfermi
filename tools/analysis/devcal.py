"""Find raw geometry under which the device's stored 4-point calibration reproduces the daemon."""
import sys
import numpy as np
sys.path.insert(0, 'tools/analysis')
exec(open('tools/analysis/calib.py').read().split("rx, ry = raw_xy()")[0])
targets = [(S[k][0], S[k + 1][0]) for k in range(0x7043, 0x704b, 2)]
W = 32767.0

def raw(q, p0=p0, p1=p1):
    c0x, c0y, c1x, c1y, r0, r1 = q
    th0 = 2 * lens(0, p0, 'p') + r0; th1 = np.pi - (2 * lens(1, p1, 'p') + r1)
    d0 = np.array([np.cos(th0), np.sin(th0)]); d1 = np.array([np.cos(th1), np.sin(th1)])
    ex, ey = c1x - c0x, c1y - c0y
    t = (ex * d1[1] - ey * d1[0]) / (d0[0] * d1[1] - d0[1] * d1[0])
    return c0x + t * d0[0], c0y + t * d0[1]

for idx in (1, 2):
    meas = [(S[k][idx], S[k + 1][idx]) for k in range(0x703b, 0x7043, 2)]
    H = homography(meas, targets)
    f = lambda q: np.concatenate([d for d in (np.array(apply(H, *raw(q))) - np.array([X, Y]))]) / 32767
    q = lm(f, np.array([0, 0, W, 0, 0, 0], float))
    x, y = apply(H, *raw(q))
    print(f'idx {idx}: {err(x, y)}')
    print(f'   cams=({q[0]:.0f},{q[1]:.0f}) ({q[2]:.0f},{q[3]:.0f})  rot=({np.degrees(q[4]):.3f},{np.degrees(q[5]):.3f}) deg')
    print(f'   in mm (width 521): cam0=({q[0]/W*521:.1f},{q[1]/W*521:.1f}) cam1=({q[2]/W*521:.1f},{q[3]/W*521:.1f})')
