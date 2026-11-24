import sys
import numpy as np
sys.path.insert(0, 'tools/analysis')
exec(open('tools/analysis/conv.py').read().split("for conv in 'ABCD':")[0])

def rawB(r0, r1):
    a0 = r0 - lensv(0, p0, 'B'); a1 = r1 - lensv(1, p1, 'B')
    k0, k1 = np.tan(a0), np.tan(a1)
    x = W * k1 / (k0 + k1)
    return x, k0 * x

for r0, r1, name in ((np.pi / 4, np.pi / 4, '45/45 fixed'),
                     (np.radians(40.50), np.radians(41.76), 'fitted 40.50/41.76')):
    rx, ry = rawB(r0, r1)
    print(f'{name:20s} stored calibration: {err(*apply(H, rx, ry))}')
    Hb = homography(np.c_[rx, ry][::5], np.c_[X, Y][::5])
    print(f'{"":20s} best homography:    {err(*apply(Hb, rx, ry))}')
# what raw position does each stored "measured" point correspond to under the 45° model?
print('stored measured points (raw):', [(round(a), round(b)) for a, b in meas])
