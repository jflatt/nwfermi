import pickle, sys
import numpy as np
sys.path.insert(0, 'tools/analysis')
exec(open('tools/analysis/fit_dev.py').read().split('res = []')[0])
A = S[0x2022][0] / S[0x2023][0]

def raw_xy(rot0=0.0, rot1=0.0, cams=(0, 0, 32767, 0)):
    th0 = 2 * lens(0, p0, 'p') + rot0; th1 = 2 * lens(1, p1, 'p') + rot1
    c0x, c0y, c1x, c1y = cams
    k0, k1 = np.tan(th0), np.tan(th1)
    x = (k1 * A * c1x + k0 * A * c0x + c1y - c0y) / (k0 * A + k1 * A)
    return x, c0y + k0 * A * (x - c0x)

def homography(src, dst):
    M = []
    for (x, y), (u, v) in zip(src, dst):
        M.append([x, y, 1, 0, 0, 0, -u * x, -u * y, -u])
        M.append([0, 0, 0, x, y, 1, -v * x, -v * y, -v])
    _, _, Vt = np.linalg.svd(np.array(M, float))
    return Vt[-1].reshape(3, 3) / Vt[-1][-1]

def apply(H, x, y):
    w = H[2, 0] * x + H[2, 1] * y + H[2, 2]
    return (H[0, 0] * x + H[0, 1] * y + H[0, 2]) / w, (H[1, 0] * x + H[1, 1] * y + H[1, 2]) / w

def err(x, y):
    e = np.hypot(x - X, (y - Y) / A)
    return f'median {100*np.median(e)/32767:.2f}% p90 {100*np.percentile(e,90)/32767:.2f}% max {100*e.max()/32767:.2f}%'

rx, ry = raw_xy()
print('raw nominal geometry          ', err(rx, ry))
Hfit = homography(np.c_[rx, ry][::5], np.c_[X, Y][::5])
print('best homography raw->daemon   ', err(*apply(Hfit, rx, ry)))
targets = [(S[k][0], S[k + 1][0]) for k in range(0x7043, 0x704b, 2)]
for idx in (1, 2):
    meas = [(S[k][idx], S[k + 1][idx]) for k in range(0x703b, 0x7043, 2)]
    H = homography(meas, targets)
    print(f'settings calibration idx {idx}   ', err(*apply(H, rx, ry)))
    # where does the best-fit homography put the measured points vs targets?
    m = np.array(meas)
    print('   measured', meas)
print('targets', targets)
# raw coordinates of the daemon's targets under the best fit: inverse map
Hi = np.linalg.inv(Hfit)
print('best-fit raw positions of targets:', [tuple(int(v) for v in apply(Hi, tx, ty)) for tx, ty in targets])

print('\n== isotropic raw coordinates (y in width units)')
def raw_iso(rot0=0.0, rot1=0.0, W=32767.0):
    th0 = 2 * lens(0, p0, 'p') + rot0; th1 = 2 * lens(1, p1, 'p') + rot1
    k0, k1 = np.tan(th0), np.tan(th1)
    x = W * k1 / (k0 + k1)
    return x, k0 * x
ix, iy = raw_iso()
for idx in (1, 2):
    meas = [(S[k][idx], S[k + 1][idx]) for k in range(0x703b, 0x7043, 2)]
    H = homography(meas, targets)
    print(f'settings calibration idx {idx}  ', err(*apply(H, ix, iy)))
H = homography(np.c_[ix, iy][::5], np.c_[X, Y][::5])
print('best homography               ', err(*apply(H, ix, iy)))
print('best-fit raw iso positions of targets:', [tuple(int(v) for v in apply(np.linalg.inv(H), tx, ty)) for tx, ty in targets])
