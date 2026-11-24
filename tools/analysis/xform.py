import sys
import numpy as np
sys.path.insert(0, 'tools/analysis')
exec(open('tools/analysis/selfcal.py').read().split("for aspect_name")[0])
A_use = S[0x2022][0] / S[0x2023][0]
OFFS = np.radians(1.44)
rot = {c: np.arctan(1 / A_use) + lensv(c, np.array([corner_px(c)]), 'B')[0] + OFFS for c in (0, 1)}
rx, ry = rawB(rot[0], rot[1])
meas = np.array([(S[k][1], S[k + 1][1]) for k in range(0x703b, 0x7043, 2)])
tg = np.array(targets)
print('homography      ', err(*apply(homography(meas, tg), rx, ry)))
M = np.c_[meas, np.ones(4)]
ax, *_ = np.linalg.lstsq(M, tg[:, 0], rcond=None); ay, *_ = np.linalg.lstsq(M, tg[:, 1], rcond=None)
print('affine lstsq    ', err(ax[0] * rx + ax[1] * ry + ax[2], ay[0] * rx + ay[1] * ry + ay[2]))
print('affine coeffs   ', np.round(ax, 5), np.round(ay, 5))
