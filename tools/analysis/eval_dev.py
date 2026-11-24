import pickle, sys
import numpy as np
sys.path.insert(0, 'tools/analysis')
exec(open('tools/analysis/fit_dev.py').read().split('res = []')[0])

def triang(A, c0x, c0y, c1x, c1y, th0, th1):
    k0, k1 = np.tan(th0), np.tan(th1)
    x = (k1 * A * c1x + k0 * A * c0x + c1y - c0y) / (k0 * A + k1 * A)
    return x, c0y + k0 * A * (x - c0x)

def report(name, A, c0x, c0y, c1x, c1y, r0, r1, k0, k1):
    th0 = k0 * lens(0, p0, 'p') + r0; th1 = k1 * lens(1, p1, 'p') + r1
    x, y = triang(A, c0x, c0y, c1x, c1y, th0, th1)
    e = np.hypot(x - X, (y - Y) / A)          # in width units, isotropic
    print(f'{name:28s} median {100*np.median(e)/32767:.2f}%  p90 {100*np.percentile(e,90)/32767:.2f}%  max {100*e.max()/32767:.2f}% of width')

q = pickle.load(open('capture/parsed/model_dev.pkl', 'rb'))[4]
report('free scale/aspect', *q)
# constrained: scale 2, aspect from settings 2022/2023
A_set = S[0x2022][0] / S[0x2023][0]
def resid_c(v):
    full = np.array([A_set, *v[:4], v[4], v[5], 2.0, 2.0])
    return resid(full, 'p', 1)
vc = lm(resid_c, np.array([*q[1:5], q[5], q[6]]))
report('scale=2, aspect=settings', A_set, *vc[:4], vc[4], vc[5], 2.0, 2.0)
print('constrained geometry: cam0=(%.0f,%.0f) cam1=(%.0f,%.0f) rot=(%.2f,%.2f) deg' % (*vc[:4], np.degrees(vc[4]), np.degrees(vc[5])))
pickle.dump(dict(A=A_set, cams=vc[:4], rot=vc[4:6]), open('capture/parsed/model_c.pkl', 'wb'))
