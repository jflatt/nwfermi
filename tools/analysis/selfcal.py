import sys
import numpy as np
sys.path.insert(0, 'tools/analysis')
exec(open('tools/analysis/corner.py').read().split("for cam in (0, 1):")[0])

def corner_px(cam):
    on = np.mean([f['wave'] for f in calm if f['cam'] == cam and f['seq'] == 1], axis=0)
    off = np.mean([f['wave'] for f in calm if f['cam'] == cam and f['seq'] == 0], axis=0)
    sig = on - off
    lo, hi = 100, 400                  # search window in the middle of the view
    i = lo + int(np.argmin(sig[lo:hi]))
    # sub-pixel: weighted centroid of the dip below half depth
    ref = np.median(sig[lo:hi])
    w = np.clip(ref - sig[i - 6:i + 7], 0, None)
    return float(np.sum(np.arange(i - 6, i + 7) * w) / np.sum(w))

for aspect_name, A_use in (('settings 521/309', S[0x2022][0] / S[0x2023][0]),):
    corner_angle = np.arctan(1 / A_use)
    rot = {}
    for cam in (0, 1):
        pc = corner_px(cam)
        rot[cam] = corner_angle + lensv(cam, np.array([pc]), 'B')[0]
        print(f'cam{cam}: corner at px {pc:.2f} -> rotation {np.degrees(rot[cam]):.2f} deg (fit was {np.degrees(fit_rot[cam]):.2f})')
    rx, ry = rawB(rot[0], rot[1])
    print(f'{aspect_name}: self-calibrated + stored calibration -> {err(*apply(H, rx, ry))}')
    # common offset between corner dip and true corner (same optics on both cams)
    d = np.mean([fit_rot[c] - rot[c] for c in (0, 1)])
    rx, ry = rawB(rot[0] + d, rot[1] + d)
    print(f'   with one shared corner offset {np.degrees(d):+.2f} deg -> {err(*apply(H, rx, ry))}')
