import sys
import numpy as np
sys.path.insert(0, 'tools/analysis')
import frames
exec(open('tools/analysis/conv_fixed.py').read().split("for r0, r1, name")[0])
F, Wr, M = frames.load('capture/parsed/cap.pkl', 'capture/marks.txt')
calm = [f for f in F if f['phase'].startswith('Do NOT')]
A_set = S[0x2022][0] / S[0x2023][0]
fit_rot = {0: np.radians(40.50), 1: np.radians(41.76)}
for cam in (0, 1):
    on = np.mean([f['wave'] for f in calm if f['cam'] == cam and f['seq'] == 1], axis=0)
    off = np.mean([f['wave'] for f in calm if f['cam'] == cam and f['seq'] == 0], axis=0)
    sig = on - off
    nz = np.nonzero(sig > 5)[0]
    lo, hi = nz[0] + 5, nz[-1] - 5
    # local minima with depth
    mins = []
    for i in range(lo + 3, hi - 3):
        if sig[i] == sig[i - 3:i + 4].min():
            depth = (sig[i - 8:i + 9].max() - sig[i]) / max(sig[i - 8:i + 9].max(), 1)
            if depth > 0.25: mins.append((i, int(sig[i]), round(depth, 2)))
    print(f'cam{cam} signal px {lo}..{hi}; minima:', mins)
    print('   profile/16:', ' '.join(str(int(v)) for v in sig[lo:hi:16]))
    # angle of each minimum under the fitted model; expected corner angle atan(1/A)
    for i, v, d in mins:
        ang = fit_rot[cam] - lensv(cam, np.array([float(i)]), 'B')[0]
        print(f'   px {i}: model angle {np.degrees(ang):.2f} deg (corner expected {np.degrees(np.arctan(1/A_set)):.2f})')
