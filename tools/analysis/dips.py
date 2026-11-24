import bisect, sys
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import frames

def build(cap, marks):
    F, W, M = frames.load(cap, marks)
    calm = [f for f in F if f['phase'].startswith('Do NOT')]
    base = {}
    for cam in (0, 1):
        for seq in (0, 1):
            rows = [f['wave'] for f in calm if f['cam'] == cam and f['seq'] == seq]
            base[cam, seq] = [sum(r[i] for r in rows) / len(rows) for i in range(512)]
    last = {}
    out = []   # (t, cam, [dips], phase)
    for f in F:
        last[f['cam'], f['seq']] = f
        if f['seq'] != 1 or (f['cam'], 0) not in last:
            continue
        on, off = f['wave'], last[f['cam'], 0]['wave']
        b1, b0 = base[f['cam'], 1], base[f['cam'], 0]
        r = []
        for i in range(512):
            den = b1[i] - b0[i]
            r.append((on[i] - off[i]) / den if den > 15 else 1.0)
        dips, i = [], 0
        while i < 512:
            if r[i] < 0.5:
                j = i
                while j < 512 and r[j] < 0.5: j += 1
                w = [(1 - r[k]) for k in range(i, j)]
                c = sum(k * wk for k, wk in zip(range(i, j), w)) / sum(w)
                dips.append((round(c, 2), j - i))
                i = j
            else:
                i += 1
        out.append((f['t'], f['cam'], dips, f['phase'], r))
    return out, W, base
