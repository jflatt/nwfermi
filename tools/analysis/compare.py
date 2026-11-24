"""Compare engine replay output with the daemon's reports."""
import bisect, collections, pickle, struct, sys
import numpy as np
cap = pickle.load(open('capture/parsed/cap.pkl', 'rb'))
marks = []
for l in open('capture/marks.txt'):
    _, ts, label = l.rstrip('\n').split(' ', 2)
    h, m, s = ts.split(':'); marks.append((int(h) * 3600 + int(m) * 60 + float(s), label))
def phase(t):
    lab = 'pre'
    for mt, ml in marks:
        if t >= mt: lab = ml
    return lab
D = []
for t, d in cap['writes']:
    st0, x0, y0, id0, st1, x1, y1, id1, cnt = struct.unpack('<Iii B3x Iii B3x B3x', d)
    pts = []
    if cnt >= 1 and st0 in (1, 2): pts.append((x0, y0))
    if cnt >= 2 and st1 in (1, 2): pts.append((x1, y1))
    D.append((t, pts))
dt = [d[0] for d in D]
E = []
for l in open(sys.argv[1] if len(sys.argv) > 1 else 'capture/parsed/engine.txt'):
    f = l.split(); t = float(f[0]); n = int(f[1])
    E.append((t, [(int(f[3 + 3 * i]), int(f[4 + 3 * i])) for i in range(n)]))
stats = collections.OrderedDict()
for t, pts in E:
    k = bisect.bisect_left(dt, t)
    cands = [D[j] for j in (k - 1, k) if 0 <= j < len(D) and abs(D[j][0] - t) < 0.05]
    if not cands:
        continue
    dp = min(cands, key=lambda d: abs(d[0] - t))[1]
    s = stats.setdefault(phase(t)[:44], dict(n=0, agree=0, eng_more=0, eng_less=0, err=[]))
    s['n'] += 1
    if len(pts) == len(dp):
        s['agree'] += 1
        for p in pts:
            e = min(np.hypot(p[0] - q[0], p[1] - q[1]) for q in dp)
            s['err'].append(e)
    elif len(pts) > len(dp): s['eng_more'] += 1
    else: s['eng_less'] += 1
print(f'{"phase":44s} {"frames":>6s} {"same#":>6s} {"eng+":>5s} {"eng-":>5s} {"err med":>8s} {"p90":>6s}  (% of 32767)')
for k, s in stats.items():
    e = np.array(s['err']) if s['err'] else np.array([np.nan])
    print(f'{k:44s} {s["n"]:6d} {s["agree"]:6d} {s["eng_more"]:5d} {s["eng_less"]:5d} {100*np.median(e)/32767:7.2f}% {100*np.percentile(e,90)/32767:5.2f}%')
