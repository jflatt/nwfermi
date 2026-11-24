"""Pair single-touch dip centres (cam0, cam1) with the daemon's reported position."""
import bisect, pickle, sys
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dips

D, W, base = dips.build(sys.argv[1], sys.argv[2])
wt = [w[0] for w in W]
last = {0: None, 1: None}
pairs = []
for t, cam, d, phase, r in D:
    last[cam] = (t, d)
    if last[0] is None or last[1] is None or abs(last[0][0] - last[1][0]) > 0.02:
        continue
    d0, d1 = last[0][1], last[1][1]
    if len(d0) != 1 or len(d1) != 1:
        continue
    # daemon report shortly after (processing latency)
    k = bisect.bisect_left(wt, t)
    if k >= len(W) or W[k][0] - t > 0.03:
        continue
    st0, x0, y0, id0, st1, x1, y1, id1, cnt = W[k][1:]
    if cnt != 1 or st0 not in (1, 2):
        continue
    pairs.append((d0[0][0], d0[0][1], d1[0][0], d1[0][1], x0, y0, phase))
pickle.dump(pairs, open(sys.argv[3], 'wb'))
print(len(pairs), 'pairs')
import collections
print(collections.Counter(p[6][:30] for p in pairs))
