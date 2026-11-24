import collections, pickle, struct, sys
cap = pickle.load(open(sys.argv[1], 'rb'))
marks = []
for l in open(sys.argv[2]):
    _, ts, *label = l.split(' ', 2)
    h, m, s = ts.split(':'); marks.append((int(h)*3600+int(m)*60+float(s), ' '.join(label).strip()))
def phase(t):
    lab = 'pre'
    for mt, ml in marks:
        if t >= mt: lab = ml
    return lab[:40]
# touch reports
print('== touch reports by phase')
agg = collections.OrderedDict()
for t, d in cap['writes']:
    st0, x0, y0, id0, st1, x1, y1, id1, cnt = struct.unpack('<Iii B3x Iii B3x B3x', d)
    a = agg.setdefault(phase(t), [])
    a.append((st0, x0, y0, id0, st1, x1, y1, id1, cnt))
for k, v in agg.items():
    sts = collections.Counter((r[0], r[4], r[8]) for r in v)
    xs = [r[1] for r in v if r[8]]; ys = [r[2] for r in v if r[8]]
    print(f'{k:40s} n={len(v):4d} states(st0,st1,cnt)={dict(sts.most_common(5))}',
          f'x0 {min(xs) if xs else "-"}..{max(xs) if xs else "-"} y0 {min(ys) if ys else "-"}..{max(ys) if ys else "-"}')
# stream packets
buf = b''.join(d for _, d in cap['reads'])
i = 0; types = collections.Counter(); lens = collections.defaultdict(collections.Counter); bad = 0; pk = []
while i + 8 <= len(buf):
    if buf[i] != 0x5b or buf[i+1] != 0x5d:
        i += 1; bad += 1; continue
    seq, ln, typ, sub, b7 = buf[i+2], struct.unpack('>H', buf[i+3:i+5])[0], buf[i+5], buf[i+6], buf[i+7]
    pk.append((i, seq, ln, typ, sub, b7))
    types[(typ, sub)] += 1; lens[(typ, sub)][ln] += 1
    i += 8 + ln
print('== packets', len(pk), 'resync bytes', bad)
for k, c in sorted(types.items()):
    print(k, c, dict(lens[k].most_common(4)))
pickle.dump(pk, open(sys.argv[1] + '.pk', 'wb'))
