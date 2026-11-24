"""Decode the Fermi settings packet (type 2, sub 2)."""
import struct, sys
SIZES = {1: 1, 2: 2, 3: 4, 4: 1, 5: 2, 6: 4, 7: 4, 8: 14, 9: 9}
FMT = {1: 'B', 2: 'H', 3: 'I', 4: 'b', 5: 'h', 6: 'i', 7: 'f'}

def decode(s, endian='>'):
    tl, vl, _ = struct.unpack('>HHH', s[:6])
    desc = [tuple(s[6 + 4 * k:10 + 4 * k]) for k in range(tl // 4)]
    desc = [d for d in desc if d[2]]
    vals = s[6 + tl:6 + tl + vl]
    a = 0
    b = sum(SIZES[d[2] & 0x3f] for d in desc if (~d[2] & 0xc0) in (0xc0, 0x40))
    out = []
    for g, i, t, n in desc:
        sz = SIZES[t & 0x3f]; f = FMT.get(t & 0x3f)
        cls = ~t & 0xc0
        if cls in (0xc0, 0x40):
            raw = [vals[a:a + sz]]
            a += sz
        else:
            raw = [vals[b + k * sz:b + (k + 1) * sz] for k in range(n)]
        b += n * sz
        v = [struct.unpack(endian + f, r)[0] if f else r.hex() for r in raw]
        out.append(((g << 8) | i, t, n, v))
    return out

if __name__ == '__main__':
    s = open(sys.argv[1], 'rb').read()
    for key, t, n, v in decode(s, sys.argv[2] if len(sys.argv) > 2 else '>'):
        vs = ['%.6g' % x if isinstance(x, float) else str(x) for x in v]
        print(f'{key:04x} t={t:02x} n={n:2d}', ' '.join(vs))
