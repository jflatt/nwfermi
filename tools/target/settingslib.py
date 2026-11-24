# SPDX-License-Identifier: GPL-2.0-only
"""The Fermi controller's settings block, as sent in stream packets (type 2/2).

Layout: tree_len:u16 values_len:u16 0:u16, then tree_len/4 descriptors
(group:u8 id:u8 type:u8 count:u8), then the values.  Scalars come first in
descriptor order; arrays follow, count elements each.

Array settings are append-only logs in flash: a write goes into the next
erased (all 0xff) element and the last written element is in force.
"""
import struct

SIZES = {1: 1, 2: 2, 3: 4, 4: 1, 5: 2, 6: 4, 7: 4, 8: 14, 9: 9}

CAL_MEAS = [0x703b + k for k in range(8)]      # x0 y0 x1 y1 x2 y2 x3 y3
CAL_TARGET = [0x7043 + k for k in range(8)]
LENS = {0: [0x3029 + k for k in range(7)], 1: [0x4029 + k for k in range(7)]}


def is_scalar(t):
    return (~t & 0xc0) in (0xc0, 0x40)


class Settings:
    def __init__(self, blob):
        self.blob = bytes(blob)
        tl, vl, _ = struct.unpack('>HHH', blob[:6])
        self.vals_at = 6 + tl
        self.desc = {}          # key -> (index, type, count, offset, size)
        d = [tuple(blob[6 + 4 * k:10 + 4 * k]) for k in range(tl // 4)]
        a = 0
        b = sum(SIZES[t & 0x3f] for g, i, t, n in d if t and is_scalar(t))
        for idx, (g, i, t, n) in enumerate(d):
            if not t:
                break
            sz = SIZES[t & 0x3f]
            off = a if is_scalar(t) else b
            self.desc[(g << 8) | i] = (idx, t, n, off, sz)
            if is_scalar(t):
                a += sz
            b += n * sz

    def raw(self, key, elem=0):
        idx, t, n, off, sz = self.desc[key]
        if is_scalar(t):
            assert elem == 0
        else:
            off += elem * sz
        p = self.vals_at + off
        return self.blob[p:p + sz]

    def count(self, key):
        idx, t, n, off, sz = self.desc[key]
        return 1 if is_scalar(t) else n

    def active(self, key):
        last = -1
        for e in range(self.count(key)):
            if self.raw(key, e) != b'\xff' * len(self.raw(key, e)):
                last = e
        return last

    def float(self, key, elem):
        return struct.unpack('>f', self.raw(key, elem))[0]

    def appended(self, writes):
        """A copy with each (key, 4-byte value) written into its next erased element."""
        blob = bytearray(self.blob)
        for key, val in writes:
            s = Settings(blob)
            e = s.active(key) + 1
            idx, t, n, off, sz = s.desc[key]
            if e >= n:
                raise ValueError(f'{key:04x}: no erased element left')
            p = s.vals_at + off + e * sz
            blob[p:p + sz] = val
        return Settings(blob)


def settings_packets(buf):
    """Yield (offset of payload, payload) for each settings packet in a raw stream."""
    i, n = 0, len(buf)
    while True:
        i = buf.find(b'[]', i)
        if i < 0 or i + 8 > n:
            return
        ln = struct.unpack_from('>H', buf, i + 3)[0]
        typ, sub = buf[i + 5], buf[i + 6]
        if typ == 2 and sub == 2 and 6 <= ln and i + 8 + ln <= n:
            p = bytes(buf[i + 8:i + 8 + ln])
            tl, vl = struct.unpack_from('>HH', p)
            if 6 + tl + vl <= ln:
                yield i + 8, p
                i += 8 + ln
                continue
        i += 1
