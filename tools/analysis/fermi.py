"""Decoder for the NextWindow Fermi bulk stream (reverse engineered)."""
import struct

def packets(buf):
    """Yield (offset, seq, type, sub, b7, length, payload_view_start)."""
    i, n = 0, len(buf)
    while i + 8 <= n:
        if buf[i] != 0x5b or buf[i + 1] != 0x5d:
            i += 1
            continue
        seq, ln, typ, sub, b7 = buf[i + 2], struct.unpack_from('>H', buf, i + 3)[0], buf[i + 5], buf[i + 6], buf[i + 7]
        yield i, seq, typ, sub, b7, ln
        if typ in (1, 4):
            used = decode_waveform(buf, i + 8, ln, typ)[1]
            i += 8 + (used if used else ln)
        else:
            i += 8 + ln

def decode_waveform(buf, off, ln, typ=1):
    """Return (wave dict, bytes consumed) for a type-1 (delta) waveform."""
    ts, camseq, start, npix, flags, aux = struct.unpack_from('>HBHHBH', buf, off)
    extra = buf[off + 10]
    p = off + 11
    count_max = ln - 11
    px = []
    if typ == 1:
        while p < len(buf) and len(px) < count_max and start + len(px) <= 0x1ff:
            b = buf[p]
            if b == 0x80:
                v = (buf[p + 1] << 8) | buf[p + 2]
                p += 3
            else:
                prev = px[-1] if px else 0
                v = prev + (b - 256 if b > 127 else b)
                v = max(v, 0)
                p += 1
            px.append(v)
    else:
        return None, 0
    ok = len(px) == count_max
    w = dict(ts=ts, cam=camseq >> 4, seq=camseq & 0xf, start=start, npix=npix,
             flags=flags, aux=aux, extra=extra, px=px, ok=ok)
    return w, (p - off)
