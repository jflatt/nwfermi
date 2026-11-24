"""Build per-frame waveform arrays (512 px, reversed per flags) with phase labels."""
import pickle, sys
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fermi

def load(cap_path, marks_path):
    cap = pickle.load(open(cap_path, 'rb'))
    marks = []
    for l in open(marks_path):
        _, ts, label = l.rstrip('\n').split(' ', 2)
        h, m, s = ts.split(':'); marks.append((int(h) * 3600 + int(m) * 60 + float(s), label))
    # map stream byte offset -> read timestamp
    offs, t_of = [], []
    pos = 0
    for t, d in cap['reads']:
        offs.append(pos); t_of.append(t); pos += len(d)
    buf = b''.join(d for _, d in cap['reads'])
    import bisect
    frames = []
    for i, seq, typ, sub, b7, ln in fermi.packets(buf):
        if typ != 1:
            continue
        w, _ = fermi.decode_waveform(buf, i + 8, ln)
        t = t_of[bisect.bisect_right(offs, i) - 1]
        wave = [0] * 512
        for k, v in enumerate(w['px']):
            wave[w['start'] + k] = v
        if w['flags'] & 1:
            wave = wave[::-1]
        lab = 'pre'
        for mt, ml in marks:
            if t >= mt: lab = ml
        w.update(t=t, wave=wave, phase=lab)
        frames.append(w)
    writes = []
    import struct
    for t, d in cap['writes']:
        writes.append((t,) + struct.unpack('<Iii B3x Iii B3x B3x', d))
    return frames, writes, marks
