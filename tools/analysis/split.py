"""Split an strace capture of nwfermi_daemon into the device stream and touch reports."""
import pickle, re, sys

src, out = sys.argv[1], sys.argv[2]
line_rx = re.compile(r'^(\d+)\s+(\d\d):(\d\d):(\d\d\.\d+) (.*)$')
call_rx = re.compile(r'(read|write)\((\d+), "(.*)", (\d+)\)\s+= (-?\d+)')
hex_rx = re.compile(r'\\x([0-9a-f]{2})')
pending, reads, writes = {}, [], []
for line in open(src, errors='replace'):
    m = line_rx.match(line.rstrip('\n'))
    if not m:
        continue
    pid, h, mi, s, rest = m.groups()
    t = int(h) * 3600 + int(mi) * 60 + float(s)
    if rest.endswith('<unfinished ...>'):
        pending[pid] = (t, rest[:-len('<unfinished ...>')])
        continue
    if rest.startswith('<... '):
        t, head = pending.pop(pid, (t, ''))
        rest = head + rest.split('resumed>', 1)[1]
    c = call_rx.match(rest)
    if not c or c.group(2) != '3' or int(c.group(5)) <= 0:
        continue
    data = bytes(int(x, 16) for x in hex_rx.findall(c.group(3)))[:int(c.group(5))]
    (reads if c.group(1) == 'read' else writes).append((t, data))
print('reads', len(reads), sum(len(d) for _, d in reads), 'bytes; writes', len(writes))
pickle.dump({'reads': reads, 'writes': writes}, open(out, 'wb'))
