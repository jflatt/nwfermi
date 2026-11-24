#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Read and write the Fermi controller's stored settings.  Run as root.

    controller.py show
    controller.py write PLAN.json [--one] [--dry-run] [--yes]
    controller.py revert PLAN.json RECORD [--dry-run] [--yes]

Settings are read from the raw stream (/dev/nwfermiN), which carries the
whole settings block about once a second.  A setting is written with a
64-byte HID output report on the controller's vendor HID interface
(/dev/hidrawN), as the vendor's own tools did:

    'S' 's' index:u16be type:u8 array:u8 value (big endian)

where index is the setting's position in the descriptor table.  The
controller confirms with an input report 'S' ? 's' ... status=1 at byte 8.
Array settings are append-only logs: a write lands in the next erased
element, which becomes the one in force, and older elements are kept.

`write` applies a plan from fit.py.  It refuses unless the controller holds
exactly the settings the plan was made from (or a partly applied plan, which
it resumes), saves a backup, and after every single write waits for the
settings block and checks that exactly the expected element changed.  Any
surprise stops it.  `--one` does only the next write, as a first test.

`revert` makes an older calibration record (and the lens coefficients the
plan replaced) current again, by appending copies of them.
"""
import glob, json, os, select, struct, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from settingslib import CAL_MEAS, LENS, Settings, settings_packets

VID_PIDS = ('1926:1846', '1926:1878')


def find_hidraw():
    for h in sorted(glob.glob('/sys/class/hidraw/hidraw*')):
        dev = os.path.realpath(os.path.join(h, 'device'))
        if any(vp.upper() in dev.upper() for vp in VID_PIDS) and ':1.1/' in dev:
            return '/dev/' + os.path.basename(h)
    sys.exit('controller HID interface (hidraw) not found')


def find_stream():
    devs = sorted(glob.glob('/dev/nwfermi*'))
    if not devs:
        sys.exit('/dev/nwfermi* not found: is nwfermi.ko loaded?')
    return devs[0]


class Stream:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.buf = bytearray()

    def drain(self):
        while True:
            try:
                if not os.read(self.fd, 65536):
                    break
            except BlockingIOError:
                break
        self.buf = bytearray()

    def next_settings(self, timeout=6.0):
        """The first complete settings block that arrives from now on."""
        end = time.time() + timeout
        while time.time() < end:
            r, _, _ = select.select([self.fd], [], [], 0.2)
            if not r:
                continue
            try:
                self.buf += os.read(self.fd, 65536)
            except BlockingIOError:
                continue
            found = list(settings_packets(self.buf))
            if found:
                off, p = found[-1]
                self.buf = self.buf[off + len(p):]
                return Settings(p)
            if len(self.buf) > 1 << 20:
                self.buf = self.buf[-65536:]
        return None


def report(w):
    r = b'Ss' + struct.pack('>HBB', w['index'], w['type'], 1) + bytes.fromhex(w['bits'])
    return r + bytes(64 - len(r))


def send(hid, w, timeout=2.0):
    """Send one setting; return the controller's confirmation report or None."""
    while select.select([hid], [], [], 0)[0]:
        os.read(hid, 64)            # stale input reports
    os.write(hid, b'\x00' + report(w))
    end = time.time() + timeout
    while time.time() < end:
        if select.select([hid], [], [], 0.1)[0]:
            r = os.read(hid, 64)
            if len(r) >= 14 and r[0] == 0x53 and r[2] == 0x73 and struct.unpack('>H', r[4:6])[0] == w['index']:
                return r
    return None


def describe_diff(a, b):
    out = []
    for key in a.desc:
        for e in range(a.count(key)):
            if a.raw(key, e) != b.raw(key, e):
                out.append(f'{key:04x}[{e}]: {a.raw(key, e).hex()} -> {b.raw(key, e).hex()}')
    return out or ['(descriptor or layout change)']


def show(S):
    print('calibration records (measured raw positions of the 4 targets):')
    rec = min(S.active(k) for k in CAL_MEAS)
    for e in range(S.count(CAL_MEAS[0])):
        vals = [S.raw(k, e) for k in CAL_MEAS]
        if all(v == b'\xff' * 4 for v in vals):
            continue
        pts = [struct.unpack('>f', v)[0] for v in vals]
        mark = '  <- in force' if e == rec else ''
        print(f'  record {e}: ' + ' '.join(f'({pts[2*i]:.1f},{pts[2*i+1]:.1f})' for i in range(4)) + mark)
    print(f'  erased records left: {S.count(CAL_MEAS[0]) - 1 - max(S.active(k) for k in CAL_MEAS)}')
    for c in (0, 1):
        act = S.active(LENS[c][1])
        print(f'camera {c} lens coefficients in force: element {act}: ' +
              ' '.join(f'{S.float(k, act):.6g}' if S.active(k) >= act else '0' for k in LENS[c]))


def apply(writes, before, args, label):
    """Apply writes to a controller that holds `before` (or a prefix of the writes)."""
    states = [before]
    for w in writes:
        states.append(states[-1].appended([(w['key'], bytes.fromhex(w['bits']))]))

    print(f'{label}: {len(writes)} writes')
    for w in writes:
        print(f"  {w['key']:04x} idx {w['index']:3d} elem {w['element']}  {w['value']:14.6g}  {w.get('what', '')}")
    if args.dry_run:
        print('\nreports that would be sent (hex):')
        for w in writes:
            print('  ' + report(w)[:12].hex(' ') + ' ' + '00 ' * 3 + '...')
        return

    if os.geteuid() != 0:
        sys.exit('run as root')
    stream = Stream(find_stream())
    hid = os.open(find_hidraw(), os.O_RDWR)
    stream.drain()
    cur = stream.next_settings()
    if cur is None:
        sys.exit('no settings block from the controller within 6 s; touch the screen once to wake it and retry')
    done = next((k for k, s in enumerate(states) if s.blob == cur.blob), None)
    if done is None:
        print('The controller does not hold the settings this plan expects. Nothing written.')
        print('\n'.join('  ' + l for l in describe_diff(before, cur)[:20]))
        sys.exit(1)
    if done == len(writes):
        print('All writes are already in place.')
        return
    backup = time.strftime('settings-backup-%Y%m%d-%H%M%S.bin')
    with open(backup, 'wb') as f:
        f.write(cur.blob)
    print(f'\nbackup of the current settings: {os.path.abspath(backup)}')
    if done:
        print(f'{done} of {len(writes)} writes already applied; resuming.')
    todo = writes[done:done + 1] if args.one else writes[done:]
    if not args.yes:
        if input(f'Type WRITE to send {len(todo)} write(s) to the controller: ').strip() != 'WRITE':
            sys.exit('aborted')

    for n, w in enumerate(todo, done + 1):
        expect = states[n]
        print(f"[{n}/{len(writes)}] {w['key']:04x} elem {w['element']} = {w['value']:.6g} ... ", end='', flush=True)
        stream.drain()
        conf = send(hid, w)
        if conf is None:
            print('no confirmation')
        else:
            print(f'confirmed status {conf[8]} ({conf[:16].hex(" ")})', end=' ')
            if conf[8] != 1:
                print('\nThe controller rejected the write. Stopping.')
                sys.exit(1)
        got = None
        for _ in range(3):
            got = stream.next_settings()
            if got is not None and got.blob == expect.blob:
                break
        if got is None or got.blob != expect.blob:
            print('\nThe settings block does not show exactly the expected change. Stopping.')
            if got is not None:
                print('Changes compared with what was expected:')
                print('\n'.join('  ' + l for l in describe_diff(expect, got)[:20]))
                with open('settings-after-stop.bin', 'wb') as f:
                    f.write(got.blob)
            sys.exit(1)
        print('verified')
    if args.one and done + 1 < len(writes):
        print(f'\nOne write done and verified. Run again without --one for the remaining {len(writes) - done - 1}.')
    else:
        print('\nAll writes done and verified.')


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('show')
    p = sub.add_parser('write'); p.add_argument('plan')
    p.add_argument('--one', action='store_true'); p.add_argument('--dry-run', action='store_true')
    p.add_argument('--yes', action='store_true')
    p = sub.add_parser('revert'); p.add_argument('plan'); p.add_argument('record', type=int)
    p.add_argument('--dry-run', action='store_true'); p.add_argument('--yes', action='store_true')
    p.set_defaults(one=False)
    args = ap.parse_args()

    if args.cmd == 'show':
        if os.geteuid() != 0:
            sys.exit('run as root')
        s = Stream(find_stream()).next_settings()
        if s is None:
            sys.exit('no settings block within 6 s; touch the screen once and retry')
        show(s)
        return

    plan = json.load(open(args.plan))
    before = Settings(bytes.fromhex(plan['settings_before']))
    if args.cmd == 'write':
        apply(plan['writes'], before, args, 'apply plan')
        return

    # revert: append copies of an old record and of the lens set the plan replaced
    after = Settings(bytes.fromhex(plan['settings_after']))
    base = after
    writes = []
    for k in CAL_MEAS:
        v = before.raw(k, args.record)
        if v == b'\xff' * 4:
            sys.exit(f'record {args.record} is empty')
        writes.append(dict(key=k, value=struct.unpack('>f', v)[0], bits=v.hex(), what=f'record {args.record}'))
    lens_set = before.active(LENS[1][1])
    lw = [dict(key=k, value=before.float(k, lens_set), bits=before.raw(k, lens_set).hex(), what='camera 1 lens, original')
          for k in LENS[1] if before.active(k) == lens_set]
    lw.sort(key=lambda w: w['key'] == LENS[1][1])
    writes += lw
    s = base
    for w in writes:
        w['index'], w['type'] = s.desc[w['key']][0], s.desc[w['key']][1]
        w['element'] = s.active(w['key']) + 1
        s = s.appended([(w['key'], bytes.fromhex(w['bits']))])
    apply(writes, base, args, f'revert to record {args.record}')


if __name__ == '__main__':
    main()
