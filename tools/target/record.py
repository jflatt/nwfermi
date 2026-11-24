#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Record the raw Fermi stream from /dev/nwfermiN for a target capture.

Run as root (the device node is root only):

    record.py /dev/nwfermi0 OUTDIR

Writes OUTDIR/stream.bin in the replay format, (f64 time.time(), u32 len,
bytes)*, owned by whoever owns OUTDIR.  Stops when OUTDIR/stop appears, on
SIGINT/SIGTERM, when the device goes away, or after MAX_SECONDS.
"""
import errno, os, select, signal, struct, sys, time

MAX_SECONDS = 20 * 60

def main():
    dev, out = sys.argv[1], sys.argv[2]
    st = os.stat(out)
    stop_file = os.path.join(out, 'stop')
    path = os.path.join(out, 'stream.bin')

    # stdout/stderr are redirected to a log in OUTDIR by the caller.
    for fd in (1, 2):
        try:
            os.fchown(fd, st.st_uid, st.st_gid)
        except OSError:
            pass

    stopping = False
    def on_signal(sig, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
    f = open(path, 'wb')
    os.fchown(f.fileno(), st.st_uid, st.st_gid)
    with open(os.path.join(out, 'recorder.pid'), 'w') as p:
        p.write(f'{os.getpid()}\n')

    start = time.time()
    nbytes = nreads = 0
    last_flush = start
    print(f'recording {dev} -> {path}', flush=True)
    while not stopping and not os.path.exists(stop_file):
        now = time.time()
        if now - start > MAX_SECONDS:
            print('time limit reached', flush=True)
            break
        try:
            r, _, x = select.select([fd], [], [fd], 0.25)
        except InterruptedError:
            continue
        if x:
            print('device error, stopping', flush=True)
            break
        if not r:
            continue
        try:
            d = os.read(fd, 65536)
        except BlockingIOError:
            continue
        except OSError as e:
            print(f'read failed: {e}', flush=True)
            if e.errno in (errno.ENODEV, errno.EIO):
                break
            raise
        if not d:
            time.sleep(0.01)
            continue
        f.write(struct.pack('<dI', time.time(), len(d)) + d)
        nbytes += len(d)
        nreads += 1
        if now - last_flush > 0.5:
            f.flush()
            last_flush = now

    f.close()
    os.close(fd)
    try:
        os.unlink(os.path.join(out, 'recorder.pid'))
    except OSError:
        pass
    print(f'done: {nreads} reads, {nbytes} bytes, {time.time() - start:.1f} s',
          flush=True)

if __name__ == '__main__':
    main()
