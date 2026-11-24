#!/bin/bash
# Record what nwfermi_daemon reads from the touchscreen and what it reports,
# so the processing can be reimplemented and checked against it.
# Run as root soon after boot, while touch is still working:
#   su -c ~/nwfermi/tools/capture.sh
set -u

HERE=$(dirname "$(readlink -f "$0")")
OUT=${OUT:-$HERE/../capture}
OWNER=$(stat -c %U "$HERE")

[ "$(id -u)" = 0 ] || { echo "run as root"; exit 1; }
PID=$(pgrep -o -f nwfermi_daemon) || { echo "nwfermi_daemon is not running"; exit 1; }

rm -rf "$OUT"; mkdir -p "$OUT"
{
	echo "uptime: $(cat /proc/uptime)"
	echo "daemon: $(ps -o pid,etime,time,args -p "$PID" | tail -1)"
	echo "fwprod: $(pgrep -a fwprod || echo 'not running')"
} > "$OUT/state.txt"
dmesg | grep -i -E 'fermi|1926|usb 4-' | tail -50 > "$OUT/dmesg.txt"
cat "$OUT/state.txt"

step() { printf '\n>>> %s  (%ss)\n' "$1" "$2"; echo "MARK $(date +%H:%M:%S.%N) $1" >> "$OUT/marks.txt"; sleep "$2"; }

echo "Attaching strace to the running daemon (pid $PID); it is not restarted..."
strace -f -tt -xx -s 70000 -e trace=read,write -p "$PID" -o "$OUT/strace.txt" &
TRACER=$!
sleep 2

step "Do NOT touch the screen" 5
step "Touch and HOLD one finger near the TOP-LEFT corner" 4
step "Lift. No touch" 2
step "Touch and HOLD near the TOP-RIGHT corner" 4
step "Lift. No touch" 2
step "Touch and HOLD near the BOTTOM-RIGHT corner" 4
step "Lift. No touch" 2
step "Touch and HOLD near the BOTTOM-LEFT corner" 4
step "Lift. No touch" 2
step "Touch and HOLD the CENTRE" 4
step "Lift. No touch" 2
step "Slowly drag one finger LEFT to RIGHT across the middle" 5
step "Lift. No touch" 2
step "Hold TWO fingers apart (left half and right half)" 5
step "Lift. No touch" 3

echo "Detaching strace (the daemon keeps running)..."
kill -INT "$TRACER"; wait "$TRACER" 2>/dev/null

lsusb -v -d 1926: > "$OUT/lsusb.txt" 2>&1
chown -R "$OWNER" "$OUT"
echo "Done: $(du -sh "$OUT" | cut -f1) in $OUT"
