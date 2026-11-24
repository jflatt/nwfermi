#!/bin/sh
# Accuracy capture: tap known targets while the raw stream is recorded.
# Run as your normal user from a terminal in the desktop session; it asks for
# the root password once, to read /dev/nwfermi0.
#
#   tools/target-capture.sh [OUTDIR] [--screen NAME]
#
# Then: python3 -I tools/target/eval.py OUTDIR
set -eu

HERE=$(cd "$(dirname "$0")/.." && pwd)
OUT=$HERE/capture/targets-$(date +%Y%m%d-%H%M%S)
case ${1:-} in
	''|-*) ;;
	*) OUT=$1; shift ;;
esac
DEV=${NWFERMI_DEV:-/dev/nwfermi0}

[ -e "$DEV" ] || { echo "$DEV not found: is nwfermi.ko loaded?"; exit 1; }
[ -n "${WAYLAND_DISPLAY:-}${DISPLAY:-}" ] || { echo "run this from the desktop session"; exit 1; }
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
rm -f "$OUT/stop"

{
	echo "date: $(date -Is)"
	echo "kernel: $(uname -r)"
	echo "module: $(grep -H . /sys/module/nwfermi/srcversion 2>/dev/null || echo '?')"
	echo "uptime: $(cut -d' ' -f1 /proc/uptime)"
} > "$OUT/info.txt"

echo "Root password, to start the stream recorder:"
su -c "nohup python3 -I '$HERE/tools/target/record.py' '$DEV' '$OUT' >'$OUT/record.log' 2>&1 &" root

i=0
until [ -s "$OUT/stream.bin" ]; do
	i=$((i + 1))
	if [ $i -gt 50 ]; then
		echo "recorder produced no data; see $OUT/record.log"
		touch "$OUT/stop"
		exit 1
	fi
	sleep 0.1
done

python3 -I "$HERE/tools/target/gui.py" "$OUT" "$@" || echo "target window exited with an error"

touch "$OUT/stop"
i=0
while [ -e "$OUT/recorder.pid" ] && [ $i -lt 30 ]; do
	sleep 0.1
	i=$((i + 1))
done
cat "$OUT/record.log"
echo "Capture in $OUT"
echo "Evaluate: python3 -I tools/target/eval.py $OUT"
