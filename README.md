# nwfermi: NextWindow Fermi touchscreen driver for Wayland/libinput

A single kernel module replaces the whole vendor stack:

| Old | New |
|---|---|
| `nw-fermi.ko`, a pipe to the daemon that reported MT protocol A | `nwfermi.ko`: MT protocol B, `INPUT_PROP_DIRECT`, a real USB parent |
| `nwfermi_daemon`, a closed 32-bit i386 binary that computed the touches | The in-kernel engine (`nwfermi_engine.c`), using integer maths only |
| `fwprod`, re-sending a keep-alive over a **world-writable** hidraw node | Sent by the driver (`keepalive_interval=`) |
| `nextwindow_drv.so` (Xorg) | Not needed. libinput reads the evdev device directly. |

## How it works

The controller has two line-scan cameras in the top corners, looking across
a lit, reflective bezel. For each camera it streams one waveform with its
LEDs off and one with them on. When idle it sends one of each per second,
and about 130 per second while the screen is touched.

1. **Shadows:** the engine subtracts the LED-off frame from the LED-on frame
   and compares the result with an adaptive baseline. Each shadow's
   sub-pixel centre is its weighted centroid.
2. **Angles:** each camera's own lens polynomial (stored in the controller's
   settings) converts pixels to angles. The camera's mounting
   rotation is measured from the far bottom corner of the bezel, which shows
   up as a sharp bright-to-dark step in its baseline. The line between the
   two cameras is slightly tilted against the screen. The tilt is read from
   the controller's stored calibration, and it opens the corner angle for
   one camera and closes it for the other.
3. **Position:** the two rays are intersected, and the controller's stored
   4-point calibration maps the result to 0–32767.

The controller keeps its lens coefficients and calibration as append-only
records in flash. A new value goes into the next empty slot, and the last
one written is in force. The engine uses the newest record, as the vendor
software did.
4. **Two fingers:** this produces two real points and two "ghosts". Fingers
   rarely land in the same frame, so the engine keeps the finger it is
   already tracking and picks the pairing that contains it. While only one
   camera sees the new finger, the engine waits a few frames instead of
   guessing. For two fingers that land together, it picks the pairing where
   both fingers are about the same size.

Measured against known target positions on a 510 × 287 mm panel
(`tools/target-capture.sh`, then `tools/target/eval.py`). The first set of
figures uses the factory calibration. The second was measured after
recalibrating the controller with `tools/target/fit.py` and `controller.py`
(see below), in a session the fit had not seen.

Holding still, the position varies by about 0.1 mm. Finger placement itself
varies by about 0.5 mm from one session to the next. The vendor daemon was
about 3–4 mm off, about the same as earlier versions of this engine. The
engine also reports touches in the bottom corners, where the daemon reported
none.

Known limitations:
- Two fingers in line with one camera merge into one shadow for that
  camera, so the farther finger can be 15–20 mm off along the other
  camera's line of sight.

## Build

    make            # nwfermi.ko for the running kernel
    make checkpatch # checkpatch.pl --strict: clean
    make replay     # userspace build of the engine for testing

## Install (as root)

    # stop and remove the old stack
    pkill nwfermi_daemon; pkill fwprod
    rmmod nw_fermi
    rm /lib/modules/$(uname -r)/extra/nw-fermi.ko
    rm /etc/udev/rules.d/40-nw-fermi.rules /etc/X11/xorg.conf.d/10-nwfermi.conf
    rm -f /usr/sbin/nwfermi_daemon /usr/bin/nwfermi_daemon /usr/sbin/fwprod /usr/bin/fwprod
    rm -f /usr/lib64/xorg/modules/input/nextwindow_drv.* \
          /usr/local/lib/xorg/modules/input/nextwindow_drv.*

    # install the new driver
    make install
    udevadm control --reload
    modprobe nwfermi

## Verify

    dmesg | grep -i fermi                    # "...on nwfermi0"
    libinput list-devices | grep -A8 Fermi    # Capabilities: touch
    libinput debug-events                    # touch the screen

Module parameters (`modinfo nwfermi`):

- `keepalive_interval=N` sets the keep-alive period in seconds (0 disables it).

## Diagnostics

`/dev/nwfermi0` (root only) delivers the raw camera stream.

### Accuracy capture

To measure accuracy against known positions, run this from a terminal in the
desktop session as your normal user, preferably soon after a reboot:

    tools/target-capture.sh                  # asks for the root password once
    python3 -I tools/target/eval.py capture/targets-<date>

A full-screen window (PyQt6) shows about 50 steps, which take 3 to 4 minutes:

- 36 single-finger holds: a grid, the edges and corners, and near the cameras
- 8 slow drags along straight lines
- 6 two-finger holds

Hold each cross until its ring fills, then lift. If the driver doesn't
report the touch, the step falls back to a timed hold after 6 s (Space does
this at once). Backspace redoes the previous step and Esc quits. A root
recorder saves the raw stream alongside the window's log, and `eval.py`
replays it through the engine and reports the error in mm for each target,
region and drag. Use `--screen NAME` to choose the display.

### Recalibrating the controller

The factory calibration was measured by the vendor's software, using camera
angles that differ slightly from this engine's. A new calibration can be
fitted to target captures and stored in the controller:

    python3 -I tools/target/fit.py capture/calibration-plan.json capture/targets-*
    su -c 'python3 -I tools/target/controller.py show'
    su -c 'python3 -I tools/target/controller.py write capture/calibration-plan.json --one'
    su -c 'python3 -I tools/target/controller.py write capture/calibration-plan.json'

`fit.py` fits a new 4-point calibration record and rescales camera 1's lens
coefficients. It checks each capture held out in turn, then replays the
captures with the new records patched into the settings, and writes a plan.

`controller.py` sends the writes as HID output reports on the controller's
vendor HID interface, using the same format as the vendor tools:
`'S' 's' index:u16 type:u8 1 value:be32`. It refuses to write unless the
controller holds exactly the settings the plan was made from, saves a
backup, and checks each write against the settings the controller sends
back. Each write fills an empty slot, so the factory records are kept.
`controller.py revert PLAN 1` (or `2`) makes a factory record current again
by appending a copy of it.

The original settings of the unit this was developed on are in
`capture/controller-settings-original.bin`. The new records survive a
power cycle.

## Compositor notes

* **Output mapping:** GNOME and KDE map a touchscreen to a display in their
  settings. In sway, use `input type:touch map_to_output <OUTPUT>`.
* **Calibration:** the controller's stored calibration is applied
  automatically. To recalibrate, see "Recalibrating the controller" above.
