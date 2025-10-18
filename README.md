UDP to OSC bridge for Android accelerometer
===========================================

This small Python utility listens for UDP JSON packets from an Android phone (e.g., via a sensor streaming app) and forwards accelerometer readings as OSC messages to a target host. Useful for controlling lighting/DMX or other OSC-enabled systems with phone motion.

Input (UDP JSON example)
------------------------

The bridge expects JSON payloads like:

```
{
  "type": "android.sensor.accelerometer",
  "timestamp": 3925657519043709,
  "values": [0.31892395, -0.97802734, 10.049896]
}
```

The three numbers from `values` are sent as an OSC message.

Install
-------

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Run
---

```bash
python udp_to_osc.py \
  --listen-host 0.0.0.0 \
  --listen-port 8686 \
  --dest-host 192.168.9.131 \
  --dest-port 9000 \
  --osc-address /accel \
  --print
```

Motion detection (trigger)
-------------------------

The bridge can detect sudden horizontal acceleration changes and send an OSC trigger.

Flags:

- `--motion-threshold` (default 1.5): required change in horizontal magnitude (m/s^2) to trigger.
- `--motion-cooldown` (default 0.5): minimum seconds between triggers.
- `--trigger-address` (default `/accel/trigger`): OSC path for the trigger message.

What gets sent:

- Continuous acceleration values: `osc_address` (default `/accel`) with `[x, y, z]`.
- Trigger on move: `trigger_address` with `[1, delta]` where `delta` is the magnitude change.

Example with motion detection tuned more sensitive:

```bash
python udp_to_osc.py \
  --listen-port 8686 \
  --dest-host 192.168.9.131 \
  --osc-address /accel \
  --trigger-address /accel/trigger \
  --motion-threshold 1.0 \
  --motion-cooldown 0.3 \
  --print
```

Gesture Detection
-----------------

The bridge can detect fast movements in cardinal directions from accelerometer data and send separate OSC triggers.

Flags:

- `--gesture-threshold` (default 3.0): acceleration threshold (m/s^2) to detect gestures
- `--gesture-cooldown` (default 0.5): minimum seconds between gesture triggers
- `--gesture-left-addr` (default `/left`): OSC address for left gesture
- `--gesture-right-addr` (default `/right`): OSC address for right gesture
- `--gesture-up-addr` (default `/up`): OSC address for up gesture
- `--gesture-down-addr` (default `/down`): OSC address for down gesture

Example:

```bash
python udp_to_osc.py \
  --listen-port 8686 \
  --dest-host 192.168.9.131 \
  --gesture-threshold 2.5 \
  --gesture-cooldown 0.3 \
  --print
```

Notes
-----

- The default type filter is `android.sensor.accelerometer`; only messages with this `type` are forwarded.
- The default OSC address path is `/accel` and carries three floats: x, y, z.
- If you need timestamp forwarding, uncomment the line in `udp_to_osc.py` that sends to `"/accel/ts"`.
- For DMX control, use an OSC-to-DMX gateway listening on the destination address/port.

Gyroscope to RGB (0..255)
-------------------------

You can stream gyroscope packets with type `android.sensor.gyroscope`. The bridge will map x,y,z to 0..255 and send as integers to `/rgb` by default.

Flags:

- `--gyro-type-filter` (default `android.sensor.gyroscope`)
- `--gyro-addr255` (default `/rgb`): OSC path carrying `[R,G,B]` as 0..255 ints
- `--gyro-range` (default `5.0`): expected absolute rad/s range; ±range maps to 0..255
- `--gyro-abs`: if set, uses absolute values (0..range -> 0..255). Without it, mapping is symmetric: -range..+range -> 0..255

Example:

```bash
python udp_to_osc.py \
  --listen-port 8686 \
  --dest-host 192.168.9.131 \
  --gyro-addr255 /rgb \
  --gyro-range 4.0 \
  --print
```

Game Rotation Vector to RGB (0..255)
------------------------------------

Use `android.sensor.game_rotation_vector` for steadier orientation-based control. The bridge converts the quaternion to yaw/pitch/roll (degrees) and maps to 0..255.

Flags:

- `--grv-type-filter` (default `android.sensor.game_rotation_vector`)
- `--grv-addr255` (default `/rgb`) → `[R,G,B]` from yaw,pitch,roll
- `--grv-range-deg` (default `180.0`) → ±range maps to 0..255
- `--grv-abs` → if set, uses absolute angle (0..range → 0..255)

Example:

```bash
python udp_to_osc.py \
  --listen-port 8686 \
  --dest-host 192.168.9.131 \
  --grv-addr255 /rgb \
  --grv-range-deg 90 \
  --print
```


