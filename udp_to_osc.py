#!/usr/bin/env python3
"""
UDP -> OSC bridge for Android accelerometer data.

Listens for UDP JSON payloads like:
{
  "type": "android.sensor.accelerometer",
  "timestamp": 3925657519043709,
  "values": [0.31892395,-0.97802734,10.049896]
}

Then forwards the values as an OSC message to a target host.

Default OSC address: /accel
Default listen port: 8686
Default destination host: 192.168.9.131

Usage example:
  python udp_to_osc.py --dest-host 192.168.9.131 --dest-port 9000 \
    --listen-port 8686 --osc-address /accel
"""

import argparse
import json
import socket
import sys
import threading
import math
import time
from typing import Any, Dict, List, Tuple

from pythonosc.udp_client import SimpleUDPClient


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="UDP-to-OSC bridge for Android accelerometer data",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--listen-host", default="0.0.0.0", help="UDP host to bind to")
    parser.add_argument("--listen-port", type=int, default=8686, help="UDP port to listen on")
    parser.add_argument("--dest-host", default="192.168.9.131", help="OSC destination host")
    parser.add_argument("--dest-port", type=int, default=7700, help="OSC destination port")
    parser.add_argument("--osc-address", default="/accel", help="OSC address path to send")
    parser.add_argument(
        "--type-filter",
        default="android.sensor.accelerometer",
        help="Only forward messages with this 'type' field",
    )
    parser.add_argument(
        "--print",
        dest="do_print",
        action="store_true",
        default=True,
        help="Print received and forwarded values",
    )
    parser.add_argument(
        "--recv-buf",
        type=int,
        default=65536,
        help="UDP receive buffer size in bytes",
    )
    # Motion detection options
    parser.add_argument(
        "--motion-threshold",
        type=float,
        default=1.5,
        help="Horizontal accel delta (m/s^2) to trigger motion",
    )
    parser.add_argument(
        "--motion-cooldown",
        type=float,
        default=0.5,
        help="Minimum seconds between motion triggers",
    )
    parser.add_argument(
        "--trigger-address",
        default="/accel/trigger",
        help="OSC address for motion trigger event",
    )
    # Gesture detection options
    parser.add_argument(
        "--gesture-threshold",
        type=float,
        default=3.0,
        help="Acceleration threshold (m/s^2) to detect gestures",
    )
    parser.add_argument(
        "--gesture-cooldown",
        type=float,
        default=0.5,
        help="Minimum seconds between gesture triggers",
    )
    parser.add_argument(
        "--gesture-left-addr",
        default="/left",
        help="OSC address for left gesture trigger",
    )
    parser.add_argument(
        "--gesture-right-addr",
        default="/right",
        help="OSC address for right gesture trigger",
    )
    parser.add_argument(
        "--gesture-up-addr",
        default="/up",
        help="OSC address for up gesture trigger",
    )
    parser.add_argument(
        "--gesture-down-addr",
        default="/down",
        help="OSC address for down gesture trigger",
    )
    # Gyroscope handling (map to 0..255 for RGB control)
    parser.add_argument(
        "--gyro-type-filter",
        default="android.sensor.gyroscope",
        help="Type string for gyroscope packets",
    )
    parser.add_argument(
        "--gyro-addr255",
        default="/rgb",
        help="OSC address to send 0..255 ints mapped from gyro x,y,z",
    )
    parser.add_argument(
        "--gyro-range",
        type=float,
        default=5.0,
        help="Expected |rad/s| range for scaling (maps ±range to 0..255)",
    )
    parser.add_argument(
        "--gyro-abs",
        action="store_true",
        help="Use absolute gyro values before scaling",
    )
    # Game Rotation Vector (quaternion -> yaw/pitch/roll -> 0..255)
    parser.add_argument(
        "--grv-type-filter",
        default="android.sensor.game_rotation_vector",
        help="Type string for game rotation vector packets",
    )
    parser.add_argument(
        "--grv-addr255",
        default="/rgb",
        help="OSC address to send 0..255 ints mapped from yaw,pitch,roll",
    )
    parser.add_argument(
        "--grv-range-deg",
        type=float,
        default=180.0,
        help="Angle range in degrees for scaling (±range -> 0..255)",
    )
    parser.add_argument(
        "--grv-abs",
        action="store_true",
        help="Use absolute angles before scaling (0..range -> 0..255)",
    )
    parser.add_argument(
        "--grv-yaw-circular",
        action="store_true",
        help="Map yaw circularly (0..360° -> 0..255) to avoid wrap jumps",
    )
    parser.add_argument(
        "--grv-yaw-offset-deg",
        type=float,
        default=0.0,
        help="Offset applied before circular yaw mapping",
    )
    parser.add_argument(
        "--grv-yaw-distance",
        action="store_true",
        default=True,
        help="Map yaw as minimal angle distance to offset (0..range -> 0..255)",
    )
    parser.add_argument(
        "--grv-yaw-range-deg",
        type=float,
        default=60.0,
        help="Override yaw range in degrees (±range or 0..range)",
    )
    parser.add_argument(
        "--grv-yaw-deadband-deg",
        type=float,
        default=2.0,
        help="Deadband for yaw distance mapping; values below are forced to 0",
    )
    parser.add_argument(
        "--grv-ema",
        type=float,
        default=0.8,
        help="Exponential smoothing factor for GRV angles (0..1, 0=off)",
    )
    # Per-axis pitch controls
    parser.add_argument(
        "--grv-pitch-distance",
        action="store_true",
        default=True,
        help="Map pitch as minimal angle distance to offset (0..range -> 0..255)",
    )
    parser.add_argument(
        "--grv-pitch-offset-deg",
        type=float,
        default=0.0,
        help="Pitch offset (deg) used with --grv-pitch-distance",
    )
    parser.add_argument(
        "--grv-pitch-range-deg",
        type=float,
        default=60.0,
        help="Override pitch range in degrees (±range or 0..range)",
    )
    parser.add_argument(
        "--grv-pitch-deadband-deg",
        type=float,
        default=2.0,
        help="Deadband for pitch distance mapping; values below are forced to 0",
    )
    # Per-axis roll controls
    parser.add_argument(
        "--grv-roll-distance",
        action="store_true",
        default=True,
        help="Map roll as minimal angle distance to offset (0..range -> 0..255)",
    )
    parser.add_argument(
        "--grv-roll-offset-deg",
        type=float,
        default=0.0,
        help="Roll offset (deg) used with --grv-roll-distance",
    )
    parser.add_argument(
        "--grv-roll-range-deg",
        type=float,
        default=60.0,
        help="Override roll range in degrees (±range or 0..range)",
    )
    parser.add_argument(
        "--grv-roll-deadband-deg",
        type=float,
        default=2.0,
        help="Deadband for roll distance mapping; values below are forced to 0",
    )
    # RGB addressing options
    parser.add_argument(
        "--rgb-split",
        action="store_true",
        default=True,
        help="Send R,G,B as separate OSC messages (use --addr-r/--addr-g/--addr-b)",
    )
    parser.add_argument(
        "--addr-r",
        default="/r",
        help="OSC address for Red channel when --rgb-split is set",
    )
    parser.add_argument(
        "--addr-g",
        default="/g",
        help="OSC address for Green channel when --rgb-split is set",
    )
    parser.add_argument(
        "--addr-b",
        default="/b",
        help="OSC address for Blue channel when --rgb-split is set",
    )
    parser.add_argument(
        "--log-unmatched",
        action="store_true",
        help="When --print is set, also log packet types that are unmatched",
    )
    return parser.parse_args()


def create_udp_socket(listen_host: str, listen_port: int, recv_buf: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        # On some systems, increasing the kernel receive buffer reduces drops for high rate streams.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, recv_buf)
    except OSError:
        # Not critical; continue with default buffer size.
        pass
    sock.bind((listen_host, listen_port))
    return sock


def extract_values(payload: Dict[str, Any], type_filter: str) -> Tuple[List[float], int] | None:
    if not isinstance(payload, dict):
        return None
    if payload.get("type") != type_filter:
        return None
    values = payload.get("values")
    if not isinstance(values, list) or len(values) < 3:
        return None
    try:
        x = float(values[0])
        y = float(values[1])
        z = float(values[2])
    except (ValueError, TypeError):
        return None
    ts = payload.get("timestamp")
    ts_int = int(ts) if isinstance(ts, (int, float)) else 0
    return [x, y, z], ts_int


def scale_to_0_255(value: float, rng: float, use_abs: bool) -> int:
    if use_abs:
        v = abs(value)
        # abs maps 0..rng to 0..255
        scaled = 255.0 * max(0.0, min(v, rng)) / max(rng, 1e-9)
    else:
        # symmetric mapping: [-rng, rng] -> [0, 255]
        v = max(-rng, min(value, rng))
        scaled = (v + rng) * (255.0 / (2.0 * max(rng, 1e-9)))
    return int(round(scaled))


def quaternion_to_euler_ypr(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    # Returns yaw (Z), pitch (Y), roll (X) in radians
    # Formulas from standard quaternion to Tait-Bryan Z-Y-X
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    return yaw, pitch, roll


def map_yaw_circular_deg(yaw_deg: float, offset_deg: float) -> int:
    a = (yaw_deg - offset_deg) % 360.0
    return int(round(a * (255.0 / 360.0)))


def minimal_angle_diff_deg(a: float, b: float) -> float:
    # Signed minimal difference a-b in (-180, 180]
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


def detect_gesture(accel_x: float, accel_y: float, accel_z: float, threshold: float) -> str | None:
    """Detect gesture direction based on dominant acceleration component.
    
    Returns 'left', 'right', 'up', 'down', or None if no gesture detected.
    """
    # Remove gravity bias by focusing on horizontal components
    # X: left/right, Y: up/down (assuming phone orientation)
    abs_x = abs(accel_x)
    abs_y = abs(accel_y)
    
    # Check if any component exceeds threshold
    if abs_x < threshold and abs_y < threshold:
        return None
    
    # Determine dominant direction
    if abs_x > abs_y:
        # Horizontal gesture
        if accel_x > 0:
            return 'right'
        else:
            return 'left'
    else:
        # Vertical gesture
        if accel_y > 0:
            return 'up'
        else:
            return 'down'


def run_bridge(
    listen_host: str,
    listen_port: int,
    dest_host: str,
    dest_port: int,
    osc_address: str,
    type_filter: str,
    do_print: bool,
    recv_buf: int,
    motion_threshold: float,
    motion_cooldown: float,
    trigger_address: str,
    gyro_type_filter: str,
    gyro_addr255: str,
    gyro_range: float,
    gyro_abs: bool,
    grv_type_filter: str,
    grv_addr255: str,
    grv_range_deg: float,
    grv_abs: bool,
    log_unmatched: bool,
    rgb_split: bool,
    addr_r: str,
    addr_g: str,
    addr_b: str,
    grv_yaw_circular: bool = False,
    grv_yaw_offset_deg: float = 0.0,
    grv_yaw_distance: bool = False,
    grv_yaw_range_deg: float | None = None,
    grv_yaw_deadband_deg: float = 0.0,
    grv_ema: float = 0.0,
    grv_pitch_distance: bool = False,
    grv_pitch_offset_deg: float = 0.0,
    grv_pitch_range_deg: float | None = None,
    grv_pitch_deadband_deg: float = 0.0,
    grv_roll_distance: bool = False,
    grv_roll_offset_deg: float = 0.0,
    grv_roll_range_deg: float | None = None,
    grv_roll_deadband_deg: float = 0.0,
    gesture_threshold: float = 3.0,
    gesture_cooldown: float = 0.5,
    gesture_left_addr: str = "/left",
    gesture_right_addr: str = "/right",
    gesture_up_addr: str = "/up",
    gesture_down_addr: str = "/down",
) -> None:
    udp_sock = create_udp_socket(listen_host, listen_port, recv_buf)
    osc_client = SimpleUDPClient(dest_host, dest_port)

    if do_print:
        print(
            f"Listening on {listen_host}:{listen_port} -> sending OSC to {dest_host}:{dest_port} {osc_address}",
            file=sys.stderr,
        )

    # Motion detection state
    prev_horiz_mag: float | None = None
    last_trigger_time = 0.0
    
    # Gesture detection state
    last_gesture_time = 0.0

    # GRV smoothing state (degrees)
    prev_yaw_deg: float | None = None
    prev_pitch_deg: float | None = None
    prev_roll_deg: float | None = None

    while True:
        try:
            data, addr = udp_sock.recvfrom(recv_buf)
        except OSError as e:
            print(f"Socket error: {e}", file=sys.stderr)
            break

        try:
            payload = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError:
            # Ignore malformed packets
            continue

        ptype = payload.get("type")

        # Accelerometer: forward raw values and detect motion
        if ptype == type_filter:
            result = extract_values(payload, type_filter)
            if result is None:
                continue
            values, ts = result

            try:
                osc_client.send_message(osc_address, values)
                # Optionally also send timestamp on a separate address
                # osc_client.send_message(f"{osc_address}/ts", [ts])
            except Exception as e:
                # Avoid crashing the loop on transient network errors.
                if do_print:
                    print(f"OSC send error: {e}", file=sys.stderr)
                continue

            # Motion detection: use change in horizontal magnitude (sqrt(x^2 + y^2)).
            try:
                x, y, _z = values
                horiz_mag = math.sqrt(x * x + y * y)
                if prev_horiz_mag is not None:
                    delta = abs(horiz_mag - prev_horiz_mag)
                    now = time.monotonic()
                    if delta >= motion_threshold and (now - last_trigger_time) >= motion_cooldown:
                        try:
                            # Send trigger with payload [1, delta]
                            osc_client.send_message(trigger_address, [1, float(delta)])
                            last_trigger_time = now
                            if do_print:
                                print(f"TRIGGER {trigger_address} delta={delta:.3f}")
                        except Exception as e:
                            if do_print:
                                print(f"OSC trigger send error: {e}", file=sys.stderr)
                prev_horiz_mag = horiz_mag
            except Exception:
                # If any numeric issue occurs, skip motion detection for this packet.
                pass

            # Gesture detection: detect fast movements in cardinal directions
            try:
                x, y, z = values
                gesture = detect_gesture(x, y, z, gesture_threshold)
                if gesture is not None:
                    now = time.monotonic()
                    if (now - last_gesture_time) >= gesture_cooldown:
                        try:
                            # Send gesture-specific trigger
                            if gesture == 'left':
                                osc_client.send_message(gesture_left_addr, [1])
                                if do_print:
                                    print(f"GESTURE LEFT {gesture_left_addr}")
                            elif gesture == 'right':
                                osc_client.send_message(gesture_right_addr, [1])
                                if do_print:
                                    print(f"GESTURE RIGHT {gesture_right_addr}")
                            elif gesture == 'up':
                                osc_client.send_message(gesture_up_addr, [1])
                                if do_print:
                                    print(f"GESTURE UP {gesture_up_addr}")
                            elif gesture == 'down':
                                osc_client.send_message(gesture_down_addr, [1])
                                if do_print:
                                    print(f"GESTURE DOWN {gesture_down_addr}")
                            last_gesture_time = now
                        except Exception as e:
                            if do_print:
                                print(f"OSC gesture send error: {e}", file=sys.stderr)
            except Exception:
                # If any numeric issue occurs, skip gesture detection for this packet.
                pass

            if do_print:
                print(f"{addr} -> {osc_address} {values}")

        # Gyroscope: map to 0..255 for RGB control
        elif ptype == gyro_type_filter:
            result = extract_values(payload, gyro_type_filter)
            if result is None:
                continue
            gvals, _ts = result
            try:
                r = scale_to_0_255(gvals[0], gyro_range, gyro_abs)
                g = scale_to_0_255(gvals[1], gyro_range, gyro_abs)
                b = scale_to_0_255(gvals[2], gyro_range, gyro_abs)
                if rgb_split:
                    osc_client.send_message(addr_r, [r])
                    osc_client.send_message(addr_g, [g])
                    osc_client.send_message(addr_b, [b])
                else:
                    osc_client.send_message(gyro_addr255, [r, g, b])
                if do_print:
                    if rgb_split:
                        print(f"{addr} -> {addr_r}/{addr_g}/{addr_b} R={r} G={g} B={b} (gyro x,y,z={gvals})")
                    else:
                        print(f"{addr} -> {gyro_addr255} [R={r} G={g} B={b}] (gyro x,y,z={gvals})")
            except Exception as e:
                if do_print:
                    print(f"OSC gyro send error: {e}", file=sys.stderr)
                continue

        # Game Rotation Vector: quaternion -> yaw/pitch/roll -> 0..255
        elif ptype == grv_type_filter:
            values = payload.get("values")
            if not isinstance(values, list) or len(values) < 3:
                continue
            try:
                qx = float(values[0])
                qy = float(values[1])
                qz = float(values[2])
                if len(values) >= 4:
                    qw = float(values[3])
                else:
                    # Infer w if not provided
                    w2 = max(0.0, 1.0 - (qx*qx + qy*qy + qz*qz))
                    qw = math.sqrt(w2)
            except (ValueError, TypeError):
                continue

            yaw, pitch, roll = quaternion_to_euler_ypr(qx, qy, qz, qw)
            # Convert radians to degrees for scaling with grv_range_deg
            yaw_deg = math.degrees(yaw)
            pitch_deg = math.degrees(pitch)
            roll_deg = math.degrees(roll)

            # Optional smoothing (EMA). Yaw smoothing is wrap-safe.
            if grv_ema > 0.0:
                if prev_yaw_deg is None:
                    prev_yaw_deg = yaw_deg
                else:
                    yaw_delta = minimal_angle_diff_deg(yaw_deg, prev_yaw_deg)
                    prev_yaw_deg = prev_yaw_deg + grv_ema * yaw_delta
                yaw_deg = prev_yaw_deg

                if prev_pitch_deg is None:
                    prev_pitch_deg = pitch_deg
                else:
                    prev_pitch_deg = prev_pitch_deg + grv_ema * (pitch_deg - prev_pitch_deg)
                pitch_deg = prev_pitch_deg

                if prev_roll_deg is None:
                    prev_roll_deg = roll_deg
                else:
                    prev_roll_deg = prev_roll_deg + grv_ema * (roll_deg - prev_roll_deg)
                roll_deg = prev_roll_deg
            try:
                # Red from yaw
                if grv_yaw_distance:
                    # Minimal angular distance from offset, mapped 0..range
                    dist = abs(minimal_angle_diff_deg(yaw_deg, grv_yaw_offset_deg))
                    if dist < grv_yaw_deadband_deg:
                        dist = 0.0
                    yaw_range = grv_yaw_range_deg if grv_yaw_range_deg is not None else grv_range_deg
                    r = scale_to_0_255(dist, yaw_range, True)
                elif grv_yaw_circular:
                    r = map_yaw_circular_deg(yaw_deg, grv_yaw_offset_deg)
                else:
                    r = scale_to_0_255(yaw_deg, grv_range_deg, grv_abs)

                # Green from pitch
                pitch_range = grv_pitch_range_deg if grv_pitch_range_deg is not None else grv_range_deg
                if grv_pitch_distance:
                    pdist = abs(minimal_angle_diff_deg(pitch_deg, grv_pitch_offset_deg))
                    if pdist < grv_pitch_deadband_deg:
                        pdist = 0.0
                    g = scale_to_0_255(pdist, pitch_range, True)
                else:
                    g = scale_to_0_255(pitch_deg, pitch_range, grv_abs)

                # Blue from roll
                roll_range = grv_roll_range_deg if grv_roll_range_deg is not None else grv_range_deg
                if grv_roll_distance:
                    rdist = abs(minimal_angle_diff_deg(roll_deg, grv_roll_offset_deg))
                    if rdist < grv_roll_deadband_deg:
                        rdist = 0.0
                    b = scale_to_0_255(rdist, roll_range, True)
                else:
                    b = scale_to_0_255(roll_deg, roll_range, grv_abs)
                if rgb_split:
                    osc_client.send_message(addr_r, [r])
                    osc_client.send_message(addr_g, [g])
                    osc_client.send_message(addr_b, [b])
                else:
                    osc_client.send_message(grv_addr255, [r, g, b])
                if do_print:
                    if rgb_split:
                        print(f"{addr} -> {addr_r}/{addr_g}/{addr_b} R={r} G={g} B={b} (yaw,pitch,roll deg={[round(yaw_deg,1), round(pitch_deg,1), round(roll_deg,1)]})")
                    else:
                        print(f"{addr} -> {grv_addr255} [R={r} G={g} B={b}] (yaw,pitch,roll deg={[round(yaw_deg,1), round(pitch_deg,1), round(roll_deg,1)]})")
            except Exception as e:
                if do_print:
                    print(f"OSC grv send error: {e}", file=sys.stderr)
                continue
        else:
            if do_print and log_unmatched:
                try:
                    # Give a compact hint of what arrived
                    vals = payload.get("values")
                    vinfo = f"len={len(vals)}" if isinstance(vals, list) else type(vals).__name__
                    print(f"UNMATCHED type={ptype} values={vinfo}")
                except Exception:
                    print(f"UNMATCHED type={ptype}")


def main() -> None:
    args = parse_args()

    try:
        run_bridge(
            listen_host=args.listen_host,
            listen_port=args.listen_port,
            dest_host=args.dest_host,
            dest_port=args.dest_port,
            osc_address=args.osc_address,
            type_filter=args.type_filter,
            do_print=args.do_print,
            recv_buf=args.recv_buf,
            motion_threshold=args.motion_threshold,
            motion_cooldown=args.motion_cooldown,
            trigger_address=args.trigger_address,
            gyro_type_filter=args.gyro_type_filter,
            gyro_addr255=args.gyro_addr255,
            gyro_range=args.gyro_range,
            gyro_abs=args.gyro_abs,
            grv_type_filter=args.grv_type_filter,
            grv_addr255=args.grv_addr255,
            grv_range_deg=args.grv_range_deg,
            grv_abs=args.grv_abs,
            log_unmatched=args.log_unmatched,
            rgb_split=args.rgb_split,
            addr_r=args.addr_r,
            addr_g=args.addr_g,
            addr_b=args.addr_b,
            grv_yaw_circular=args.grv_yaw_circular,
            grv_yaw_offset_deg=args.grv_yaw_offset_deg,
            grv_yaw_distance=args.grv_yaw_distance,
            grv_yaw_range_deg=args.grv_yaw_range_deg,
            grv_yaw_deadband_deg=args.grv_yaw_deadband_deg,
            grv_ema=args.grv_ema,
            grv_pitch_distance=args.grv_pitch_distance,
            grv_pitch_offset_deg=args.grv_pitch_offset_deg,
            grv_pitch_range_deg=args.grv_pitch_range_deg,
            grv_pitch_deadband_deg=args.grv_pitch_deadband_deg,
            grv_roll_distance=args.grv_roll_distance,
            grv_roll_offset_deg=args.grv_roll_offset_deg,
            grv_roll_range_deg=args.grv_roll_range_deg,
            grv_roll_deadband_deg=args.grv_roll_deadband_deg,
            gesture_threshold=args.gesture_threshold,
            gesture_cooldown=args.gesture_cooldown,
            gesture_left_addr=args.gesture_left_addr,
            gesture_right_addr=args.gesture_right_addr,
            gesture_up_addr=args.gesture_up_addr,
            gesture_down_addr=args.gesture_down_addr,
        )
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    # Run in the main thread; could be extended to asyncio if needed later.
    main()


