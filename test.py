#!/usr/bin/env python3
import json
import socket
import time
import threading
import statistics
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import ThreadingOSCUDPServer  # thread-safe OSC server

# ---------------- CONFIG ----------------
TARGET_IP = "127.0.0.1"
TARGET_PORT = 8686       # JSON receiver
OSC_LISTEN_PORT = 7700   # OSC replies
MESSAGE_COUNT = 500      # total JSON messages
TARGET_RATE_HZ = 1000    # messages per second
BURST_SIZE = 10          # messages per burst
DUMP_LATENCIES = True    # save per-message latencies CSV
POST_WAIT = 2.0          # seconds to wait after last send
# ----------------------------------------

lock = threading.Lock()
sent_timestamps = {}   # key: msg_id -> send timestamp ns
latencies = []         # measured latency in ns
msg_id_counter = 0

# OSC handler
def osc_handler(address, *args):
    if len(args) >= 1:
        msg_id = int(args[0])
        now = time.perf_counter_ns()
        with lock:
            sent_time = sent_timestamps.get(msg_id)
            if sent_time:
                latencies.append(now - sent_time)

# OSC listener thread
def osc_listener():
    dispatcher = Dispatcher()
    dispatcher.set_default_handler(osc_handler)
    server = ThreadingOSCUDPServer(("0.0.0.0", OSC_LISTEN_PORT), dispatcher)
    print(f"[OSC] Listening on port {OSC_LISTEN_PORT}")
    server.serve_forever()

# Send JSON messages in bursts
def send_json_messages():
    global msg_id_counter
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    payload_template = {
        "type": "android.sensor.game_rotation_vector",
        "values": [0.31892395, -0.97802734, 10.049896]
    }

    interval = 1.0 / TARGET_RATE_HZ
    start_time = time.perf_counter()
    next_send_time = start_time

    for i in range(MESSAGE_COUNT):
        msg_id = msg_id_counter
        msg_id_counter += 1

        payload = payload_template.copy()
        payload["timestamp"] = time.perf_counter_ns()
        payload["id"] = msg_id

        with lock:
            sent_timestamps[msg_id] = payload["timestamp"]

        sock.sendto(json.dumps(payload).encode("utf-8"), (TARGET_IP, TARGET_PORT))

        # maintain TARGET_RATE_HZ using bursts
        if (i + 1) % BURST_SIZE == 0:
            next_send_time += interval * BURST_SIZE
            sleep_time = next_send_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    elapsed = time.perf_counter() - start_time
    print(f"[SEND] Sent {MESSAGE_COUNT} messages in {elapsed:.3f} s "
          f"({MESSAGE_COUNT/elapsed:.1f} msg/s)")
    sock.close()

# Analyze latencies
def analyze_latencies(latencies_ns):
    if not latencies_ns:
        print("[RESULT] No OSC replies received.")
        return

    lat_ms = [x / 1e6 for x in latencies_ns]
    count = len(lat_ms)
    avg = statistics.mean(lat_ms)
    median = statistics.median(lat_ms)
    min_v = min(lat_ms)
    max_v = max(lat_ms)
    stdev = statistics.stdev(lat_ms) if count > 1 else 0.0

    print("\n[RESULT] Latency statistics:")
    print(f"  Messages received: {count}")
    print(f"  Avg latency   : {avg:.3f} ms")
    print(f"  Median latency: {median:.3f} ms")
    print(f"  Min / Max     : {min_v:.3f} / {max_v:.3f} ms")
    print(f"  Std deviation : {stdev:.3f} ms")

    if DUMP_LATENCIES:
        with open("latencies.csv", "w") as f:
            f.write("latency_ms\n")
            for l in lat_ms:
                f.write(f"{l:.6f}\n")
        print("  → Latencies saved to latencies.csv")

def main():
    # Start OSC listener
    t = threading.Thread(target=osc_listener, daemon=True)
    t.start()

    time.sleep(0.5)  # give server time to start

    # Send JSON messages in bursts
    send_json_messages()

    # Wait for OSC replies
    print(f"[MAIN] Waiting {POST_WAIT} s for remaining OSC replies...")
    time.sleep(POST_WAIT)

    # Analyze latencies
    with lock:
        analyze_latencies(latencies)

if __name__ == "__main__":
    main()

