import socket
import json
import time
import threading
import statistics

# Configuration
TARGET_IP = '127.0.0.1'  # Change if the software is running on a different host
JSON_PORT = 8686        # Port where the software listens for JSON UDP
OSC_PORT = 7700         # Port where the software sends OSC UDP
NUM_PACKETS = 1000      # For sustained testing
INTERVAL_MS = 0.05      # Sub-ms interval (~10k Hz simulation)

send_times = []
receive_times = []

def sender():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (TARGET_IP, JSON_PORT)
    
    for i in range(NUM_PACKETS):
        # Generate JSON payload with current timestamp
        data = {
            "type": "android.sensor.game_rotation_vector",
            "timestamp": time.time_ns(),
            "values": [0.31892395, -0.97802734, 10.049896]  # Fixed values; can randomize if needed
        }
        payload = json.dumps(data).encode('utf-8')
        
        start_time = time.perf_counter()
        sock.sendto(payload, addr)
        send_times.append(start_time)
        
        # Busy-wait for precise sub-ms interval
        target_time = time.perf_counter() + (INTERVAL_MS / 1000.0)
        while time.perf_counter() < target_time:
            pass  # Spin loop for finer timing
    
    sock.close()

def listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)  # Increase buffer to 1MB
    sock.bind(('0.0.0.0', OSC_PORT))
    sock.settimeout(5.0)  # Timeout for exit if loss
    
    received = 0
    while received < NUM_PACKETS:
        try:
            data, addr = sock.recvfrom(1024)
            end_time = time.perf_counter()
            receive_times.append(end_time)
            received += 1
            if received % 100 == 0:  # Print every 100 to avoid I/O slowdown
                print(f"Received {received} OSC packets so far...")
            # Optional: Parse OSC (requires 'python-osc' package)
            # from pythonosc.osc_message_parser import parse_message
            # address, typetag, args = parse_message(data)
            # print(f"OSC: {address} {args}")
        except socket.timeout:
            print("Timeout waiting for OSC packet; possible packet loss.")
            break
    
    sock.close()

if __name__ == "__main__":
    # Start listener thread first
    listener_thread = threading.Thread(target=listener)
    listener_thread.start()
    
    # Short delay to ensure bound
    time.sleep(0.1)
    
    # Start sender
    sender_thread = threading.Thread(target=sender)
    sender_thread.start()
    
    # Wait for completion
    sender_thread.join()
    listener_thread.join()
    
    # Calculate latencies if matched
    if len(receive_times) == len(send_times):
        latencies = [(r - s) * 1000 for r, s in zip(receive_times, send_times)]  # ms
        
        print("\nLatency Measurements (ms):")
        print(f"Min: {min(latencies):.3f}")
        print(f"Max: {max(latencies):.3f}")
        print(f"Average: {statistics.mean(latencies):.3f}")
        print(f"Standard Deviation: {statistics.stdev(latencies):.3f}")
    else:
        print(f"\nMismatch: Sent {len(send_times)}, Received {len(receive_times)}. Possible packet loss or multiple OSC per JSON.")
