# robot_tracking_client

Robot-side client for the HamBot Overhead Perception System.

Runs on a Raspberry Pi aboard the robot. Connects to an overhead perception
server over TCP, receives world state packets every frame, and makes them
available to your behavior code via a simple `get()` call.

---

## How the System Works

HamBot operates as a two-component system:

```
┌──────────────────────────────────┐         ┌──────────────────────────────────────┐
│          Jetson Nano             │   TCP   │              HamBot                  │
│                                  │ ──────► │                                      │
│  Ceiling-mounted RealSense D435  │  JSON   │  world_state_receiver.py             │
│  Detects robots in the field     │  ~30Hz  │    background thread caches latest   │
│  Streams world state to clients  │         │                                      │
│                                  │         │  receiver.get()  ← behavior code    │
└──────────────────────────────────┘         └──────────────────────────────────────┘
```

**Overhead Perception Server** — runs on the Jetson Nano connected to the ceiling
camera. Detects all robots in the field and streams a world state JSON packet
to every connected robot client. You do not need to modify or run this
yourself — the lab provides it.

**robot_tracking_client** (this repo) — runs on your robot's Raspberry Pi. A
background thread connects to the server and caches the latest world state.
Your behavior code calls `receiver.get()` on demand to pull the current state.

See [PROTOCOL.md](PROTOCOL.md) for the full packet specification.

The overhead server lives in a separate repository:
[robot_tracking_server](https://github.com/biorobaw/robot_tracking_server.git)

---

## Requirements

- Python 3.11+
- Raspberry Pi running HamBot hardware drivers
- Network connection to the lab Jetson Nano

This repo has **no dependency on RealSense, OpenCV, or NumPy**.
All networking uses Python stdlib only: `socket`, `json`, `time`, `threading`.

---

## Installation

```bash
git clone https://github.com/biorobaw/robot_tracking_client.git
cd robot_tracking_client
```

No additional packages required for `world_state_receiver.py`.

If you are using `pid_controller.py`, your robot will also need
the HamBot hardware drivers (included separately).

---

## Quick Start

```python
from world_state_receiver import WorldStateReceiver
import time

receiver = WorldStateReceiver(server_ip="192.168.1.100")

try:
    while True:
        state, age = receiver.get()

        if state is None:
            time.sleep(0.01)
            continue

        if age > 0.5:
            # Data is stale — take a safe action
            continue

        # Use the world state
        robot = state["robots"]["42"]   # your robot's ArUco ID
        print(f"Position: ({robot['x']:.1f}, {robot['y']:.1f})")

finally:
    receiver.stop()
```

---

## API Reference

### `WorldStateReceiver(server_ip, port, on_connect, on_disconnect)`

Instantiating the class starts the background receive thread immediately.

| Parameter       | Type       | Default | Description                                       |
|-----------------|------------|---------|---------------------------------------------------|
| `server_ip`     | `str`      | —       | IP address of the Jetson Nano (required)          |
| `port`          | `int`      | `9999`  | TCP port (must match server)                      |
| `on_connect`    | `callable` | `None`  | Called (no args) when connection is established   |
| `on_disconnect` | `callable` | `None`  | Called (no args) when connection is lost          |

### `get() → (dict | None, float | None)`

Returns `(state, age_seconds)` where `age_seconds` is the time elapsed since
the packet was received from the server. Returns `(None, None)` if no packet
has arrived yet. Non-blocking. Thread-safe.

### `is_connected() → bool`

Returns `True` if the TCP connection to the server is currently live.

### `reset()`

Clears the cached packet. Subsequent `get()` calls return `(None, None)`
until the next packet arrives. Useful between runs.

### `stop()`

Signals the background thread to exit cleanly and waits for it to finish.
Always call this before your program exits.

---

## Repository Structure

```
robot_tracking_client/
├── world_state_receiver.py   # Core client — import this into your code
├── pid_controller.py         # PID controllers and steering math
├── client_example.py         # Minimal usage example
├── PROTOCOL.md               # World state JSON packet specification
└── README.md
```

---

## Finding the Server IP

Ask your lab instructor for the Jetson Nano's IP address, or find it on the
Jetson Nano itself:

```
ip addr
# Look for the IP on your WiFi or Ethernet adapter
```

---

## License

MIT
