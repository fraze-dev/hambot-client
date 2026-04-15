# HamBot Client

Robot-side client for the HamBot Overhead Perception System.

Runs on a Raspberry Pi aboard the robot. Connects to an overhead perception
server over TCP, receives world state packets every frame, and makes them
available to your behavior code via a simple `get()` call.

---

## How the System Works

HamBot operates as part of a two-component system:

```
┌─────────────────────────────────┐         ┌──────────────────────────┐
│         Overhead PC             │   TCP   │         HamBot           │
│                                 │ ──────► │                          │
│  Ceiling-mounted camera         │  JSON   │  world_state_receiver.py │
│  Detects robots in the field    │  ~30Hz  │                          │
│  Streams world state to clients │         │  Your behavior code      │
└─────────────────────────────────┘         └──────────────────────────┘
```

**Overhead Perception Server** — runs on the lab PC connected to the ceiling
camera. Detects all robots in the field and streams a world state JSON packet
to every connected robot client. You do not need to modify or run this
yourself — the lab provides it.

**HamBot Client** (this repo) — runs on your robot's Raspberry Pi. Connects
to the server, caches the latest world state, and lets your behavior code
pull it on demand.

See [PROTOCOL.md](PROTOCOL.md) for the full packet specification.

The overhead server lives in a separate repository:
[hambot-perception-server](https://github.com/fraze-dev/hambot-perception-server)

---

## Requirements

- Python 3.11+
- Raspberry Pi running HamBot hardware drivers
- Network connection to the lab overhead PC

This repo has **no dependency on RealSense, OpenCV, or NumPy**.
All networking uses Python stdlib only: `socket`, `json`, `time`, `threading`.

---

## Installation

```bash
git clone https://github.com/fraze-dev/hambot-client.git
cd hambot-client
```

No additional packages required for `world_state_receiver.py`.

If you are running the `push_to_goal.py` example, your robot will also need
the HamBot hardware drivers and `pid_controller.py` (included in this repo).

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

| Parameter       | Type       | Default | Description                                      |
|-----------------|------------|---------|--------------------------------------------------|
| `server_ip`     | `str`      | —       | IP address of the overhead PC (required)         |
| `port`          | `int`      | `9999`  | TCP port (must match server)                     |
| `on_connect`    | `callable` | `None`  | Called (no args) when connection is established  |
| `on_disconnect` | `callable` | `None`  | Called (no args) when connection is lost         |

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
hambot-client/
├── world_state_receiver.py   # Core client — import this into your code
├── pid_controller.py         # PID controllers and steering math
├── examples/
│   └── push_to_goal.py       # Full behavior example: seek ball, push to goal
├── PROTOCOL.md               # World state JSON packet specification
└── README.md
```

---

## Examples

### `push_to_goal.py`

A complete behavior implementation using `WorldStateReceiver`. The robot
detects a ball, drives toward it, and pushes it to the goal using PID-based
heading and speed control.

```bash
python examples/push_to_goal.py --server-ip 192.168.1.100
```

Use this as a starting point for your own behavior code.

---

## Finding the Server IP

Ask your lab instructor for the overhead PC's IP address, or find it yourself:

- **Windows:** run `ipconfig` in a terminal, look for the IPv4 address on the
  WiFi adapter
- **Linux/Mac:** run `ip addr` or `ifconfig`

---

## License

MIT
