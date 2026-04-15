"""
world_state_receiver.py - World State TCP Client (Pull Model)
Overhead Perception System
Author: Aaron Fraze
Date: April 15, 2026

Purpose:
    Runs on the Raspberry Pi aboard HamBot.
    Connects to the overhead PC server (world_state_server.py), receives
    world state JSON every frame, and caches the latest packet for your
    behavior code to pull on demand.

    This file has NO dependency on RealSense, OpenCV, or NumPy.
    Only stdlib modules: socket, json, time, threading.
    Compatible with Python 3.11.2 and all Pi-side library versions.

Usage:
    from world_state_receiver import WorldStateReceiver

    receiver = WorldStateReceiver(
        server_ip     = "192.168.1.100",
        port          = 9999,               # optional, default 9999
        on_connect    = my_connect_fn,      # optional, called with no args
        on_disconnect = my_disconnect_fn,   # optional, called with no args
    )

    # In your main loop:
    state, age = receiver.get()
    if state is not None and age < 0.5:
        robot_x = state["robot"]["x"]
        ...

    receiver.stop()

API:
    WorldStateReceiver(server_ip, port, on_connect, on_disconnect)
        Instantiation starts the background receive thread immediately.

    get() -> tuple[dict | None, float | None]
        Returns (latest_state, age_seconds) where age_seconds is the time
        elapsed since the packet was received from the server.
        Returns (None, None) if no packet has arrived yet.
        Non-blocking. Thread-safe.

    is_connected() -> bool
        Returns True if the TCP connection to the server is currently live.

    reset()
        Clears the cached packet. Subsequent get() calls return (None, None)
        until the next packet arrives from the server.

    stop()
        Signals the background thread to exit and waits for it to finish.
        Call this before your program exits.

State dict structure (matches world_state.py FrameState.to_dict()):
    {
        "timestamp": 1740339600.123,   # Unix time seconds
        "frame_id":  4821,             # monotonically increasing
        "fps":       29.8,             # server-side FPS
        "robot": {
            "detected":         true,
            "x":                45.2,  # cm, world frame
            "y":               -12.8,
            "heading_deg":     127.4,  # 0=right, 90=forward, CCW positive
            "heading_current":  true,  # False = held from last ArUco frame
            "heading_age":      0,     # frames since last fresh ArUco heading
            "source":          "aruco",# "aruco", "hsv", or "lost"
            "confidence":       0.94
        },
        "ball": {
            "detected":  true,
            "x":         80.1,
            "y":         33.5,
            "vx":       -12.3,         # cm/s, world frame
            "vy":         4.1,
            "speed":     12.98,        # cm/s magnitude
            "confidence": 0.87
        },
        "goal": {
            "x": 110.0,
            "y":   0.0
        }
    }
"""

import json
import socket
import time
import threading
from typing import Callable, Optional


# ── Connection defaults ────────────────────────────────────────────────────────

DEFAULT_PORT          = 9999
RECONNECT_DELAY_SEC   = 2.0
CONNECTION_TIMEOUT    = 5.0


# ── WorldStateReceiver ─────────────────────────────────────────────────────────

class WorldStateReceiver:
    """
    Background TCP client that caches the latest world state JSON packet.

    Starts a daemon thread on construction that connects to the overhead PC
    server, reconnects automatically on dropout, and stores each received
    packet. Your behavior code calls get() whenever it needs the latest data.

    Thread safety:
        get() and reset() are protected by a threading.Lock.
        is_connected() reads a threading.Event and is always safe to call.
    """

    def __init__(
        self,
        server_ip:     str,
        port:          int                        = DEFAULT_PORT,
        on_connect:    Optional[Callable] = None,
        on_disconnect: Optional[Callable] = None,
    ):
        """
        Args:
            server_ip:     IP address of the overhead PC running world_state_server.py
            port:          TCP port (default 9999, must match server)
            on_connect:    Optional callable invoked (no args) when connection is established
            on_disconnect: Optional callable invoked (no args) when connection is lost
        """
        self._server_ip     = server_ip
        self._port          = port
        self._on_connect    = on_connect
        self._on_disconnect = on_disconnect

        # Cached state — protected by _lock
        self._latest_state:    Optional[dict]  = None
        self._last_recv_time:  Optional[float] = None
        self._lock             = threading.Lock()

        # Connection status — threading.Event for lock-free reads
        self._connected = threading.Event()

        # Shutdown signal
        self._stop_event = threading.Event()

        # Session stats
        self._messages_received = 0
        self._connect_attempts  = 0
        self._start_time        = time.time()

        # Start background thread
        self._thread = threading.Thread(
            target  = self._run,
            name    = "WorldStateReceiver",
            daemon  = True,
        )
        self._thread.start()
        print(f"[Receiver] Started. Connecting to {server_ip}:{port}")

    # ── Public API ─────────────────────────────────────────────────────────────

    def get(self) -> tuple[Optional[dict], Optional[float]]:
        """
        Return the latest world state packet and its age in seconds.

        Returns:
            (state_dict, age_seconds) where age_seconds is time elapsed since
            the packet was received from the server.
            Returns (None, None) if no packet has arrived yet.

        Example:
            state, age = receiver.get()
            if state is not None and age < 0.5:
                robot_x = state["robot"]["x"]
        """
        with self._lock:
            if self._latest_state is None:
                return None, None
            age = time.time() - self._last_recv_time
            return self._latest_state, age

    def is_connected(self) -> bool:
        """Return True if the TCP connection to the server is currently live."""
        return self._connected.is_set()

    def reset(self):
        """
        Clear the cached packet.

        Subsequent get() calls return (None, None) until the next packet
        arrives from the server. Useful between runs or after a reset event.
        """
        with self._lock:
            self._latest_state   = None
            self._last_recv_time = None
        print("[Receiver] Cache cleared.")

    def stop(self):
        """
        Signal the background thread to stop and wait for it to finish.

        Always call this before your program exits to ensure clean shutdown.
        """
        print("[Receiver] Stopping...")
        self._stop_event.set()
        self._thread.join(timeout=5.0)
        self._print_stats()
        print("[Receiver] Stopped.")

    # ── Background thread ──────────────────────────────────────────────────────

    def _run(self):
        """Main loop: connect, receive, reconnect. Runs in background thread."""
        while not self._stop_event.is_set():
            self._connect_attempts += 1
            sock = self._try_connect()

            if sock is None:
                print(f"[Receiver] Retrying in {RECONNECT_DELAY_SEC}s...")
                self._stop_event.wait(timeout=RECONNECT_DELAY_SEC)
                continue

            print("[Receiver] Connected. Receiving world state...")
            self._connected.set()
            self._fire(self._on_connect)

            dropped = self._receive_loop(sock)

            try:
                sock.close()
            except OSError:
                pass

            self._connected.clear()
            self._fire(self._on_disconnect)

            if dropped and not self._stop_event.is_set():
                print("[Receiver] Connection lost. Reconnecting...")
                self._stop_event.wait(timeout=RECONNECT_DELAY_SEC)

    def _try_connect(self) -> Optional[socket.socket]:
        """Attempt a TCP connection. Returns socket on success, None on failure."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(CONNECTION_TIMEOUT)
            sock.connect((self._server_ip, self._port))
            sock.settimeout(None)
            return sock
        except (ConnectionRefusedError, socket.timeout, OSError) as e:
            print(f"[Receiver] Could not connect: {e}")
            return None

    def _receive_loop(self, sock: socket.socket) -> bool:
        """
        Read newline-delimited JSON from the socket until disconnected or stopped.

        Returns:
            True  if the connection dropped unexpectedly (trigger reconnect)
            False if stopped cleanly via stop()
        """
        try:
            sock_file = sock.makefile('r', encoding='utf-8')

            while not self._stop_event.is_set():
                line = sock_file.readline()

                if not line:
                    # Server closed the connection
                    return True

                line = line.strip()
                if not line:
                    continue

                try:
                    state = json.loads(line)
                except json.JSONDecodeError as e:
                    print(f"[Receiver] JSON parse error: {e}")
                    continue

                recv_time = time.time()
                with self._lock:
                    self._latest_state   = state
                    self._last_recv_time = recv_time
                self._messages_received += 1

        except (ConnectionResetError, BrokenPipeError, OSError):
            return True

        return False

    # ── Helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _fire(callback: Optional[Callable]):
        """Invoke a callback if one was provided, catching any exceptions."""
        if callback is None:
            return
        try:
            callback()
        except Exception as e:
            print(f"[Receiver] Callback error: {e}")

    def _print_stats(self):
        elapsed = time.time() - self._start_time
        print(f"\n[Receiver] Session stats:")
        print(f"  Runtime           : {elapsed:.1f}s")
        print(f"  Messages received : {self._messages_received}")
        print(f"  Connect attempts  : {self._connect_attempts}")
        if elapsed > 0 and self._messages_received > 0:
            print(f"  Avg receive rate  : {self._messages_received / elapsed:.1f} msg/s")
