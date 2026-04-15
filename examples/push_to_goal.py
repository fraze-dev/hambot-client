"""
push_to_goal.py - Example Behavior: Seek Ball and Push to Goal
HamBot Client
Author: Aaron Fraze
Date: April 15, 2026

Purpose:
    Demonstrates how to use WorldStateReceiver in a behavior loop.
    The robot detects a ball, drives toward it using PID heading and
    speed control, then pushes it to the goal.

    This file is intended as a starting point -- copy and adapt it
    for your own behavior code.

Usage:
    python examples/push_to_goal.py --server-ip 192.168.1.100

    Arguments:
        --server-ip   IP address of the overhead PC (required)
        --port        Port number (default 9999, must match server)
        --marker-id   Your robot's ArUco marker ID (default 1)

Hardware:
    Requires HamBot robot with hardware drivers installed.
    See your lab documentation for driver setup.
"""

import argparse
import time

from world_state_receiver import WorldStateReceiver
from pid_controller import (
    HeadingPID, SpeedPID,
    mix_to_wheel_speeds,
    angle_to_target,
    heading_error,
    distance_to,
)

# Import your robot hardware driver
# Adjust this import to match your hardware driver repo
from robot import HamBot


# ── Connection settings ───────────────────────────────────────────────────────

DEFAULT_PORT      = 9999
DEFAULT_MARKER_ID = 1


# ── Behavior tuning constants ─────────────────────────────────────────────────

# Distance thresholds (cm)
BALL_CONTACT_DIST    = 18.0   # enter push mode when this close to ball
BALL_LOST_DIST       = 35.0   # exit push mode if ball gets further than this
GOAL_REACHED_DIST    = 15.0   # stop when ball is this close to goal

# Speed limits (RPM) per phase
SEEK_MAX_RPM         = 25.0   # top speed while driving toward ball
PUSH_MAX_RPM         = 15.0   # slow push — keeps ball in scoop
MIN_BASE_RPM         = 5.0    # always move a little even when heading is off

# Approach offset — aim slightly north of ball so it arrives centered in scoop
APPROACH_OFFSET_CM   = 6.0

# Heading staleness
STALE_HEADING_AGE    = 10     # frames before heading is considered unreliable

# Dropout tolerance — consecutive missed frames before reacting
ROBOT_DROPOUT_FRAMES = 5
BALL_DROPOUT_FRAMES  = 30     # ~1 second at 30fps

# Backup behavior when ball is truly lost
# Robot may be sitting on top of the ball blocking camera view
BACKUP_RPM           = -12.0  # negative = reverse
BACKUP_DURATION_SEC  = 0.75

# Data staleness threshold — ignore packets older than this
MAX_PACKET_AGE_SEC   = 0.5


# ── Behavior ──────────────────────────────────────────────────────────────────

class PushToGoalBehavior:
    """
    PID-driven ball-seeking and ball-pushing behavior.

    State machine:
        waiting        -- not connected, motors stopped
        lost           -- robot not detected by overhead system
        reacquiring    -- heading stale, robot slows to let ArUco reacquire
        searching_ball -- robot detected but ball not visible
        seeking_ball   -- drive toward ball using heading + speed PIDs
        pushing_ball   -- ball in scoop, drive toward goal
        done           -- ball reached goal, stopped

    Usage:
        behavior = PushToGoalBehavior(bot, marker_id=42, goal_x=110.0, goal_y=0.0)
        behavior.on_connect()
        while running:
            state, age = receiver.get()
            behavior.update(state, age)
        behavior.on_disconnect()
    """

    def __init__(self, bot: HamBot, marker_id: int, goal_x: float, goal_y: float):
        self.bot        = bot
        self.marker_id  = str(marker_id)
        self.goal_x     = goal_x
        self.goal_y     = goal_y

        self.state_name = 'waiting'
        self._last_print_t = 0.0

        # Dropout counters
        self._robot_missing_frames = 0
        self._ball_missing_frames  = 0

        # Held ball position for brief dropouts
        self._last_ball_x: float = 0.0
        self._last_ball_y: float = 0.0

        # Locked heading for push phase
        self._locked_push_heading = None

        # Backup timer for searching_ball
        self._backup_start_time = None

        HeadingPID.reset()
        SpeedPID.reset()

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def on_connect(self):
        print("[Behavior] Connected to overhead system")
        self.state_name = 'seeking_ball'
        self._locked_push_heading = None
        HeadingPID.reset()
        SpeedPID.reset()
        self.bot.stop_motors()

    def on_disconnect(self):
        print("[Behavior] Lost connection — stopping motors")
        self.state_name = 'waiting'
        self.bot.stop_motors()

    # ── Main update ───────────────────────────────────────────────────────────

    def update(self, state: dict, age: float):
        """
        Call this every loop iteration with the latest packet from receiver.get().

        Args:
            state: World state dict from receiver.get()
            age:   Seconds since packet was received from the server
        """
        if state is None or age > MAX_PACKET_AGE_SEC:
            return

        # ── Unpack state ──────────────────────────────────────────────────────
        robots = state.get('robots', {})
        robot  = robots.get(self.marker_id, {})
        ball   = state.get('ball', {})
        goal_x = self.goal_x
        goal_y = self.goal_y

        robot_detected  = robot.get('detected',        False)
        robot_x         = robot.get('x',               0.0)
        robot_y         = robot.get('y',               0.0)
        heading         = robot.get('heading_deg',     0.0)
        heading_current = robot.get('heading_current', False)
        heading_age     = robot.get('heading_age',     0)

        ball_detected   = ball.get('detected',         False)
        ball_x          = ball.get('x',                self._last_ball_x)
        ball_y          = ball.get('y',                self._last_ball_y)

        server_fps      = state.get('fps', 0.0)

        heading_stale = (not heading_current) and (heading_age > STALE_HEADING_AGE)

        # ── Dropout-tolerant flags ────────────────────────────────────────────
        if robot_detected:
            self._robot_missing_frames = 0
        else:
            self._robot_missing_frames += 1

        if ball_detected:
            self._ball_missing_frames = 0
            self._last_ball_x = ball_x
            self._last_ball_y = ball_y
        else:
            self._ball_missing_frames += 1

        robot_truly_lost = self._robot_missing_frames >= ROBOT_DROPOUT_FRAMES
        ball_truly_gone  = self._ball_missing_frames  >= BALL_DROPOUT_FRAMES

        # ── State transitions ─────────────────────────────────────────────────

        if robot_truly_lost:
            self._transition('lost')
            self.bot.stop_motors()
            self._log(heading, robot_x, robot_y, robot_detected,
                      heading_current, heading_age,
                      self._last_ball_x, self._last_ball_y,
                      ball_detected, server_fps)
            return

        if not robot_detected:
            self._log(heading, robot_x, robot_y, robot_detected,
                      heading_current, heading_age,
                      self._last_ball_x, self._last_ball_y,
                      ball_detected, server_fps)
            return

        if heading_stale:
            self._transition('reacquiring')
            left, right = mix_to_wheel_speeds(base_rpm=10.0, steering_rpm=0.0)
            self.bot.set_left_motor_speed(left)
            self.bot.set_right_motor_speed(right)
            self._log(heading, robot_x, robot_y, robot_detected,
                      heading_current, heading_age,
                      self._last_ball_x, self._last_ball_y,
                      ball_detected, server_fps)
            return

        if ball_truly_gone:
            self._transition('searching_ball')
            if self._backup_start_time is None:
                self._backup_start_time = time.time()
            if time.time() - self._backup_start_time < BACKUP_DURATION_SEC:
                left, right = mix_to_wheel_speeds(base_rpm=BACKUP_RPM, steering_rpm=0.0)
                self.bot.set_left_motor_speed(left)
                self.bot.set_right_motor_speed(right)
            else:
                self.bot.stop_motors()
            self._log(heading, robot_x, robot_y, robot_detected,
                      heading_current, heading_age,
                      self._last_ball_x, self._last_ball_y,
                      ball_detected, server_fps)
            return

        # ── Full state available — run PID ────────────────────────────────────

        ball_x = self._last_ball_x
        ball_y = self._last_ball_y

        dist_to_ball = distance_to(robot_x, robot_y, ball_x, ball_y)
        dist_to_goal = distance_to(ball_x, ball_y, goal_x, goal_y)

        if self.state_name == 'pushing_ball' and dist_to_goal < GOAL_REACHED_DIST:
            self._transition('done')
            self.bot.stop_motors()
            print("[Behavior] 🎉 Ball reached the goal!")
            return

        if self.state_name == 'done':
            return

        # Phase selection with hysteresis
        if self.state_name == 'pushing_ball':
            if dist_to_ball > BALL_LOST_DIST:
                self._locked_push_heading = None
                self._transition('seeking_ball')
        else:
            if dist_to_ball <= BALL_CONTACT_DIST:
                self._locked_push_heading = angle_to_target(
                    robot_x, robot_y, goal_x, goal_y)
                self._transition('pushing_ball')
            else:
                self._transition('seeking_ball')

        # Desired heading
        if self.state_name == 'seeking_ball':
            desired_heading = angle_to_target(
                robot_x, robot_y,
                ball_x,  ball_y + APPROACH_OFFSET_CM)
            speed_cap = SEEK_MAX_RPM
        else:
            desired_heading = self._locked_push_heading
            speed_cap = PUSH_MAX_RPM

        # Heading PID → steering
        h_error      = heading_error(heading, desired_heading)
        steering_rpm = HeadingPID.compute(h_error)

        # Speed PID → forward drive
        if self.state_name == 'seeking_ball':
            target_dist = dist_to_ball
        else:
            target_dist = distance_to(robot_x, robot_y, goal_x, goal_y)

        raw_base_rpm     = SpeedPID.compute(target_dist)
        alignment_factor = max(MIN_BASE_RPM / speed_cap,
                               1.0 - abs(h_error) / 180.0)
        base_rpm         = raw_base_rpm * alignment_factor
        base_rpm         = max(MIN_BASE_RPM, min(base_rpm, speed_cap))

        # Send to motors
        left_rpm, right_rpm = mix_to_wheel_speeds(base_rpm, steering_rpm)
        self.bot.set_left_motor_speed(left_rpm)
        self.bot.set_right_motor_speed(right_rpm)

        self._log(heading, robot_x, robot_y, robot_detected,
                  heading_current, heading_age,
                  ball_x, ball_y, ball_detected, server_fps,
                  extra=f"h_err={h_error:+.1f}° base={base_rpm:.1f} "
                        f"steer={steering_rpm:.1f} dist={dist_to_ball:.1f}cm")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _transition(self, new_state: str):
        if new_state == self.state_name:
            return
        print(f"[Behavior] {self.state_name} → {new_state}")
        phase_change = (
            (self.state_name == 'seeking_ball' and new_state == 'pushing_ball') or
            (self.state_name == 'pushing_ball' and new_state == 'seeking_ball')
        )
        if phase_change:
            HeadingPID.reset()
            SpeedPID.reset()
        if self.state_name == 'searching_ball':
            self._backup_start_time = None
        self.state_name = new_state

    def _log(self, heading, rx, ry, r_det, hdg_cur, hdg_age,
             bx, by, b_det, fps, extra=''):
        now = time.time()
        if now - self._last_print_t < 1.0:
            return
        self._last_print_t = now
        robot_str = f"({rx:.1f},{ry:.1f})" if r_det else "LOST"
        ball_str  = f"({bx:.1f},{by:.1f})" if b_det else "none"
        hdg_str   = f"{heading:.1f}° {'live' if hdg_cur else f'held+{hdg_age}'}"
        print(
            f"[Behavior] {self.state_name:18s} | "
            f"robot={robot_str:14s} | hdg={hdg_str:16s} | "
            f"ball={ball_str:14s} | {fps:.1f}fps"
            + (f" | {extra}" if extra else "")
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description='HamBot Push-to-Goal Behavior')
    p.add_argument('--server-ip',  required=True,
                   help='IP address of the overhead PC (e.g. 192.168.1.100)')
    p.add_argument('--port',       default=DEFAULT_PORT,      type=int,
                   help=f'Server port (default {DEFAULT_PORT})')
    p.add_argument('--marker-id',  default=DEFAULT_MARKER_ID, type=int,
                   help=f'Your ArUco marker ID (default {DEFAULT_MARKER_ID})')
    p.add_argument('--goal-x',     default=110.0,             type=float,
                   help='Goal X position in cm (default 110.0)')
    p.add_argument('--goal-y',     default=0.0,               type=float,
                   help='Goal Y position in cm (default 0.0)')
    return p.parse_args()


if __name__ == '__main__':
    args = parse_args()

    print("=" * 60)
    print("HamBot Push-to-Goal")
    print("=" * 60)
    print(f"  Server    : {args.server_ip}:{args.port}")
    print(f"  Marker ID : {args.marker_id}")
    print(f"  Goal      : ({args.goal_x}, {args.goal_y})")
    print("=" * 60)
    print()

    bot      = HamBot(lidar_enabled=False, camera_enabled=False)
    behavior = PushToGoalBehavior(
        bot       = bot,
        marker_id = args.marker_id,
        goal_x    = args.goal_x,
        goal_y    = args.goal_y,
    )

    receiver = WorldStateReceiver(
        server_ip     = args.server_ip,
        port          = args.port,
        on_connect    = behavior.on_connect,
        on_disconnect = behavior.on_disconnect,
    )

    try:
        while True:
            state, age = receiver.get()
            behavior.update(state, age)
            time.sleep(0.01)   # ~100Hz poll — server sends ~30Hz so we never miss a frame

    except KeyboardInterrupt:
        print("\n[Main] Stopped by user.")

    finally:
        receiver.stop()
        bot.stop_motors()
        bot.disconnect_robot()
