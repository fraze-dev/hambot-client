"""
pid_controller.py - PID Controller for HamBot Navigation
Overhead Perception System
Author: Aaron Fraze
Date: March 30, 2026

Purpose:
    Provides a generic PIDController class and two pre-configured
    controllers used by HambotBehavior:

        HeadingPID   -- minimizes angular error (turn toward target)
        SpeedPID     -- minimizes distance error (drive toward target)

    Both controllers output RPM deltas that are applied to the left/right
    motors via differential steering.

Differential steering model (tank drive):
    base_rpm        -- both wheels at this speed → drives straight
    steering_rpm    -- added to one wheel, subtracted from the other → turns

    left_rpm  = base_rpm + steering_rpm
    right_rpm = base_rpm - steering_rpm

    Positive steering_rpm → turns LEFT  (CCW, matches heading convention)
    Negative steering_rpm → turns RIGHT (CW)

Motor limits:
    HamBot motors are capped at ±75 RPM by robot.py's check_speed().
    Outputs from this module are clamped to that range before use.

Tuning guide (start here, adjust on the floor):
    Kp  -- proportional gain. Increase if response feels sluggish,
            decrease if the robot oscillates or overshoots.
    Ki  -- integral gain. Helps correct steady-state offset (robot stops
            slightly off-target). Start at 0, add small amounts only.
    Kd  -- derivative gain. Damps oscillation. Increase if robot
            overshoots and wiggles. Too high → noisy/jittery.

    Recommended first-pass tuning order:
        1. Set Ki=0, Kd=0. Raise Kp until robot tracks but oscillates.
        2. Back Kp off ~20%. Add Kd until oscillation damps out.
        3. Only add Ki if there's a persistent steady-state error.
"""

import math
import time


# ── Motor limits ──────────────────────────────────────────────────────────────

RPM_MAX   =  75.0   # hard cap from robot.py check_speed()
RPM_MIN   = -75.0


# ── Generic PID controller ────────────────────────────────────────────────────

class PIDController:
    """
    Discrete PID controller with integral wind-up clamp and
    angular-error wrapping for heading control.

    Args:
        kp (float):          Proportional gain
        ki (float):          Integral gain
        kd (float):          Derivative gain
        output_limit (float): Symmetric clamp on output  (|output| ≤ limit)
        integral_limit (float): Symmetric clamp on integral accumulator
        angular (bool):      If True, wraps error to -180..180 (for heading)
    """

    def __init__(
        self,
        kp:             float,
        ki:             float,
        kd:             float,
        output_limit:   float = 75.0,
        integral_limit: float = 30.0,
        angular:        bool  = False,
    ):
        self.kp             = kp
        self.ki             = ki
        self.kd             = kd
        self.output_limit   = output_limit
        self.integral_limit = integral_limit
        self.angular        = angular

        self._integral:    float = 0.0
        self._last_error:  float = 0.0
        self._last_time:   float = 0.0
        self._initialized: bool  = False

    # ── Public API ────────────────────────────────────────────────────────────

    def compute(self, error: float) -> float:
        """
        Compute PID output for the given error.

        Args:
            error: setpoint − measured_value.
                   For heading control pass the already-computed
                   angular error (wrapped to -180..180).

        Returns:
            float: Controller output, clamped to ±output_limit.
        """
        now = time.monotonic()

        if not self._initialized:
            # First call -- no derivative yet, just prime the state
            self._last_error = error
            self._last_time  = now
            self._initialized = True
            dt = 0.0
        else:
            dt = now - self._last_time

        # Wrap angular errors so we always take the short way around
        if self.angular:
            error = _wrap_angle(error)

        # ── P ────────────────────────────────────────────────────────────────
        p_term = self.kp * error

        # ── I ────────────────────────────────────────────────────────────────
        if dt > 0:
            self._integral += error * dt
            # Anti-windup: clamp integral accumulator
            self._integral = _clamp(self._integral, self._integral_limit)
        i_term = self.ki * self._integral

        # ── D ────────────────────────────────────────────────────────────────
        if dt > 0:
            derivative = (error - self._last_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative

        # ── Combine and clamp ────────────────────────────────────────────────
        output = p_term + i_term + d_term
        output = _clamp(output, self.output_limit)

        # ── Update state ─────────────────────────────────────────────────────
        self._last_error = error
        self._last_time  = now

        return output

    def reset(self):
        """Reset controller state. Call when switching behavior phases."""
        self._integral    = 0.0
        self._last_error  = 0.0
        self._last_time   = 0.0
        self._initialized = False


# ── Pre-configured controllers ────────────────────────────────────────────────
#
# These are the two controllers used by HambotBehavior.
# Adjust the gains here for tuning -- no other file needs to change.
#
# HeadingPID output  → steering_rpm  (how hard to turn)
# SpeedPID   output  → base_rpm      (how fast to drive straight)
#
#                        Kp     Ki    Kd    out_lim  int_lim  angular
HeadingPID = PIDController(1.2,  0.0,  0.05,  60.0,    20.0,    True)
SpeedPID   = PIDController(0.8,  0.0,  0.02,  50.0,    20.0,    False)


# ── Differential steering mixer ───────────────────────────────────────────────

def mix_to_wheel_speeds(
    base_rpm:     float,
    steering_rpm: float,
) -> tuple[float, float]:
    """
    Convert base + steering RPMs into individual wheel commands.

    The mixer ensures neither wheel exceeds RPM_MAX by scaling both
    down proportionally if needed, preserving the turning ratio.

    Args:
        base_rpm:     Forward speed component (positive = forward)
        steering_rpm: Turn component.
                      Positive → turn LEFT (add to left, subtract from right)
                      Negative → turn RIGHT

    Returns:
        (left_rpm, right_rpm): Wheel speeds ready for bot.set_*_motor_speed()
    """
    left  = base_rpm + steering_rpm
    right = base_rpm - steering_rpm

    # Scale down if either wheel exceeds the motor limit
    peak = max(abs(left), abs(right))
    if peak > RPM_MAX:
        scale = RPM_MAX / peak
        left  *= scale
        right *= scale

    return left, right


# ── Geometry helpers ──────────────────────────────────────────────────────────

def angle_to_target(
    robot_x: float, robot_y: float,
    target_x: float, target_y: float,
) -> float:
    """
    Absolute bearing from robot position to a target point, in degrees.
    Matches the world-frame convention: 0°=right, 90°=up, CCW positive.

    Args:
        robot_x, robot_y:   Robot position (cm)
        target_x, target_y: Target position (cm)

    Returns:
        float: Bearing in degrees, range -180..180
    """
    dx = target_x - robot_x
    dy = target_y - robot_y
    return math.degrees(math.atan2(dy, dx))


def heading_error(current_heading: float, target_heading: float) -> float:
    """
    Signed angular error from current heading to target heading.
    Wrapped to -180..180 so the robot always takes the short way around.

    Positive error → robot must turn LEFT  (CCW) to reach target
    Negative error → robot must turn RIGHT (CW)  to reach target

    Args:
        current_heading: Robot's current heading in degrees
        target_heading:  Desired heading in degrees

    Returns:
        float: Angular error in degrees, range -180..180
    """
    return _wrap_angle(target_heading - current_heading)


def distance_to(
    x1: float, y1: float,
    x2: float, y2: float,
) -> float:
    """Euclidean distance between two world-frame points (cm)."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# ── Internal helpers ──────────────────────────────────────────────────────────

def _wrap_angle(angle: float) -> float:
    """Wrap angle to the range -180..180 degrees."""
    while angle >  180.0: angle -= 360.0
    while angle < -180.0: angle += 360.0
    return angle


def _clamp(value: float, limit: float) -> float:
    """Clamp value to ±limit."""
    return max(-limit, min(limit, value))
