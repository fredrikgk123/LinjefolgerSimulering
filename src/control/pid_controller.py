# pid_controller.py

class PID:
    def __init__(self, kp, ki, kd, limit=5.0, integral_limit=2.0,
                 derivative_filter=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral_limit = integral_limit   # anti-windup clamp
        self.derivative_filter = derivative_filter  # LP filter coeff (0–1)
        self.integral = 0.0
        self.prev_err = 0.0
        self.d_filtered = 0.0

    def compute(self, err, dt):
        # Proportional
        p = self.kp * err

        # Integral with anti-windup
        self.integral += err * dt
        self.integral = max(min(self.integral, self.integral_limit),
                            -self.integral_limit)
        i = self.ki * self.integral

        # Derivative with low-pass filter to reduce noise sensitivity
        d_raw = (err - self.prev_err) / dt
        alpha = self.derivative_filter
        self.d_filtered = alpha * d_raw + (1 - alpha) * self.d_filtered
        d = self.kd * self.d_filtered

        self.prev_err = err

        u = p + i + d
        return max(min(u, self.limit), -self.limit)

    def reset(self):
        """Reset state — call when the robot re-acquires the line."""
        self.integral = 0.0
        self.prev_err = 0.0
        self.d_filtered = 0.0

class SpeedController:
    """State machine for adaptive speed control based on lateral error."""

    def __init__(self, straight_speed=0.80, turn_speed=0.50,
                 error_threshold=0.01, smoothing=0.1):
        """
        Args:
            straight_speed: Speed when error is near zero (high speed zone)
            turn_speed: Speed when turning (low speed zone)
            error_threshold: Error magnitude threshold to switch states (meters)
            smoothing: Low-pass filter coefficient for smooth transitions (0-1)
        """
        self.straight_speed = straight_speed
        self.turn_speed = turn_speed
        self.error_threshold = error_threshold
        self.smoothing = smoothing

        self.state = "STRAIGHT"  # STRAIGHT or TURNING
        self.current_speed = straight_speed
        self.speed_filtered = straight_speed

    def update(self, lateral_error, angular_velocity_cmd, pid_limit):
        """
        Update speed based on lateral error and steering command.

        Args:
            lateral_error: Current lateral error from centerline (meters)
            angular_velocity_cmd: PID output angular velocity command (rad/s)
            pid_limit: PID controller limit for normalization

        Returns:
            Recommended base speed (m/s)
        """
        abs_error = abs(lateral_error)
        abs_w = abs(angular_velocity_cmd)
        turn_factor = abs_w / pid_limit if pid_limit > 0 else 0

        # State machine logic
        if abs_error < self.error_threshold and turn_factor < 0.3:
            # Switch to STRAIGHT state - robot is well-aligned and not turning much
            self.state = "STRAIGHT"
            target_speed = self.straight_speed
        else:
            # Switch to TURNING state - robot needs to correct or is turning
            self.state = "TURNING"
            # Reduce speed more aggressively in turning state
            target_speed = self.turn_speed * max(0.4, 1.0 - turn_factor)

        # Smooth speed transitions with low-pass filter
        self.speed_filtered = (self.smoothing * target_speed +
                               (1 - self.smoothing) * self.speed_filtered)

        self.current_speed = self.speed_filtered
        return self.current_speed

    def get_state(self):
        """Return current state for debugging."""
        return self.state

