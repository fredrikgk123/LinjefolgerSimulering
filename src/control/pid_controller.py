# pid_controller.py

class PID:
    def __init__(self, kp, ki, kd, limit=5.0, integral_limit=2.0, derivative_filter=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit           = limit
        self.integral_limit  = integral_limit
        self.derivative_filter = derivative_filter
        self.integral   = 0.0
        self.prev_err   = 0.0
        self.d_filtered = 0.0

    def compute(self, err, dt):
        p = self.kp * err

        self.integral = max(min(self.integral + err * dt,
                                self.integral_limit), -self.integral_limit)
        i = self.ki * self.integral

        d_raw = (err - self.prev_err) / dt
        alpha = self.derivative_filter
        self.d_filtered = alpha * d_raw + (1 - alpha) * self.d_filtered
        d = self.kd * self.d_filtered

        self.prev_err = err
        return max(min(p + i + d, self.limit), -self.limit)

    def reset(self):
        self.integral   = 0.0
        self.prev_err   = 0.0
        self.d_filtered = 0.0


class SpeedController:
    """Adaptive speed: STRAIGHT at full speed, TURNING at reduced speed."""

    def __init__(self, straight_speed=0.80, turn_speed=0.50,
                 error_threshold=0.25, smoothing=0.08,
                 turn_factor_threshold=0.50, min_speed_factor=0.20):
        self.straight_speed        = straight_speed
        self.turn_speed            = turn_speed
        self.error_threshold       = error_threshold
        self.smoothing             = smoothing
        self.turn_factor_threshold = turn_factor_threshold
        self.min_speed_factor      = min_speed_factor
        self.state          = "STRAIGHT"
        self.speed_filtered = straight_speed

    def reset(self):
        self.state          = "STRAIGHT"
        self.speed_filtered = self.straight_speed

    def update(self, lateral_error, angular_velocity_cmd, pid_limit):
        abs_error  = abs(lateral_error)
        turn_factor = abs(angular_velocity_cmd) / pid_limit if pid_limit > 0 else 0

        if abs_error < self.error_threshold and turn_factor < self.turn_factor_threshold:
            self.state   = "STRAIGHT"
            target_speed = self.straight_speed
        else:
            self.state   = "TURNING"
            target_speed = self.turn_speed * max(self.min_speed_factor, 1.0 - turn_factor)

        # Low-pass blend to smooth speed transitions
        self.speed_filtered = (self.smoothing * target_speed +
                               (1 - self.smoothing) * self.speed_filtered)
        return self.speed_filtered

    def get_state(self):
        return self.state

