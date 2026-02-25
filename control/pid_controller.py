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
