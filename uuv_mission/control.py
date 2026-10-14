class PDController:
    """Discrete-time PD controller: u[t] = KP*e[t] + KD*(e[t] - e[t-1])."""

    def __init__(self, kp: float = 0.15, kd: float = 0.6):
        self.kp = kp
        self.kd = kd
        self.reset()

    def reset(self) -> None:
        """Clear stored error so e[-1] = 0 at the start of a simulation."""
        self.prev_error = 0.0

    def compute_action(self, reference: float, observation: float) -> float:
        """Return u[t] given the reference r[t] and measured depth y[t]."""
        error = reference - observation
        action = self.kp * error + self.kd * (error - self.prev_error)
        self.prev_error = error
        return action
    