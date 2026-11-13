"""Time-based motion classification from measured three-mode IMM posteriors."""
import numpy as np

UNKNOWN, STATIC, DYNAMIC = 0, 1, 2


class MotionClassifier:
    """Hysteresis with continuous measured evidence; gaps never confirm motion."""

    def __init__(self, config):
        self.config = config
        self.state = UNKNOWN
        self.reset_evidence()

    def reset_evidence(self):
        self.last_stamp = None
        self.candidate = UNKNOWN
        self.since = None

    def update(self, stamp, probabilities, confirmed):
        cfg = self.config
        p = np.asarray(probabilities, dtype=float)
        if (not confirmed or p.shape != (3,) or not np.all(np.isfinite(p))
                or np.any(p < 0) or p.sum() <= 0):
            self.state = UNKNOWN
            self.reset_evidence()
            return self.state
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return self.state
        if (self.last_stamp is not None
                and stamp-self.last_stamp > cfg.motion_evidence_max_gap):
            self.reset_evidence()
            self.state = UNKNOWN
        self.last_stamp = stamp
        p = p / p.sum()
        static, dynamic = p[2], p[0] + p[1]
        if ((self.state == STATIC and static < cfg.static_exit_probability)
                or (self.state == DYNAMIC and dynamic < cfg.dynamic_exit_probability)):
            self.state = UNKNOWN
        candidate = (STATIC if static >= cfg.static_enter_probability else
                     DYNAMIC if dynamic >= cfg.dynamic_enter_probability else UNKNOWN)
        if candidate == UNKNOWN:
            self.candidate, self.since = UNKNOWN, None
        else:
            if candidate != self.candidate:
                self.candidate, self.since = candidate, stamp
            duration = (cfg.static_confirm_time if candidate == STATIC
                        else cfg.dynamic_confirm_time)
            if stamp-self.since >= duration-1e-9:
                self.state = candidate
        return self.state
