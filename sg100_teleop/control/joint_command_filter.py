import numpy as np


class JointCommandFilter:
    """
    Time-domain low-pass + velocity limiting.

    Designed for:
        low-rate tracking goal (~60 Hz)
            ↓
        high-rate MuJoCo control (500 Hz)

    This filter has no SG100-specific mapping logic.
    """

    def __init__(
        self,
        dof: int,
        tau: float = 0.06,
        max_velocity: float = 4.0,
    ):
        if tau <= 0:
            raise ValueError(
                "tau must be > 0"
            )

        if max_velocity <= 0:
            raise ValueError(
                "max_velocity must be > 0"
            )

        self.dof = int(dof)
        self.tau = float(tau)
        self.max_velocity = float(
            max_velocity
        )

        self._q = np.zeros(
            self.dof,
            dtype=np.float64,
        )

        self._initialized = False

    @property
    def value(self):
        return self._q.copy()

    def reset(
        self,
        q,
    ):
        q = np.asarray(
            q,
            dtype=np.float64,
        )

        if q.shape != (self.dof,):
            raise ValueError(
                f"Expected {(self.dof,)}, "
                f"got {q.shape}"
            )

        self._q[:] = q
        self._initialized = True

    def update(
        self,
        target,
        dt: float,
    ):
        target = np.asarray(
            target,
            dtype=np.float64,
        )

        if target.shape != (
            self.dof,
        ):
            raise ValueError(
                f"Expected {(self.dof,)}, "
                f"got {target.shape}"
            )

        if dt <= 0:
            return self.value

        if not self._initialized:
            self.reset(target)
            return self.value

        # Continuous-time first-order filter:
        #
        # alpha = 1 - exp(-dt / tau)
        #
        # Unlike a fixed alpha this behaves consistently
        # even if physics timestep changes.
        alpha = (
            1.0
            - np.exp(
                -dt / self.tau
            )
        )

        filtered_target = (
            self._q
            + alpha
            * (
                target
                - self._q
            )
        )

        max_delta = (
            self.max_velocity
            * dt
        )

        delta = np.clip(
            filtered_target
            - self._q,
            -max_delta,
            max_delta,
        )

        self._q += delta

        return self.value
