"""PhysicsEngine – MuJoCo simulation loop with collision detection."""
import time
import mujoco

from PySide6.QtCore import QObject, Signal

from .shared_state import SharedState


class PhysicsEngine(QObject):
    """Advances the MuJoCo simulation at wall-clock rate.

    Emits collision_status(bool) when the contype-2 self-collision state
    changes, and writes the current state to SharedState.has_collision so
    other components (LiveCollisionMonitor) can read it without subscribing
    to the signal.
    """

    collision_status = Signal(bool)

    def __init__(self, nubi, shared_state: SharedState):
        super().__init__()
        self.nubi   = nubi
        self.shared = shared_state

        self._acc            = 0.0
        self._last_time      = time.time()
        self._last_col_state = False

    # ------------------------------------------------------------------
    def step(self) -> None:
        t = time.time()
        self._acc += t - self._last_time
        self._acc = min(self._acc, 0.25)
        self._last_time = t

        dt = self.nubi.model.opt.timestep
        while self._acc >= dt:
            with self.shared.lock:
                current_mode = self.shared.mode
                overrides    = dict(self.shared.ctrl_overrides)

            if current_mode in ("custom", "ros"):
                for act_name, rad_val in overrides.items():
                    idx = self.nubi.actuators.get(act_name)
                    if idx is not None:
                        self.nubi.data.ctrl[idx] = rad_val
            else:
                func = self.nubi.menu.get(current_mode, self.nubi.load)
                func()
                if current_mode == "0":
                    with self.shared.lock:
                        self.shared.mode = ""

            mujoco.mj_step(self.nubi.model, self.nubi.data)

            has_col = self._detect_collision()

            # Write collision state into shared state (read by LiveCollisionMonitor)
            with self.shared.lock:
                self.shared.has_collision = has_col

            self.shared.push_to_render(self.nubi.model)

            if has_col != self._last_col_state:
                self._last_col_state = has_col
                self.collision_status.emit(has_col)

            self._acc -= dt

    # ------------------------------------------------------------------
    def _detect_collision(self) -> bool:
        """Return True if any contact pair has contype == 2 on both geoms."""
        for i in range(self.nubi.data.ncon):
            g1 = self.nubi.data.contact[i].geom1
            g2 = self.nubi.data.contact[i].geom2
            if (self.nubi.model.geom_contype[g1] == 2 and
                    self.nubi.model.geom_contype[g2] == 2):
                return True
        return False
