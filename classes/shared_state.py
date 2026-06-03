"""SharedState – double-buffered thread-safe bridge between physics and UI."""
import threading
import mujoco


class SharedState:
    """Thread-safe buffer shared between the PhysicsEngine (writer) and
    all readers (renderer, UI, LiveCollisionMonitor)."""

    def __init__(self, model: mujoco.MjModel, physics_data: mujoco.MjData):
        self.lock = threading.Lock()
        self.physics_data = physics_data
        self.render_data  = mujoco.MjData(model)

        mujoco.mj_copyData(self.render_data, model, self.physics_data)

        # Current control mode: "", "0", "custom", "ros", or behavior key
        self.mode: str = "0"

        # Active joint overrides: actuator_name → radians
        self.ctrl_overrides: dict[str, float] = {}

        # Latest collision state written by PhysicsEngine
        self.has_collision: bool = False

    def push_to_render(self, model: mujoco.MjModel) -> None:
        """Copy physics_data → render_data while holding the lock."""
        with self.lock:
            mujoco.mj_copyData(self.render_data, model, self.physics_data)
