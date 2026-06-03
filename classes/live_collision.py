"""LiveCollisionMonitor – reads collision state from SharedState and publishes
it to the ROS status_command topic with a smart state machine:

  - Idle (no collision, burst exhausted): publishes nothing.
  - Colliding:                            publishes [FLAG, 1] every tick.
  - Just cleared (collision → no collision): publishes [FLAG, 0] exactly
    CLEAR_BURST times, then returns to Idle.
"""

from PySide6.QtCore import QObject, QTimer

from .shared_state import SharedState


class LiveCollisionMonitor(QObject):
    """Smart collision publisher.

    Publish behaviour
    -----------------
    * While colliding          → send [FLAG, 1] on every timer tick.
    * On first tick after clear → begin sending [FLAG, 0]; do so for
      exactly CLEAR_BURST ticks, then go silent.
    * While silent (no collision, burst done) → send nothing.

    Parameters
    ----------
    shared_state : SharedState
        Shared physics state; reads ``has_collision``.
    ros_node : NubiRosNode | None
        ROS node with ``publish_collision_status(bool)``.
        Pass ``None`` when ROS is unavailable – monitor becomes a no-op.
    frequency_hz : float
        Initial publish rate in Hz (default 10 Hz).
    """

    DEFAULT_HZ  = 10.0
    MIN_HZ      = 0.1
    MAX_HZ      = 50.0
    CLEAR_BURST = 5          # number of [FLAG, 0] messages sent after a collision clears

    def __init__(self, shared_state: SharedState, ros_node, frequency_hz: float = DEFAULT_HZ):
        super().__init__()
        self._shared   = shared_state
        self._ros_node = ros_node
        self._enabled  = ros_node is not None

        # State machine
        self._prev_colliding: bool = False   # collision state on the previous tick
        self._clear_remaining: int = 0       # how many [FLAG, 0] bursts still to send

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._publish_tick)
        self.set_frequency(frequency_hz)

    # ------------------------------------------------------------------
    def start(self) -> None:
        """Start (or restart) the publish timer."""
        if self._enabled:
            self._timer.start()

    def stop(self) -> None:
        self._timer.stop()

    def set_frequency(self, hz: float) -> None:
        """Set the publish rate.  Clamped to [MIN_HZ, MAX_HZ]."""
        hz = max(self.MIN_HZ, min(self.MAX_HZ, hz))
        self._timer.setInterval(int(1000.0 / hz))

    # ------------------------------------------------------------------
    def _publish_tick(self) -> None:
        """State-machine tick – called by the QTimer."""
        if not self._enabled or self._ros_node is None:
            return

        with self._shared.lock:
            colliding = self._shared.has_collision

        if colliding:
            # Active collision – publish 1 and reset the clear burst counter
            self._ros_node.publish_collision_status(True)
            self._clear_remaining = self.CLEAR_BURST

        elif self._clear_remaining > 0:
            # Collision just cleared – send [FLAG, 0] for CLEAR_BURST more ticks
            self._ros_node.publish_collision_status(False)
            self._clear_remaining -= 1

        # else: idle – publish nothing

        self._prev_colliding = colliding
