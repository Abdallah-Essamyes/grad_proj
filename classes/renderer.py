"""MujocoRenderer – OpenGL rendering of the MuJoCo scene."""
import time
import numpy as np
import mujoco

from PySide6.QtCore import Qt, Signal, Slot
from PySide6.QtOpenGL import QOpenGLWindow

from .theme import Theme
from .shared_state import SharedState


class MujocoRenderer(QOpenGLWindow):
    """Renders the MuJoCo scene via QOpenGLWindow.

    Emits:
        hover_signal(str)  – actuator name under the cursor (or "")
        click_signal(str)  – actuator name that is now locked (or "")
    """

    hover_signal = Signal(str)
    click_signal = Signal(str)

    def __init__(self, model: mujoco.MjModel, shared_state: SharedState):
        super().__init__()
        self.m      = model
        self.shared = shared_state

        self.cam   = mujoco.MjvCamera()
        self.opt   = mujoco.MjvOption()
        self.scene = mujoco.MjvScene(self.m, max(1000, self.m.ngeom * 5))

        mujoco.mjv_defaultCamera(self.cam)
        mujoco.mjv_defaultOption(self.opt)

        self.opt.geomgroup[0] = 0
        self.cam.lookat    = [0, 0, 0.155]
        self.cam.distance  = 0.6
        self.cam.azimuth   = 115
        self.cam.elevation = -15

        self._last_pos  = None
        self._press_pos = None
        self._btn       = 0
        self._context   = None

        self.highlighted_actuator = ""
        self.locked_actuator      = ""

        self._last_pick_time = 0.0
        self._pick_interval  = 1.0 / 30.0

        # Build geometry ↔ actuator lookup tables
        self._geom_to_act     = {}
        self._act_to_geom_set = {}
        for act_id in range(self.m.nu):
            act_name = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, act_id)
            jnt_id   = self.m.actuator_trnid[act_id, 0]
            body_id  = self.m.jnt_bodyid[jnt_id]
            geoms    = [i for i in range(self.m.ngeom) if self.m.geom_bodyid[i] == body_id]
            self._act_to_geom_set[act_name] = set(geoms)
            for g in geoms:
                self._geom_to_act[g] = act_name

    # ------------------------------------------------------------------
    # OpenGL lifecycle
    # ------------------------------------------------------------------
    def initializeGL(self) -> None:
        self._context = mujoco.MjrContext(
            self.m, mujoco.mjtFontScale.mjFONTSCALE_150.value
        )

    def paintGL(self) -> None:
        if not self._context:
            return

        with self.shared.lock:
            mujoco.mjv_updateScene(
                self.m, self.shared.render_data, self.opt, None, self.cam,
                mujoco.mjtCatBit.mjCAT_ALL.value, self.scene,
            )

        self.scene.flags[mujoco.mjtRndFlag.mjRND_SKYBOX.value] = 1

        target = self.locked_actuator or self.highlighted_actuator
        if target and target in self._act_to_geom_set:
            color = Theme.to_rgba(Theme.CYAN, 1.0 if self.locked_actuator else 0.45)
            for i in range(self.scene.ngeom):
                if (self.scene.geoms[i].objtype == mujoco.mjtObj.mjOBJ_GEOM and
                        self.scene.geoms[i].objid in self._act_to_geom_set[target]):
                    self.scene.geoms[i].rgba[0:4] = color

        w = int(self.width()  * self.devicePixelRatio())
        h = int(self.height() * self.devicePixelRatio())
        mujoco.mjr_render(mujoco.MjrRect(0, 0, w, h), self.scene, self._context)

    # ------------------------------------------------------------------
    # Mouse interaction
    # ------------------------------------------------------------------
    def mousePressEvent(self, e) -> None:
        self._last_pos  = e.position()
        self._press_pos = e.position()
        self._btn = {
            Qt.LeftButton:   mujoco.mjtMouse.mjMOUSE_ROTATE_V,
            Qt.RightButton:  mujoco.mjtMouse.mjMOUSE_MOVE_V,
            Qt.MiddleButton: mujoco.mjtMouse.mjMOUSE_ZOOM,
        }.get(e.button(), 0)

    def mouseMoveEvent(self, e) -> None:
        if not e.buttons():
            self._btn = 0
            self._press_pos = None
            return
        if self._btn and self._last_pos:
            dx = e.position().x() - self._last_pos.x()
            dy = e.position().y() - self._last_pos.y()
            mujoco.mjv_moveCamera(
                self.m, self._btn,
                dx / max(1, self.width()),
                dy / max(1, self.height()),
                self.scene, self.cam,
            )
            self._last_pos = e.position()

    def mouseDoubleClickEvent(self, e) -> None:
        if e.button() == Qt.LeftButton:
            act = self._raycast(e.position())
            if act:
                self.locked_actuator = "" if self.locked_actuator == act else act
                self.click_signal.emit(self.locked_actuator)

    def mouseReleaseEvent(self, e) -> None:
        if self._press_pos and e.button() == Qt.LeftButton:
            if (e.position() - self._press_pos).manhattanLength() < 5.0:
                if not self._raycast(e.position()) and self.locked_actuator:
                    self.locked_actuator = ""
                    self.click_signal.emit("")
        self._btn = 0
        self._press_pos = None

    def wheelEvent(self, e) -> None:
        mujoco.mjv_moveCamera(
            self.m, mujoco.mjtMouse.mjMOUSE_ZOOM, 0,
            -e.angleDelta().y() / 1200, self.scene, self.cam,
        )

    # ------------------------------------------------------------------
    # Hover & raycast helpers
    # ------------------------------------------------------------------
    @Slot(str)
    def set_highlight(self, act_name: str) -> None:
        self.highlighted_actuator = act_name

    def process_hover(self, pos) -> None:
        now = time.time()
        if now - self._last_pick_time < self._pick_interval:
            return
        self._last_pick_time = now
        act = self._raycast(pos)
        if self.highlighted_actuator != act:
            self.highlighted_actuator = act
            self.hover_signal.emit(act)

    def _raycast(self, pos) -> str:
        aspect = self.width() / max(1, self.height())
        relx   = pos.x() / self.width()
        rely   = (self.height() - pos.y()) / self.height()

        selpnt = np.zeros(3, dtype=np.float64)
        geomid = np.zeros(1, dtype=np.int32)
        flexid = np.zeros(1, dtype=np.int32)
        skinid = np.zeros(1, dtype=np.int32)

        with self.shared.lock:
            mujoco.mjv_select(
                self.m, self.shared.render_data, self.opt,
                aspect, relx, rely, self.scene,
                selpnt, geomid, flexid, skinid,
            )

        g_id = geomid[0]
        return self._geom_to_act.get(g_id, "") if g_id != -1 else ""
