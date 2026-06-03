from robot import Nubi
import mujoco
import numpy as np

class NubiBehaviors(Nubi):
    def __init__(self, xml_path):
        super().__init__(xml_path)

        # Dictionary of actions
        methods = sorted([m for m in dir(self) if m.startswith('do_') and callable(getattr(self, m))])
        self.menu = {m.split('_')[1]: getattr(self, m) for m in methods}
        self.actions = " | ".join([f"{k}:{v.__name__.split('_', 2)[-1]}" for k, v in self.menu.items()])

    # --- Behaviors (Ordered by the middle number) ---

    def do_0_reset(self):
        """Reset simulation to initial drop state."""
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def do_1_stand(self):
        """Standard pose with captured foot anchors."""
        self.data.ctrl[:] = 0.0

    def do_2_wave(self):
        self.do_1_stand()  # Start from standing pose
        """Greeting Wave."""
        t = self.data.time
        osc = 30 * np.sin(2 * np.pi * 1.5 * t)
        self.move("R", "Shoulder", "roll", -20 + osc)
        self.move("R", "Shoulder", "pitch", 160)

    def do_3_handup(self):
        """Raise Hand."""
        self.do_1_stand()  # Start from standing pose
        self.move("R", "Shoulder", "pitch", 150)

    def do_4_salute(self):
        """Military Salute."""
        self.do_1_stand()  # Start from standing pose
        self.move("R", "Shoulder", "pitch", 120)
        self.move("R", "Elbow", "pitch", 90)

    def do_5_fight(self):
        self.do_1_stand()  # Start from standing pose
        """Dynamic fighting stance with auto-leveling feet."""
        t = self.data.time
        
        # 1. Configuration Constants
        squat_depth = 30.0  # Base squat angle
        stagger = 15.0      # How far one leg leads the other
        
        def apply_stable_leg(side, lead_offset):
            # Calculate base angles
            h_pitch = squat_depth + lead_offset
            k_pitch = h_pitch * 2.1 # Knee roughly double hip for vertical drop
            
            # THE MATH: To keep foot flat, ankle must counter hip and knee.
            # We use a trick: move hip/knee, then set ankle to negative sum.
            a_pitch = -(h_pitch + k_pitch)

            # Apply moves
            self.move(side, "Hip", "pitch", h_pitch)
            self.move(side, "Knee", "pitch", k_pitch)
            self.move(side, "Ankle", "pitch", a_pitch)
            
            # Width for stability
            roll_sign = 1 if side == "R" else -1
            self.move(side, "Hip", "roll", 8 * roll_sign)
            self.move(side, "Ankle", "roll", -8 * roll_sign)

        # 2. Execute Staggered Stance
        # Right leg forward (+ stagger), Left leg back (- stagger)
        apply_stable_leg("R", stagger)
        apply_stable_leg("L", -stagger)

        # 3. Dynamic Guard (Boxing)
        # Hands up, chin tucked, slight swaying
        sway = 10 * np.sin(2 * np.pi * 0.75 * t)
        self.move("R", "Shoulder", "pitch", 45 + sway)
        self.move("R", "Elbow", "pitch", 110)
        self.move("L", "Shoulder", "pitch", 35 - sway)
        self.move("L", "Elbow", "pitch", 90)