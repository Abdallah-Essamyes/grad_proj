from dataclasses import dataclass, field

@dataclass
class Servo_Pair:
    servos_full_range: int

    servo_left_id: int
    servo_right_id: int

    servo_left_start_angle: int
    servo_right_start_angle: int

    min_angle: int = 20
    max_angle: int = 160

    servo_left_angle: int = field(init=False)
    servo_right_angle: int = field(init=False)

    def __post_init__(self):
        # initialize to start positions
        self.servo_left_angle = self.servo_left_start_angle
        self.servo_right_angle = self.servo_right_start_angle

    def _compute_from_left(self, left_angle: int):
        right = self.servo_left_start_angle + self.servo_right_start_angle - left_angle
        return left_angle, right

    def _compute_from_right(self, right_angle: int):
        left = self.servo_left_start_angle + self.servo_right_start_angle - right_angle
        return left, right_angle

    def set_angle(self, servo_id: int, angle: int):
        """
        Set one servo, automatically compute the other.
        No shared hidden state.
        """
        if servo_id == self.servo_left_id:
            left, right = self._compute_from_left(angle)
            
        elif servo_id == self.servo_right_id:
            left, right = self._compute_from_right(angle)
            
        else:
            return False

        # safety check (atomic update)
        if not (self.min_angle <= left <= self.max_angle and
                self.min_angle <= right <= self.max_angle):
            return False

        self.servo_left_angle = left
        self.servo_right_angle = right

        return True
    
    def get_ids(self) -> tuple[int, int]:
        """Returns the (left_id, right_id) for quick lookup."""
        return self.servo_left_id, self.servo_right_id