from controller import Robot, Camera
import math

# =============================================================================
# CONFIGURATION
# =============================================================================
OWN_GOAL = 'magenta'          # Goal we DEFEND (right side, x ~ +1.1)
OPP_GOAL = 'cyan'       # Goal we ATTACK (left side, x ~ -1.1)

TIME_STEP = 64
MAX_SPEED = 10
WHEEL_RADIUS = 0.04
WHEELBASE = 0.12
DT = TIME_STEP / 1000.0

# =============================================================================
# HARDWARE INIT
# =============================================================================
robot = Robot()

camera = robot.getDevice('camera')
camera.enable(TIME_STEP)
cam_w = camera.getWidth()
cam_h = camera.getHeight()
cam_fov = camera.getFov()

lidar = robot.getDevice('LDS-01')
lidar.enable(TIME_STEP)

wheels = []
for name in ['front_left_wheel', 'front_right_wheel',
             'back_left_wheel', 'back_right_wheel']:
    w = robot.getDevice(name)
    w.setPosition(float('inf'))
    w.setVelocity(0.0)
    wheels.append(w)

for name in ['ds_front_left', 'ds_front_right', 'ds_back', 'ds_right', 'ds_left']:
    robot.getDevice(name).enable(TIME_STEP)

# =============================================================================
# SHARED STATE — updated every frame by step_and_scan()
# =============================================================================
ball = None         # {'cx': float, 'cy': float, 'size': int} or None
own_goal = None     # {'cx': float, 'size': int} or None
opp_goal = None     # {'cx': float, 'size': int} or None

# LIDAR wall distances (meters, inf if not detected)
wall_front = float('inf')
wall_back = float('inf')
wall_left = float('inf')
wall_right = float('inf')

# Dead reckoning
est_ball_angle = 0.0    # radians: 0=ahead, +left, -right
ball_confidence = 0.0   # 1.0=just saw, decays 4%/frame
cmd_left = 0.0
cmd_right = 0.0

# Return status for blocking functions
SUCCESS = 'success'
TIMEOUT = 'timeout'
LOST_BALL = 'lost_ball'

# =============================================================================
# SENSOR LAYER
# =============================================================================
def is_yellow(r, g, b):
    if r > 140 and g > 140 and b < 100:
        return True
    if r > 160 and g > 160 and b < 130 and r > b + 50 and g > b + 50:
        return True
    if r > 100 and g > 100 and b < 60 and r > b + 60 and g > b + 60:
        return True
    return False

def is_cyan(r, g, b):
    return r < 100 and g > 120 and b > 120

def is_magenta(r, g, b):
    return r > 120 and g < 100 and b > 120

def _scan_camera(image):
    """Scan camera image for ball and goal colors."""
    global ball, own_goal, opp_goal
    bx, by, bn = 0, 0, 0
    cx, cn = 0, 0
    mx, mn = 0, 0
    STEP = 2
    for y in range(0, cam_h, STEP):
        for x in range(0, cam_w, STEP):
            r = Camera.imageGetRed(image, cam_w, x, y)
            g = Camera.imageGetGreen(image, cam_w, x, y)
            b = Camera.imageGetBlue(image, cam_w, x, y)
            if is_yellow(r, g, b):
                bx += x; by += y; bn += 1
            elif is_cyan(r, g, b):
                cx += x; cn += 1
            elif is_magenta(r, g, b):
                mx += x; mn += 1

    ball = {'cx': bx / bn, 'cy': by / bn, 'size': bn} if bn > 10 else None
    cyan_g = {'cx': cx / cn, 'size': cn} if cn > 6 else None
    magenta_g = {'cx': mx / mn, 'size': mn} if mn > 6 else None

    if OWN_GOAL == 'cyan':
        own_goal, opp_goal = cyan_g, magenta_g
    else:
        own_goal, opp_goal = magenta_g, cyan_g

def _scan_lidar():
    """Read LIDAR and extract wall distances in 4 cardinal directions.

    The RobotisLds01 has 360 rays over 360 degrees.
    It is mounted rotated 180 degrees, so ray 0 points backward.
    Ray layout (from robot's perspective):
      rays 0-44 / 315-359 : BACK
      rays 45-134          : RIGHT  (LIDAR left = robot right due to 180 flip)
      rays 135-224         : FRONT
      rays 225-314         : LEFT   (LIDAR right = robot left due to 180 flip)
    """
    global wall_front, wall_back, wall_left, wall_right
    dists = lidar.getRangeImage()
    n = len(dists)
    if n == 0:
        return

    def sector_min(start, end):
        vals = [dists[i] for i in range(start, end) if not math.isinf(dists[i])]
        return min(vals) if vals else float('inf')

    wall_back = min(sector_min(0, 45), sector_min(315, 360))
    wall_right = sector_min(45, 135)
    wall_front = sector_min(135, 225)
    wall_left = sector_min(225, 315)

def _update_dead_reckoning():
    """Update ball angle estimate from last frame's wheel commands."""
    global est_ball_angle, ball_confidence
    omega = (cmd_right - cmd_left) * WHEEL_RADIUS / WHEELBASE
    est_ball_angle -= omega * DT
    est_ball_angle = (est_ball_angle + math.pi) % (2 * math.pi) - math.pi
    ball_confidence *= 0.96

def _correct_from_camera():
    """Snap dead reckoning to camera observation."""
    global est_ball_angle, ball_confidence
    if ball:
        pixel_offset = (ball['cx'] - cam_w / 2) / (cam_w / 2)
        est_ball_angle = -pixel_offset * (cam_fov / 2)
        ball_confidence = 1.0

def drive(l, r):
    """Set wheel speeds (clamped) and track for dead reckoning."""
    global cmd_left, cmd_right
    l = max(-MAX_SPEED, min(MAX_SPEED, l))
    r = max(-MAX_SPEED, min(MAX_SPEED, r))
    wheels[0].setVelocity(l)
    wheels[2].setVelocity(l)
    wheels[1].setVelocity(r)
    wheels[3].setVelocity(r)
    cmd_left = l
    cmd_right = r

def step_and_scan():
    """Advance one simulation frame and update all sensor state.
    Returns False if simulation ended."""
    if robot.step(TIME_STEP) == -1:
        return False
    _update_dead_reckoning()
    image = camera.getImage()
    _scan_camera(image)
    _scan_lidar()
    _correct_from_camera()
    return True

# =============================================================================
# HELPER: camera-relative error for steering
# =============================================================================
def ball_error():
    """Return ball's horizontal offset: -1 (left) to +1 (right). None if no ball."""
    if ball is None:
        return None
    return (ball['cx'] - cam_w / 2) / (cam_w / 2)

def steer_toward(error, base_speed=None, Kp=0.85):
    """Convert a horizontal error [-1,+1] into (left, right) wheel speeds.
    Positive error = target is right = turn right."""
    if base_speed is None:
        base_speed = MAX_SPEED
    l = base_speed * (1 + Kp * error)
    r = base_speed * (1 - Kp * error)
    return l, r

# =============================================================================
# LAYER 2 — PRIMITIVE ACTIONS (blocking, with timeout)
# =============================================================================

def FaceBall(max_frames=80):
    """Spin toward the ball using dead reckoning, stop when ball is
    centered in camera (within ~10% of center). Returns SUCCESS or TIMEOUT."""
    for _ in range(max_frames):
        if not step_and_scan():
            return TIMEOUT

        if ball:
            err = ball_error()
            if abs(err) < 0.1:
                drive(0, 0)
                return SUCCESS
            l, r = steer_toward(err, base_speed=MAX_SPEED * 0.5, Kp=1.2)
            drive(l, r)
        elif ball_confidence > 0.15:
            # Use dead reckoning to spin toward estimated ball position
            angle = est_ball_angle
            spd = MAX_SPEED * 0.6
            if angle > 0:
                drive(-spd, spd)     # turn left
            else:
                drive(spd, -spd)     # turn right
        else:
            # No idea where ball is, spin to search
            drive(MAX_SPEED * 0.6, -MAX_SPEED * 0.6)

    drive(0, 0)
    return TIMEOUT


def FaceOwnGoal(max_frames=80):
    """Spin until own goal color is detected and roughly centered."""
    for _ in range(max_frames):
        if not step_and_scan():
            return TIMEOUT
        if own_goal and own_goal['size'] > 15:
            err = (own_goal['cx'] - cam_w / 2) / (cam_w / 2)
            if abs(err) < 0.2:
                drive(0, 0)
                return SUCCESS
            l, r = steer_toward(err, base_speed=MAX_SPEED * 0.4, Kp=1.0)
            drive(l, r)
        else:
            drive(MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
    drive(0, 0)
    return TIMEOUT


def FaceOpponentsGoal(max_frames=80):
    """Spin until opponent goal color is detected and roughly centered."""
    for _ in range(max_frames):
        if not step_and_scan():
            return TIMEOUT
        if opp_goal and opp_goal['size'] > 15:
            err = (opp_goal['cx'] - cam_w / 2) / (cam_w / 2)
            if abs(err) < 0.2:
                drive(0, 0)
                return SUCCESS
            l, r = steer_toward(err, base_speed=MAX_SPEED * 0.4, Kp=1.0)
            drive(l, r)
        else:
            drive(-MAX_SPEED * 0.5, MAX_SPEED * 0.5)
    drive(0, 0)
    return TIMEOUT


def PushBallForward(max_frames=150):
    """Drive toward ball at full speed. Stops when ball is lost or timeout.
    Returns SUCCESS if ball was pushed (lost from view while driving forward),
    LOST_BALL if can't find it, TIMEOUT otherwise."""
    frames_without_ball = 0
    for _ in range(max_frames):
        if not step_and_scan():
            return TIMEOUT

        if ball:
            frames_without_ball = 0
            err = ball_error()
            l, r = steer_toward(err, base_speed=MAX_SPEED, Kp=0.85)
            drive(l, r)
        else:
            frames_without_ball += 1
            if frames_without_ball <= 6:
                # Grace period: keep driving forward
                drive(MAX_SPEED * 0.7, MAX_SPEED * 0.7)
            else:
                drive(0, 0)
                return LOST_BALL

    drive(0, 0)
    return TIMEOUT


def EmergencyDefend(max_frames=60):
    """Full speed charge at ball. Used when ball is near own goal.
    Returns SUCCESS when ball is cleared, LOST_BALL if lost without contact, TIMEOUT."""
    frames_driven = 0
    for _ in range(max_frames):
        if not step_and_scan():
            return TIMEOUT

        if ball:
            frames_driven += 1
            err = ball_error()
            l, r = steer_toward(err, base_speed=MAX_SPEED, Kp=0.9)
            drive(l, r)
        else:
            drive(0, 0)
            return SUCCESS if frames_driven > 5 else LOST_BALL

    drive(0, 0)
    return TIMEOUT


def IsMoving():
    """Check if robot is actually moving by comparing LIDAR readings.
    Takes a few frames to determine. Returns True/False."""
    # Read LIDAR twice with a gap
    if not step_and_scan():
        return False
    front1 = wall_front
    for _ in range(3):
        if not step_and_scan():
            return False
    front2 = wall_front
    return abs(front2 - front1) > 0.005


# =============================================================================
# LAYER 3 — COMPOUND ACTIONS
# =============================================================================

def RecenterCar(max_frames=120):
    """Use LIDAR wall distances to drive toward the center of the field.
    Centers between left/right walls, then between front/back walls."""
    for _ in range(max_frames):
        if not step_and_scan():
            return TIMEOUT

        lr_diff = wall_left - wall_right   # positive = closer to right wall
        fb_diff = wall_front - wall_back   # positive = closer to back wall

        # If roughly centered (within 10cm on each axis), done
        if abs(lr_diff) < 0.15 and abs(fb_diff) < 0.15:
            drive(0, 0)
            return SUCCESS

        # Priority: fix left-right first (more important for goal alignment)
        if abs(lr_diff) > 0.15:
            # Need to move toward the farther wall
            if lr_diff > 0:
                # Closer to right wall, need to go left
                drive(MAX_SPEED * 0.3, MAX_SPEED * 0.6)
            else:
                # Closer to left wall, need to go right
                drive(MAX_SPEED * 0.6, MAX_SPEED * 0.3)
        elif abs(fb_diff) > 0.15:
            if fb_diff > 0:
                # Closer to back wall, drive forward
                drive(MAX_SPEED * 0.5, MAX_SPEED * 0.5)
            else:
                # Closer to front wall, back up
                drive(-MAX_SPEED * 0.4, -MAX_SPEED * 0.4)

    drive(0, 0)
    return TIMEOUT


def DriveAroundBall(direction='auto', max_frames=80):
    """Drive toward ball, then arc around it.
    direction: 'left', 'right', or 'auto' (picks side based on goal position).
    Returns SUCCESS when ball is roughly behind robot, TIMEOUT otherwise."""

    # Decide which way to circle
    if direction == 'auto':
        # If we can see the ball, pick based on which side has more room
        direction = 'right' if wall_left < wall_right else 'left'

    for i in range(max_frames):
        if not step_and_scan():
            return TIMEOUT

        if ball is None:
            if ball_confidence > 0.3:
                # Keep arcing, we probably just passed the ball
                if direction == 'left':
                    drive(MAX_SPEED * 0.2, MAX_SPEED * 0.7)
                else:
                    drive(MAX_SPEED * 0.7, MAX_SPEED * 0.2)
                continue
            else:
                drive(0, 0)
                return LOST_BALL

        err = ball_error()
        # If ball is now behind us (confidence says ball is > 90 deg away), done
        if ball_confidence > 0.5 and abs(est_ball_angle) > math.pi * 0.6:
            drive(0, 0)
            return SUCCESS

        # Phase 1: approach ball until close
        if ball['size'] < 60:
            l, r = steer_toward(err, base_speed=MAX_SPEED * 0.7, Kp=0.7)
            drive(l, r)
        else:
            # Phase 2: arc around ball
            if direction == 'left':
                drive(MAX_SPEED * 0.15, MAX_SPEED * 0.8)
            else:
                drive(MAX_SPEED * 0.8, MAX_SPEED * 0.15)

    drive(0, 0)
    return TIMEOUT


def AlignBallGoal(max_frames=200):
    """Position robot so ball is between robot and opponent's goal.
    This is the key scoring setup function.
    Returns SUCCESS when aligned, TIMEOUT otherwise."""

    for _ in range(max_frames):
        if not step_and_scan():
            return TIMEOUT

        # Best case: ball AND opponent goal visible AND ball is roughly centered
        # This means we're behind the ball facing the opponent goal
        if ball and opp_goal and abs(ball_error()) < 0.35:
            drive(0, 0)
            return SUCCESS

        # If we see ball AND own goal → we're on the wrong side
        if ball and own_goal:
            drive(0, 0)
            result = DriveAroundBall(max_frames=40)
            if result == LOST_BALL:
                FaceBall(max_frames=30)
            continue

        # If we see ball but no goal → approach and reassess
        if ball:
            err = ball_error()
            l, r = steer_toward(err, base_speed=MAX_SPEED * 0.5, Kp=0.7)
            drive(l, r)
            continue

        # No ball → find it first
        FaceBall(max_frames=30)

    drive(0, 0)
    return TIMEOUT


def SidleLeft(frames_per_phase=15):
    """Shift left while roughly maintaining heading.
    Phase 1: reverse arc curving left. Phase 2: forward arc correcting right."""
    for _ in range(frames_per_phase):
        if not step_and_scan():
            return TIMEOUT
        drive(-MAX_SPEED * 0.2, -MAX_SPEED * 0.5)

    for _ in range(frames_per_phase):
        if not step_and_scan():
            return TIMEOUT
        drive(MAX_SPEED * 0.5, MAX_SPEED * 0.2)

    drive(0, 0)
    return SUCCESS


def SidleRight(frames_per_phase=15):
    """Shift right while roughly maintaining heading.
    Phase 1: reverse arc curving right. Phase 2: forward arc correcting left."""
    for _ in range(frames_per_phase):
        if not step_and_scan():
            return TIMEOUT
        drive(-MAX_SPEED * 0.5, -MAX_SPEED * 0.2)

    for _ in range(frames_per_phase):
        if not step_and_scan():
            return TIMEOUT
        drive(MAX_SPEED * 0.2, MAX_SPEED * 0.5)

    drive(0, 0)
    return SUCCESS


def PushBallLeft(max_frames=80):
    """Approach ball from the right side, pushing it left."""
    SidleRight()
    return PushBallForward(max_frames=max_frames)


def PushBallRight(max_frames=80):
    """Approach ball from the left side, pushing it right."""
    SidleLeft()
    return PushBallForward(max_frames=max_frames)


def Retreat(max_frames=25):
    """Back up straight. Used after scoring or when stuck near a wall."""
    for _ in range(max_frames):
        if not step_and_scan():
            return TIMEOUT
        drive(-MAX_SPEED * 0.7, -MAX_SPEED * 0.7)
    drive(0, 0)
    return SUCCESS


# =============================================================================
# LAYER 4 — GAME STRATEGY
# =============================================================================

def play():
    """Main game loop. Runs forever."""

    # Initial scan
    step_and_scan()

    while True:
        # Always refresh sensors at top of loop
        if not step_and_scan():
            return

        # ----- EMERGENCY DEFEND -----
        # If ball is visible AND own goal is big (ball near our goal), clear it
        if ball and own_goal and own_goal['size'] > 200:
            EmergencyDefend()
            continue

        # ----- ATTACK: ball visible -----
        if ball:
            # Can we see opponent goal too? → Push!
            if opp_goal:
                PushBallForward()
                # Refresh sensors after push completes
                step_and_scan()
                if not ball and opp_goal and opp_goal['size'] > 30:
                    Retreat()
                continue

            # Can see ball + own goal → wrong side, realign
            if own_goal:
                AlignBallGoal()
                continue

            # Can see ball, no goal → approach and push
            # (will transition to PUSH or ALIGN once goal becomes visible)
            PushBallForward(max_frames=60)
            continue

        # ----- NO BALL VISIBLE -----

        # Near opponent goal? Back up first
        if opp_goal and opp_goal['size'] > 30:
            Retreat()
            continue

        # Use dead reckoning if confident
        if ball_confidence > 0.2:
            FaceBall(max_frames=30)
            continue

        # Last resort: recenter and search
        RecenterCar(max_frames=40)
        FaceBall(max_frames=60)


# =============================================================================
# ENTRY POINT
# =============================================================================
play()
