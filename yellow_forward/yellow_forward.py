from controller import Robot, Camera, Lidar
import math

TIME_STEP = 64
MAX_SPEED = 10
TURN_STEPS = 20   # tune this to control how many degrees it turns
STRAIGHT_STEPS = 30  # tune this to control how far it drives past the ball

robot = Robot()

# ------------------ CAMERA ------------------
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)
cam_width = camera.getWidth()
cam_height = camera.getHeight()

# ------------------ LIDAR ------------------
lidar = robot.getDevice('LDS-01')
lidar.enable(TIME_STEP)

lidar_resolution = lidar.getHorizontalResolution()
lidar_fov = lidar.getFov()
# ------------------ WHEELS ------------------
wheels = []
wheel_names = [
    'front_left_wheel',
    'front_right_wheel',
    'back_left_wheel',
    'back_right_wheel'
]
for name in wheel_names:
    w = robot.getDevice(name)
    w.setPosition(float('inf'))
    w.setVelocity(0.0)
    wheels.append(w)

def set_speed(left, right):
    wheels[0].setVelocity(left)   # front left
    wheels[2].setVelocity(left)   # back left
    wheels[1].setVelocity(right)  # front right
    wheels[3].setVelocity(right)  # back right
    
# ------------------ DISTANCE SENSORS ------------------
ds_names = ['ds_front_left', 'ds_front_right', 'ds_back', 'ds_right', 'ds_left']
distance_sensors = {}
for name in ds_names:
    ds = robot.getDevice(name)
    ds.enable(TIME_STEP)
    distance_sensors[name] = ds

def get_ds(name):
    """
    Returns the distance reading (in meters) for a named sensor.
    
    Available sensors:
        'DS_FRONT_LEFT', 'DS_FRONT_RIGHT', 'DS_BACK', 'DS_RIGHT', 'DS_LEFT'
    
    Example usage:
        if get_ds('DS_FRONT_LEFT') < 0.2:
            print("Close to something on the front-left!")
    """
    return distance_sensors[name].getValue()
    
def clamp(x, lo, hi): #utility function that constrains a value between a minimum and maximum bound [use if wanted]
    return max(lo, min(hi, x))

def is_yellow(r, g, b):
    # Core yellow: high R, high G, low B
    if r > 140 and g > 140 and b < 100:
        return True
    # Bright/washed-out yellow at distance (lighting reduces saturation)
    if r > 160 and g > 160 and b < 130 and r > b + 50 and g > b + 50:
        return True
    # Darker yellow in shadows
    if r > 100 and g > 100 and b < 60 and r > b + 60 and g > b + 60:
        return True
    return False
    
def is_magenta(r, g, b):
    return r > 200 and g < 80 and b > 200

def is_cyan(r, g, b):
    return r < 80 and g > 200 and b > 200

def get_lidar_distances():
    """
    Returns a flat list of distance readings (in meters) from the lidar.

    - Each value is the distance to the nearest obstacle in that direction.
    - Values equal to 'inf' mean nothing was detected in that direction.
    - The list goes left-to-right across the lidar's field of view.

    Example usage:
        distances = get_lidar_distances()
        front_distance = distances[len(distances) // 2]  # center ray
    """
    return list(lidar.getRangeImage())

def get_lidar_sector(distances, sector='front'):
    """
    Returns the minimum distance detected in a named sector of the lidar.

    Sectors divide the lidar's view into five equal zones:
        'left', 'front-left', 'front', 'front-right', 'right'

    Parameters:
        distances (list): output from get_lidar_distances()
        sector (str): one of the five sector names above

    Returns:
        float: closest distance (meters) in that sector, or inf if nothing detected

    Example usage:
        distances = get_lidar_distances()
        if get_lidar_sector(distances, 'front') < 0.3:
            print("Obstacle ahead!")
    """
    n = len(distances)
    sectors = {
        'left':        distances[0           : n // 5],
        'front-left':  distances[n // 5      : 2 * n // 5],
        'front':       distances[2 * n // 5  : 3 * n // 5],
        'front-right': distances[3 * n // 5  : 4 * n // 5],
        'right':       distances[4 * n // 5  :],
    }
    readings = sectors.get(sector, [])
    return min((d for d in readings if not math.isinf(d)), default=float('inf'))

def PushBallForward(info):
    if info.yellow_count == 0:
        # ---- SEARCH FOR BALL ----
        info.leftSpeed  = 0.5 * MAX_SPEED
        info.rightSpeed = -0.5 * MAX_SPEED

    if info.yellow_count > 1:
        # ---- CHARGE OPPONENT ----
        avg_x = info.yellow_x_sum / info.yellow_count #average x coord of yellow pixels
        if avg_x < cam_width * 0.4:
            info.leftSpeed  = 0.5 * MAX_SPEED
            info.rightSpeed = MAX_SPEED
        elif avg_x > cam_width * 0.6:
            info.leftSpeed  = MAX_SPEED
            info.rightSpeed = 0.5 * MAX_SPEED
        else:
            info.leftSpeed  = MAX_SPEED
            info.rightSpeed = MAX_SPEED

class SoccerRobot:
    def __init__(self):
        self.leftSpeed = 0.0
        self.rightSpeed = 0.0
        self.distances = []
        self.ballXPos = 0.0
        self.canSeeBall = False
        self.bypass_done = False
        self.circle_direction = None
        self.v_step = 0
    
    def getObjectPosition(self, color_fn):
        image = camera.getImage()
        x_sum = 0
        count = 0

        for y in range(cam_height):
            for x in range(cam_width):
                r = Camera.imageGetRed(image, cam_width, x, y)
                g = Camera.imageGetGreen(image, cam_width, x, y)
                b = Camera.imageGetBlue(image, cam_width, x, y)

                if color_fn(r, g, b):
                    x_sum += x
                    count += 1

        if count > 0:
            return x_sum / count, True
        else:
            return None, False

    def faceObject(self, color_fn):
        obj_x, seen = self.getObjectPosition(color_fn)

        if seen and obj_x is not None:
            if obj_x < cam_width * 0.4:
                self.leftSpeed  = -0.5 * MAX_SPEED
                self.rightSpeed = 0.5 * MAX_SPEED

            elif obj_x > cam_width * 0.6:
                self.leftSpeed  = 0.5 * MAX_SPEED
                self.rightSpeed = -0.5 * MAX_SPEED

            else:
                # centered
                self.leftSpeed  = 0.0
                self.rightSpeed = 0.0
        else:
            # search
            self.leftSpeed  = 0.5 * MAX_SPEED
            self.rightSpeed = -0.5 * MAX_SPEED

    def getBallPosition(self):
        image = camera.getImage()
        yellow_x_sum = 0
        yellow_count = 0
        for y in range(cam_height):
            for x in range(cam_width):
                r = Camera.imageGetRed(image, cam_width, x, y)
                g = Camera.imageGetGreen(image, cam_width, x, y)
                b = Camera.imageGetBlue(image, cam_width, x, y)
                if is_yellow(r, g, b):
                    yellow_x_sum += x
                    yellow_count += 1
                
        if yellow_count > 0:
            # if the ball is on screen, update the ballXPos
            print("ball detected, setting ballXPos")
            self.canSeeBall = True
            self.ballXPos = yellow_x_sum / yellow_count
        else: 
            # if the ball is not on screen, don't touch the ballXPos
            # this keeps the last x position of the ball so the robot knows which way to spin
            self.canSeeBall = False

    def faceBall(self):
        self.getBallPosition()
    
        if self.ballXPos < cam_width * 0.4:
            self.leftSpeed  = -0.5 * MAX_SPEED
            self.rightSpeed = 0.5 * MAX_SPEED
    
        elif self.ballXPos > cam_width * 0.6:
            self.leftSpeed  = 0.5 * MAX_SPEED
            self.rightSpeed = -0.5 * MAX_SPEED
    
        elif self.canSeeBall:
            # centered
            self.leftSpeed  = 0.0
            self.rightSpeed = 0.0
    
        else:
            # searching
            self.leftSpeed  = 0.5 * MAX_SPEED
            self.rightSpeed = -0.5 * MAX_SPEED

    def faceOpponentGoal(self):
        print("Facing opponent goal (cyan)")
        self.faceObject(is_cyan)

    def faceOwnGoal(self):
        print("Facing own goal (magenta)")
        self.faceObject(is_magenta)

    def getDistances(self):
        self.distances = get_lidar_distances()

    def setSpeed(self):
        set_speed(self.leftSpeed, self.rightSpeed)
    
    def driveForwardTimed(self, steps=60):
        self.v_step += 1
    
        self.leftSpeed  = 0.7 * MAX_SPEED
        self.rightSpeed = 0.7 * MAX_SPEED
    
        if self.v_step >= steps:
            self.v_step = 0
            return True
    
        return False
        
    def repositionAroundBall(self):
        """
        Simple bypass:
        1. Slight turn away from ball
        2. Drive forward past it
        3. Done
        """
    
        self.v_step += 1
    
        # decide direction ONCE
        if self.v_step == 1:
            if self.ballXPos < cam_width / 2:
                self.circle_direction = "left"
            else:
                self.circle_direction = "right"
    
        TURN_TIME = 10
        FORWARD_TIME = 25
    
        # -------- Phase 1: slight turn --------
        if self.v_step <= TURN_TIME:
            if self.circle_direction == "left":
                self.leftSpeed  = -0.3 * MAX_SPEED
                self.rightSpeed =  0.6 * MAX_SPEED
            else:
                self.leftSpeed  =  0.6 * MAX_SPEED
                self.rightSpeed = -0.3 * MAX_SPEED
    
        # -------- Phase 2: drive forward --------
        elif self.v_step <= TURN_TIME + FORWARD_TIME:
            self.leftSpeed  = 0.7 * MAX_SPEED
            self.rightSpeed = 0.7 * MAX_SPEED
    
        # -------- Done --------
        else:
            self.v_step = 0
            self.circle_direction = None
            return True
    
        return False


# ------------------ MAIN LOOP ------------------
soccerRobot = SoccerRobot()

state = "SEARCH"
soccerRobot.v_step = 0

while robot.step(TIME_STEP) != -1:

    # always update vision (safe)
    soccerRobot.getBallPosition()

    # ---------------- SEARCH ----------------
    if state == "SEARCH":
        if soccerRobot.canSeeBall:
            print("STATE → APPROACH")
            state = "APPROACH"
        else:
            soccerRobot.leftSpeed  = 0.5 * MAX_SPEED
            soccerRobot.rightSpeed = -0.5 * MAX_SPEED

    # ---------------- APPROACH ----------------
    elif state == "APPROACH":
        if not soccerRobot.canSeeBall:
            print("Lost ball → SEARCH")
            state = "SEARCH"

        else:
            ball_x = soccerRobot.ballXPos

            if ball_x < cam_width * 0.45:
                soccerRobot.leftSpeed  = 0.3 * MAX_SPEED
                soccerRobot.rightSpeed = MAX_SPEED

            elif ball_x > cam_width * 0.55:
                soccerRobot.leftSpeed  = MAX_SPEED
                soccerRobot.rightSpeed = 0.3 * MAX_SPEED

            else:
                # move forward slowly
                soccerRobot.leftSpeed  = 0.5 * MAX_SPEED
                soccerRobot.rightSpeed = 0.5 * MAX_SPEED

                # centered → start bypass
                if cam_width * 0.47 < ball_x < cam_width * 0.53:
                    print("STATE → ORBIT (BYPASS)")
                    soccerRobot.v_step = 0
                    state = "ORBIT"

    # ---------------- ORBIT (LOCKED BYPASS) ----------------
    #Ignore functions the orbit functions in soccerRobot
    elif state == "ORBIT":
        soccerRobot.v_step += 1

        # 🔥 slight curve forward (prevents hitting ball)

        # ignore vision completely during this phase
        if soccerRobot.v_step < 40 and soccerRobot.v_step >= 20:
            soccerRobot.leftSpeed  = 0.8 * MAX_SPEED
            soccerRobot.rightSpeed = 0.6 * MAX_SPEED
        else:
            soccerRobot.leftSpeed  = 0.6 * MAX_SPEED
            soccerRobot.rightSpeed = 0.8 * MAX_SPEED
            
        if soccerRobot.v_step >= 40:
            soccerRobot.v_step = 0
            print("BYPASS DONE → SEARCH")
            state = "SEARCH"

    # ---------------- ATTACK ----------------
    elif state == "ATTACK":
        if not soccerRobot.canSeeBall:
            print("Lost ball → SEARCH")
            state = "SEARCH"

        else:
            ball_x = soccerRobot.ballXPos

            if ball_x < cam_width * 0.4:
                soccerRobot.leftSpeed  = 0.5 * MAX_SPEED
                soccerRobot.rightSpeed = MAX_SPEED

            elif ball_x > cam_width * 0.6:
                soccerRobot.leftSpeed  = MAX_SPEED
                soccerRobot.rightSpeed = 0.5 * MAX_SPEED

            else:
                soccerRobot.leftSpeed  = MAX_SPEED
                soccerRobot.rightSpeed = MAX_SPEED

    # ---------------- APPLY ----------------
    soccerRobot.setSpeed()