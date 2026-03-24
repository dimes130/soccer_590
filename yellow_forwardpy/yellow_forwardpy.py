from controller import Robot, Camera, Lidar
import math

TIME_STEP = 64
MAX_SPEED = 10

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

# ------------------ MAIN LOOP ------------------
while robot.step(TIME_STEP) != -1:
    leftSpeed = 0.0
    rightSpeed = 0.0

    # ================= LIDAR READING =================
    # Get all distance readings from the lidar this timestep
    distances = get_lidar_distances()

    # Example: read the closest obstacle distance straight ahead
    front_dist = get_lidar_sector(distances, 'front')

    # Example: stop if something is within 0.2 meters in front
    # if front_dist < 0.2:
    #     set_speed(0, 0)
    #     continue

    # ================= YELLOW DETECTION =================
    image = camera.getImage()
    yellow_x_sum = 0
    yellow_count = 0
    magenta_count = 0
    cyan_count = 0
    cyan_x_sum = 0
    
    for y in range(cam_height // 2, cam_height, 2):
        for x in range(0, cam_width, 2):
            r = Camera.imageGetRed(image, cam_width, x, y)
            g = Camera.imageGetGreen(image, cam_width, x, y)
            b = Camera.imageGetBlue(image, cam_width, x, y)
            
            if is_yellow(r, g, b):
                yellow_x_sum += x
                yellow_count += 1
            if is_magenta(r, g, b):
                magenta_count += 1
            if is_cyan(r, g, b):
                cyan_count += 1
                cyan_x_sum += x
    
    if yellow_count > 10:
        ball_x = yellow_x_sum / yellow_count
    
    else:
        ball_x = None
    
    if cyan_count > 10:
        goal_x = cyan_x_sum / cyan_count
    else:
        goal_x = None
        
    if ball_x is not None:

    # BAD POSITION: goal behind ball
        robot_center = cam_width / 2

        if goal_x is not None:
        
            # goal and ball on same side of robot → bad position
            if (goal_x < robot_center and ball_x < robot_center) or \
               (goal_x > robot_center and ball_x > robot_center):
        
                # circle around ball
                leftSpeed = 0.5 * MAX_SPEED
                rightSpeed = -0.3 * MAX_SPEED
        
            else:
                # good position → push ball
                if ball_x < cam_width * 0.4:
                    leftSpeed = 0.5 * MAX_SPEED
                    rightSpeed = MAX_SPEED
        
                elif ball_x > cam_width * 0.6:
                    leftSpeed = MAX_SPEED
                    rightSpeed = 0.5 * MAX_SPEED
        
                else:
                    leftSpeed = MAX_SPEED
                    rightSpeed = MAX_SPEED
            
        else:
            # normal ball chasing
            if ball_x < cam_width * 0.4:
                leftSpeed = 0.5 * MAX_SPEED
                rightSpeed = MAX_SPEED
    
            elif ball_x > cam_width * 0.6:
                leftSpeed = MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED
    
            else:
                leftSpeed = MAX_SPEED
                rightSpeed = MAX_SPEED
    else:
        leftSpeed  = -0.4 * MAX_SPEED
        rightSpeed = 0.4 * MAX_SPEED
        
    set_speed(leftSpeed, rightSpeed)