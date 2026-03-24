from controller import Robot, Keyboard

TIME_STEP = 64
MAX_SPEED = 6.0

robot = Robot()

keyboard = Keyboard()
keyboard.enable(TIME_STEP)

wheel_names = [
    'front_left_wheel',
    'front_right_wheel',
    'back_left_wheel',
    'back_right_wheel'
]

wheels = []

for name in wheel_names:
    wheel = robot.getDevice(name)
    wheel.setPosition(float('inf'))
    wheel.setVelocity(0.0)
    wheels.append(wheel)


def set_speed(left, right):
    wheels[0].setVelocity(left)
    wheels[2].setVelocity(left)
    wheels[1].setVelocity(right)
    wheels[3].setVelocity(right)


while robot.step(TIME_STEP) != -1:

    key = keyboard.getKey()

    if key == Keyboard.UP:
        set_speed(MAX_SPEED, MAX_SPEED)

    elif key == Keyboard.DOWN:
        set_speed(-MAX_SPEED, -MAX_SPEED)

    elif key == Keyboard.LEFT:
        set_speed(-MAX_SPEED, MAX_SPEED)

    elif key == Keyboard.RIGHT:
        set_speed(MAX_SPEED, -MAX_SPEED)

    else:
        set_speed(0, 0)