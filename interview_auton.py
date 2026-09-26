# Autonomous routine for the 15 second period.
#
# The thing worth knowing before reading this file: the route is worked
# out, not typed in. We tell the code where the robot starts, how big it
# is, and where the goal sits on the field grid. It works out the turn
# angle and the reverse distance itself. Change START_ANGLE_DEG and every
# number after it follows automatically.
#
# Field measurements come from Game Manual v1.0, Appendix A sheets A10
# and A12, rather than from guessing off photos.
#
# Everything meant to be edited is in the block of constants below. Past
# that is the implementation, which we mostly leave alone.
#
# Test modes. Only one is True at a time, and we run them in this order
# because each depends on the one before it being right:
#
#   BRING_UP      is every motor alive and spinning the correct way
#   MEASURE_MODE  push the robot a known distance, read the gear ratio
#   SELFTEST      drive a commanded square, check distance and turns
#   TURN_TEST     one turn at a time, for trimming turn calibration
#   STEP_MODE     the real route, one step per button press
#
# All of them False with STANDALONE True runs the real route off the
# play button.

from vex import *
import math

brain = Brain()
controller = Controller()

BUILD = "v18 turntrim"

SELFTEST = False
BRING_UP = False
MEASURE_MODE = False
STEP_MODE = False

TURN_TEST = False
STANDALONE = True

MEASURE_DISTANCE_IN = 24.0
BENCH_START_DELAY_MS = 2000
START_DELAY_MS = 0

# 4 inch omnis. Measured by rolling the robot rather than with a ruler
# on the wheel, because the rollers squash under the robot's weight.
WHEEL_CIRCUMFERENCE = 12.56
# A tape measure across the wheels says 14.5, but the number that makes
# a commanded 90 degree turn come out as an actual 90 is 10.6. The
# wheels scrub sideways during a pivot, so the effective track width is
# smaller than the physical one. We trust the number that turns right.
TRACK_WIDTH = 10.6
DRIVE_MOTOR_RPM = 200

# 2:7, with the small gear on the motor. We got this wrong twice before
# settling on it. Pushing the robot by hand and reading the encoders
# gives a low answer, because the wheels slip and it is hard to push
# exactly the distance you meant to. Commanding a known distance and
# measuring what the robot actually did is the test that got it right.
DRIVE_GEAR_RATIO = 3.5

ROBOT_LENGTH_IN = 18.0
ROBOT_WIDTH_IN = 18.0

TURN_CENTER_FROM_BACK_IN = ROBOT_LENGTH_IN / 2.0

# STILL TO MEASURE. Back bumper to the centre of the held pin, with the
# lift at scoring height and the wrist folded over. It has to be at
# least half the goal base, 2.81, or the bumper reaches the goal before
# the pin does and we shove the goal instead of scoring on it.
CLAW_REACH_IN = 0.0

CLAW_SIDE_OFFSET_IN = 0.0

DISC_ON_RIGHT = False

MIRROR_TURNS = True
START_ANGLE_DEG = 45.0
DRIVE_OFF_WALL_IN = None
TURN_TRIM_DEG = 0.0

OVERSHOOT_IN = 0.0
BACK_AWAY_IN = 6.0

SPIN_TOGGLE_FIRST = True
TOGGLE_SPIN_MS = 1200
TOGGLE_SPIN_PCT = 100
TOGGLE_SPIN_DIR = REVERSE

HOME_LIFT_FIRST = True
SCORE_LIFT_DEG = 270.0
LIFT_VOLTS = 12.0
LIFT_DOWN_PCT = 35
LIFT_TOL_DEG = 5
LIFT_TIMEOUT_MS = 2500

PRELOAD_PIN = True
PNEUMATIC_SETTLE_MS = 300

KP_DRIVE_PER_IN = 5.0
KP_STRAIGHT_PER_IN = 4.0
KP_TURN_PER_DEG = 1.2
# Power floor, so the last half inch still moves instead of the P term
# fading away to nothing. Raising this made our speed-geared drive
# overshoot and hunt back and forth, so it stays low.
MIN_DRIVE_PCT = 8
DRIVE_TOL_IN = 0.5
TURN_TOL_DEG = 2.0
SETTLE_SPEED_IN_S = 1.0
SETTLE_MS = 100
MOVE_TIMEOUT_FACTOR = 2.5

# Half a second of margin inside the 15 second period. Every move checks
# this before it starts, so nothing is still running when the field cuts
# us off partway through a motion.
AUTON_TIME_LIMIT_MS = 14500

left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(left_motor_a, left_motor_b)

right_motor_a = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
right_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(right_motor_a, right_motor_b)

lift_left = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
lift_right = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
lift = MotorGroup(lift_left, lift_right)

intake_conveyor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)

claw_pivot = DigitalOut(brain.three_wire_port.a)
claw_grab = DigitalOut(brain.three_wire_port.b)

# Field geometry, all from Game Manual v1.0 Appendix A. Inside wall to
# inside wall, and the goal centre measured from the bottom left corner
# of the field the way the drawings lay it out.
FIELD_INSIDE_IN = 140.41

GOAL_X_IN = 46.66
GOAL_Y_IN = 23.11
GOAL_BASE_DIAMETER_IN = 5.61

START_X_IN = 70.20

WALL_GAP_LEFT_IN = 51.40
WALL_GAP_RIGHT_IN = 89.01

PI = 3.14159265

_SIDE = -1 if MIRROR_TURNS else 1
_HALF = ROBOT_LENGTH_IN / 2.0

GOAL_OUT_IN = GOAL_Y_IN
GOAL_LAT_IN = -_SIDE * (FIELD_INSIDE_IN / 2.0 - GOAL_X_IN)

_A = math.radians(_SIDE * START_ANGLE_DEG)

START_OUT_IN = _HALF * (abs(math.cos(_A)) + abs(math.sin(_A)))

if DRIVE_OFF_WALL_IN is None:
    if abs(START_ANGLE_DEG) < 1.0:
        DRIVE_OFF_WALL_IN = GOAL_OUT_IN - TURN_CENTER_FROM_BACK_IN
    else:
        DRIVE_OFF_WALL_IN = max(
            0.0, math.sqrt(2.0) * _HALF - START_OUT_IN) + 8.0

_c_out = START_OUT_IN + DRIVE_OFF_WALL_IN * math.cos(_A)
_c_lat = DRIVE_OFF_WALL_IN * math.sin(_A)

_v_out = GOAL_OUT_IN - _c_out
_v_lat = GOAL_LAT_IN - _c_lat
_range = math.sqrt(_v_out * _v_out + _v_lat * _v_lat)

TURN_TO_GOAL_DEG = math.degrees(math.atan2(_v_lat, _v_out) + math.pi - _A)
while TURN_TO_GOAL_DEG > 180.0:
    TURN_TO_GOAL_DEG -= 360.0
while TURN_TO_GOAL_DEG < -180.0:
    TURN_TO_GOAL_DEG += 360.0

if TURN_TO_GOAL_DEG < 0:
    TURN_TO_GOAL_DEG -= TURN_TRIM_DEG
else:
    TURN_TO_GOAL_DEG += TURN_TRIM_DEG

DRIVE_TO_GOAL_IN = -(_range - TURN_CENTER_FROM_BACK_IN
                     - CLAW_REACH_IN + OVERSHOOT_IN)

_TOUCH_RIGHT = (_SIDE * START_ANGLE_DEG) > 0
DISC_AT_WALL = (abs(START_ANGLE_DEG) < 1.0
                or _TOUCH_RIGHT == DISC_ON_RIGHT)
DISC_GAP_IN = 0.0 if DISC_AT_WALL else 2.0 * _HALF * abs(math.sin(_A))
TURN_CLEARS_WALL = _c_out >= math.sqrt(2.0) * _HALF
BUMPER_HITS_GOAL = (CLAW_REACH_IN - OVERSHOOT_IN
                    < GOAL_BASE_DIAMETER_IN / 2.0)
START_SPOT_BAD = (START_X_IN - ROBOT_WIDTH_IN / 2.0 <= WALL_GAP_LEFT_IN
                  or START_X_IN + ROBOT_WIDTH_IN / 2.0 >= WALL_GAP_RIGHT_IN)

is_homed = False
step_number = 0
last_turn = ""
bench_run = False


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def sign_of(x):
    if x < 0:
        return -1
    return 1


def out_of_time():
    # Checked before and during every move. We would rather end a step
    # early and finish tidy than get cut off in the middle of a motion.
    if SELFTEST:
        return False
    if STEP_MODE and bench_run:
        return False
    return brain.timer.time(MSEC) >= AUTON_TIME_LIMIT_MS


def motor_deg_per_inch():
    # The one conversion the whole file rests on. One wheel turn is 360
    # motor degrees divided by the gear ratio, and covers one wheel
    # circumference of ground.
    return (360.0 / WHEEL_CIRCUMFERENCE) * DRIVE_GEAR_RATIO


def top_speed_in_s(pct):
    return (DRIVE_MOTOR_RPM * pct / 100.0 / DRIVE_GEAR_RATIO
            * WHEEL_CIRCUMFERENCE / 60.0)


def move_timeout_ms(inches, pct):
    return int(1500 + MOVE_TIMEOUT_FACTOR * abs(inches)
               / top_speed_in_s(pct) * 1000)


def wheel_speed_in_s():
    rpm = (abs(left_drive.velocity(RPM))
           + abs(right_drive.velocity(RPM))) / 2.0
    return rpm * 6.0 / motor_deg_per_inch()


def reset_drive_encoders():
    left_drive.set_position(0, DEGREES)
    right_drive.set_position(0, DEGREES)


def apply_min_power(power):
    if power > 0 and power < MIN_DRIVE_PCT:
        return MIN_DRIVE_PCT
    if power < 0 and power > -MIN_DRIVE_PCT:
        return -MIN_DRIVE_PCT
    return power


def lift_position():
    return (lift_left.position(DEGREES)
            + lift_right.position(DEGREES)) / 2.0


def status(text):
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    brain.screen.print(text)
    brain.screen.set_cursor(2, 1)
    brain.screen.print("t = {:.1f} s".format(brain.timer.time(MSEC) / 1000.0))

    controller.screen.set_cursor(1, 1)
    controller.screen.print("{:<18}".format(text))

    brain.screen.set_cursor(12, 1)
    brain.screen.print("BUILD " + BUILD + "   ")
    if last_turn:
        brain.screen.set_cursor(9, 1)
        brain.screen.print(last_turn)


def step(text):
    global step_number
    step_number += 1
    status("{}. {}".format(step_number, text))

    if not (STEP_MODE and bench_run):
        return
    brain.screen.set_cursor(4, 1)
    brain.screen.print("STEP MODE - press A")
    while not controller.buttonA.pressing():
        wait(20, MSEC)
    while controller.buttonA.pressing():
        wait(20, MSEC)


def time_left_ms():
    return AUTON_TIME_LIMIT_MS - brain.timer.time(MSEC)


def pause(ms):
    budget(ms)


def budget(ms):
    left = time_left_ms()
    if left <= 0:
        return
    wait(min(ms, left), MSEC)


def countdown(ms):
    left = ms
    while left > 0:
        status("Starting in {:.0f}s".format((left + 999) / 1000))
        wait(250, MSEC)
        left -= 250


def drive_inches(inches, speed=50, timeout_ms=None):
    if timeout_ms is None:
        timeout_ms = move_timeout_ms(inches, speed)
    dpi = motor_deg_per_inch()
    target = inches * dpi
    reset_drive_encoders()

    elapsed = 0
    settled = 0
    while elapsed < timeout_ms and not out_of_time():
        l = left_drive.position(DEGREES)
        r = right_drive.position(DEGREES)
        error_in = (target - (l + r) / 2.0) / dpi

        close = abs(error_in) < DRIVE_TOL_IN
        if close and wheel_speed_in_s() < SETTLE_SPEED_IN_S:
            settled += 20
            if settled >= SETTLE_MS:
                break
        else:
            settled = 0

        power = clamp(error_in * KP_DRIVE_PER_IN, -speed, speed)
        if not close:
            power = apply_min_power(power)

        correction = ((l - r) / dpi) * KP_STRAIGHT_PER_IN

        left_drive.spin(FORWARD, power - correction, PERCENT)
        right_drive.spin(FORWARD, power + correction, PERCENT)

        wait(20, MSEC)
        elapsed += 20

    left_drive.stop()
    right_drive.stop()


def turn_degrees(angle, speed=40, timeout_ms=None):
    if timeout_ms is None:
        timeout_ms = move_timeout_ms(
            abs(angle) * PI * TRACK_WIDTH / 360.0, speed)
    dpi = motor_deg_per_inch()
    per_robot_deg = (PI * TRACK_WIDTH / 360.0) * dpi
    direction = sign_of(angle)
    target_deg = abs(angle)

    reset_drive_encoders()

    elapsed = 0
    settled = 0
    while elapsed < timeout_ms and not out_of_time():
        l = left_drive.position(DEGREES) * direction
        r = right_drive.position(DEGREES) * direction * -1
        error_deg = target_deg - ((l + r) / 2.0) / per_robot_deg

        close = abs(error_deg) < TURN_TOL_DEG
        if close and wheel_speed_in_s() < SETTLE_SPEED_IN_S:
            settled += 20
            if settled >= SETTLE_MS:
                break
        else:
            settled = 0

        power = clamp(error_deg * KP_TURN_PER_DEG, -speed, speed)
        if not close:
            power = apply_min_power(power)
        power = power * direction

        left_drive.spin(FORWARD, power, PERCENT)
        right_drive.spin(FORWARD, -power, PERCENT)

        wait(20, MSEC)
        elapsed += 20

    left_drive.stop()
    right_drive.stop()

    global last_turn
    done = ((left_drive.position(DEGREES) * direction
             + right_drive.position(DEGREES) * direction * -1) / 2.0
            ) / per_robot_deg
    last_turn = "turn ask {:.0f} enc {:.0f} {:<7}".format(
        target_deg, done, "TIMEOUT" if elapsed >= timeout_ms else "ok")
    brain.screen.set_cursor(9, 1)
    brain.screen.print(last_turn)


def wrist_up():
    claw_pivot.set(True)
    budget(PNEUMATIC_SETTLE_MS)


def wrist_down():
    claw_pivot.set(False)
    budget(PNEUMATIC_SETTLE_MS)


def grip_close():
    claw_grab.set(True)
    budget(PNEUMATIC_SETTLE_MS)


def grip_open():
    claw_grab.set(False)
    budget(PNEUMATIC_SETTLE_MS)


def lift_home():
    global is_homed

    lift.set_stopping(BRAKE)
    lift.set_max_torque(30, PERCENT)
    lift.spin(REVERSE, 25, PERCENT)

    elapsed = 0
    settled = 0
    while elapsed < 2000:
        wait(20, MSEC)
        elapsed += 20
        if elapsed < 300:
            continue
        if abs(lift.velocity(RPM)) < 5:
            settled += 20
            if settled >= 250:
                break
        else:
            settled = 0

    lift.stop()
    lift_left.set_position(0, DEGREES)
    lift_right.set_position(0, DEGREES)
    lift.set_max_torque(100, PERCENT)
    is_homed = True


def lift_to(target_deg):
    if out_of_time():
        return

    lift.set_stopping(HOLD)
    lift.set_max_torque(100, PERCENT)

    going_up = target_deg > lift_position()
    if going_up:
        lift_left.spin(FORWARD, LIFT_VOLTS, VOLT)
        lift_right.spin(FORWARD, LIFT_VOLTS, VOLT)
    else:
        lift_left.spin(REVERSE, LIFT_DOWN_PCT, PERCENT)
        lift_right.spin(REVERSE, LIFT_DOWN_PCT, PERCENT)

    elapsed = 0
    pos = lift_position()
    while elapsed < LIFT_TIMEOUT_MS and not out_of_time():
        pos = lift_position()
        if going_up and pos >= target_deg - LIFT_TOL_DEG:
            break
        if not going_up and pos <= target_deg + LIFT_TOL_DEG:
            break
        wait(20, MSEC)
        elapsed += 20

    lift_left.spin_to_position(lift_left.position(DEGREES), DEGREES,
                               100, PERCENT, wait=False)
    lift_right.spin_to_position(lift_right.position(DEGREES), DEGREES,
                                100, PERCENT, wait=False)

    if abs(pos - target_deg) >= LIFT_TOL_DEG:
        brain.screen.set_cursor(10, 1)
        brain.screen.print("LIFT SHORT {:.0f} of {:.0f}   ".format(
            pos, target_deg))


def spin_toggle():
    if not intake_conveyor.installed():
        status("NO INTAKE MOTOR p8")
        wait(700, MSEC)
        return

    intake_conveyor.set_stopping(COAST)
    intake_conveyor.set_max_torque(100, PERCENT)
    intake_conveyor.spin(TOGGLE_SPIN_DIR, TOGGLE_SPIN_PCT, PERCENT)

    waited = 0
    while waited < TOGGLE_SPIN_MS and not out_of_time():
        wait(20, MSEC)
        waited += 20

    intake_conveyor.stop()
    intake_conveyor.set_stopping(HOLD)


def autonomous():
    global step_number
    step_number = 0

    left_drive.set_stopping(BRAKE)
    right_drive.set_stopping(BRAKE)
    intake_conveyor.set_stopping(HOLD)
    intake_conveyor.stop()

    brain.timer.clear()

    if START_DELAY_MS > 0:
        status("Starting in {:.0f}s".format(START_DELAY_MS / 1000.0))
        wait(START_DELAY_MS, MSEC)

    if PRELOAD_PIN:
        claw_grab.set(True)
    claw_pivot.set(False)

    if HOME_LIFT_FIRST and not BRING_UP:
        step("home lift")
        lift_home()
        pause(100)

    if SPIN_TOGGLE_FIRST and not BRING_UP:
        step("spin toggle")
        spin_toggle()
        pause(100)

    step("off wall {:.1f}in".format(DRIVE_OFF_WALL_IN))
    drive_inches(DRIVE_OFF_WALL_IN, 80)
    pause(150)

    step("turn {:.0f}deg".format(TURN_TO_GOAL_DEG))
    turn_degrees(TURN_TO_GOAL_DEG, 50)
    pause(150)

    step("raise to {:.0f}deg".format(SCORE_LIFT_DEG))
    lift_to(SCORE_LIFT_DEG)
    pause(100)

    if abs(DRIVE_TO_GOAL_IN) > 0.1:
        step("reverse {:.1f}in".format(abs(DRIVE_TO_GOAL_IN)))
        drive_inches(DRIVE_TO_GOAL_IN, 70)
        pause(150)

    step("wrist over goal")
    wrist_up()

    step("release pin")
    grip_open()
    pause(200)

    if out_of_time():
        left_drive.stop()
        right_drive.stop()
        status("STOPPED {:.1f}s".format(brain.timer.time(MSEC) / 1000.0))
        return

    step("clear goal {:.0f}in".format(BACK_AWAY_IN))
    drive_inches(BACK_AWAY_IN, 80)
    pause(100)

    step("reset")
    wrist_down()
    lift_to(0)

    left_drive.stop()
    right_drive.stop()
    status("DONE {:.1f}s".format(brain.timer.time(MSEC) / 1000.0))


def turn_test():
    brain.timer.clear()
    status("square it on a seam")
    wait(3000, MSEC)
    for i in range(4):
        step("turn {} of 4".format(i + 1))
        turn_degrees(90.0, 40)
        pause(1200)
    left_drive.stop()
    right_drive.stop()
    status("TURN TEST DONE")
    brain.screen.set_cursor(6, 1)
    brain.screen.print("past start -> LOWER TRACK_WIDTH")
    brain.screen.set_cursor(7, 1)
    brain.screen.print("short       -> RAISE it")


def selftest():
    brain.timer.clear()

    step("drive fwd 12in")
    drive_inches(12.0, 60)
    pause(700)

    step("drive back 12in")
    drive_inches(-12.0, 60)
    pause(700)

    step("turn RIGHT 90")
    turn_degrees(90.0, 40)
    pause(4000)

    step("turn LEFT 90")
    turn_degrees(-90.0, 40)
    pause(4000)

    step("lift UP")
    lift_to(SCORE_LIFT_DEG)
    pause(900)

    step("lift DOWN")
    lift_to(0.0)
    pause(900)

    step("wrist UP")
    wrist_up()
    pause(900)

    step("wrist DOWN")
    wrist_down()
    pause(900)

    step("grip OPEN")
    grip_open()
    pause(900)

    step("grip CLOSE")
    grip_close()
    pause(900)

    step("intake IN")
    intake_conveyor.set_max_torque(100, PERCENT)
    intake_conveyor.spin(REVERSE, 100, PERCENT)
    pause(1500)
    intake_conveyor.stop()
    pause(500)

    step("intake OUT")
    intake_conveyor.spin(FORWARD, 100, PERCENT)
    pause(1500)
    intake_conveyor.stop()

    left_drive.stop()
    right_drive.stop()
    status("SELFTEST DONE")


def measure_mode():
    left_drive.set_stopping(COAST)
    right_drive.set_stopping(COAST)
    left_drive.stop()
    right_drive.stop()
    lift.set_stopping(COAST)
    lift.stop()
    reset_drive_encoders()
    lift_left.set_position(0, DEGREES)
    lift_right.set_position(0, DEGREES)

    wrist = False
    grip = PRELOAD_PIN
    claw_pivot.set(wrist)
    claw_grab.set(grip)
    y_prev = False
    a_prev = False
    loops = 0

    brain.screen.clear_screen()

    while True:
        y = controller.buttonY.pressing()
        a = controller.buttonA.pressing()
        if y and not y_prev:
            wrist = not wrist
            claw_pivot.set(wrist)
        if a and not a_prev:
            grip = not grip
            claw_grab.set(grip)
        y_prev = y
        a_prev = a

        loops += 1
        wait(20, MSEC)
        if loops % 5 != 0:
            continue

        l = left_drive.position(DEGREES)
        r = right_drive.position(DEGREES)
        ratio = ((l + r) / 2.0) * WHEEL_CIRCUMFERENCE / (
            MEASURE_DISTANCE_IN * 360.0)

        brain.screen.set_cursor(1, 1)
        brain.screen.print("MEASURE: push robot fwd      ")
        brain.screen.set_cursor(2, 1)
        brain.screen.print("{:.0f} in, then read:        ".format(
            MEASURE_DISTANCE_IN))
        brain.screen.set_cursor(4, 1)
        brain.screen.print("motor deg L{:>6.0f} R{:>6.0f} ".format(l, r))
        brain.screen.set_cursor(6, 1)
        brain.screen.print("DRIVE_GEAR_RATIO = {:.3f}    ".format(ratio))
        brain.screen.set_cursor(8, 1)
        brain.screen.print("(L and R should be close)    ")
        brain.screen.set_cursor(10, 1)
        brain.screen.print("lift {:>5.0f} deg             ".format(
            lift_position()))
        brain.screen.set_cursor(11, 1)
        brain.screen.print("Y wrist {}  A grip {}  ".format(
            "UP  " if wrist else "DOWN", "SHUT" if grip else "OPEN"))


def user_control():
    left_drive.set_stopping(BRAKE)
    right_drive.set_stopping(BRAKE)
    lift.set_stopping(HOLD)
    lift_moving = False

    while True:
        throttle = controller.axis3.position()
        steering = controller.axis1.position()
        if abs(throttle) < 5:
            throttle = 0
        if abs(steering) < 5:
            steering = 0

        left_drive.spin(FORWARD, throttle + steering, PERCENT)
        right_drive.spin(FORWARD, throttle - steering, PERCENT)

        if controller.buttonA.pressing():
            claw_grab.set(False)
        if controller.buttonY.pressing():
            claw_pivot.set(False)

        if controller.buttonL1.pressing():
            lift.spin(FORWARD, 50, PERCENT)
            lift_moving = True
        elif controller.buttonL2.pressing():
            lift.spin(REVERSE, 30, PERCENT)
            lift_moving = True
        elif lift_moving:
            lift.stop()
            lift_moving = False

        wait(20, MSEC)


def setup_problems():
    problems = []
    if START_SPOT_BAD:
        problems.append("START_X_IN: robot hits wall cups")
    if BUMPER_HITS_GOAL:
        problems.append("CLAW_REACH_IN: bumper hits goal")
    if SPIN_TOGGLE_FIRST and not DISC_AT_WALL:
        problems.append("Disc {:.1f}in off Toggle".format(DISC_GAP_IN))
    if not TURN_CLEARS_WALL:
        problems.append("Turn clips wall: raise DRIVE_OFF_WALL_IN")
    if TURN_TEST:
        problems.append("TURN_TEST on - not the route")
    if SELFTEST:
        problems.append("SELFTEST on - not the route")
    if BRING_UP:
        problems.append("BRING_UP on - no lift home, no toggle")
    if STEP_MODE:
        problems.append("STEP_MODE on")
    if START_DELAY_MS > 0:
        problems.append("START_DELAY_MS is not 0")
    return problems


def show_problems():
    brain.screen.set_cursor(4, 1)
    brain.screen.print("start {:.0f}deg  turn {:+.0f}  rev {:.1f}in  ".format(
        START_ANGLE_DEG, TURN_TO_GOAL_DEG, abs(DRIVE_TO_GOAL_IN)))

    problems = setup_problems()
    row = 6
    for text in problems:
        brain.screen.set_cursor(row, 1)
        brain.screen.print("CHECK " + text)
        row += 1
    if problems:
        controller.screen.set_cursor(3, 1)
        controller.screen.print("! {:<16}".format(problems[0][:16]))

claw_pivot.set(False)
claw_grab.set(PRELOAD_PIN)

if MEASURE_MODE:
    measure_mode()
elif STANDALONE:
    show_problems()
    countdown(BENCH_START_DELAY_MS)
    if TURN_TEST:
        turn_test()
    elif SELFTEST:
        selftest()
    else:
        autonomous()
    user_control()
else:
    show_problems()
    competition = Competition(user_control, autonomous)
