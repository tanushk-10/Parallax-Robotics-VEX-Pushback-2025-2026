# Competition program. This is the single file we load for a match.
#
# Field control runs one program for the whole match. It calls
# autonomous() for the 15 second period, then user_control() for the
# 1:45 driver period. There is no swapping programs in between, which is
# why the auton and the driver code have to live in the same file.
#
# This is driver_control.py and auton.py merged. Where the two disagreed
# we kept the driver_control version, since that is the one that has
# actually been driven. The one deliberate difference is
# GRAB_START_CLOSED, which is True here so the claw boots already holding
# the preload pin.
#
# Port map and controls are the same as driver_control.py.

from vex import *
import math

brain = Brain()
controller = Controller()

BUILD = "comp v1"
last_turn = ""

left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(left_motor_a, left_motor_b)

right_motor_a = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
right_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(right_motor_a, right_motor_b)

LIFT_CARTRIDGE = GearSetting.RATIO_18_1

LIFT_LEFT_PORT = 10
LIFT_RIGHT_PORT = 9

lift_left = Motor(
    getattr(Ports, "PORT" + str(LIFT_LEFT_PORT)),
    LIFT_CARTRIDGE,
    False
)

lift_right = Motor(
    getattr(Ports, "PORT" + str(LIFT_RIGHT_PORT)),
    LIFT_CARTRIDGE,
    True
)

lift = MotorGroup(lift_left, lift_right)

intake_conveyor = Motor(
    Ports.PORT8,
    GearSetting.RATIO_18_1,
    False
)

claw_pivot = DigitalOut(brain.three_wire_port.a)
claw_grab  = DigitalOut(brain.three_wire_port.b)

DEADBAND = 5

MECH_SPEED = 100

PIVOT_START_UP    = False
# True here, unlike the bench and tuning files. The claw has to boot
# already gripping the preload pin, because a match starts with it in
# our hand.
GRAB_START_CLOSED = True

MAX_TORQUE_PCT = 100

USE_MAX_LIMIT = False
LIFT_MAX_DEG = 1000

# Raw volts, not percent. A percent command is a velocity request, and
# the motor's own control loop backs off long before a 1:1 DR4B breaks
# free.
UP_VOLTS = 12.0
DOWN_PCT = 40
SLEW_VOLTS_PER_LOOP = 3.0

HOLD_SPEED_PCT = 100
CALIBRATING = False

THERMAL_GUARD = False
TEMP_CUTOFF_C = 50
TEMP_RESUME_C = 45

STALL_GUARD = True
STALL_VEL_RPM = 2
STALL_MS = 1200
STALL_DOWN_MS = 300
STALL_ARM_VOLTS = 6.0

HOMING_PCT = 25
HOMING_TORQUE_PCT = 30
HOMING_GRACE_MS = 300
HOMING_SETTLE_MS = 250
HOMING_TIMEOUT_MS = 2500

SCREEN_UPDATE_MS = 250

HOME_LIFT_FIRST = True
START_DELAY_MS = 0

WHEEL_CIRCUMFERENCE = 12.56
# Tape says 14.5 across the wheels. 10.6 is the number that makes a
# commanded 90 come out as an actual 90, because the wheels scrub
# sideways during a pivot.
TRACK_WIDTH = 10.6
DRIVE_MOTOR_RPM = 200

# 2:7, small gear on the motor. Confirmed by commanding a known distance
# and measuring the result, not by pushing the robot by hand.
DRIVE_GEAR_RATIO = 3.5

ROBOT_LENGTH_IN = 18.0
ROBOT_WIDTH_IN = 18.0

TURN_CENTER_FROM_BACK_IN = ROBOT_LENGTH_IN / 2.0

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
MIN_DRIVE_PCT = 8
DRIVE_TOL_IN = 0.5
TURN_TOL_DEG = 2.0
SETTLE_SPEED_IN_S = 1.0
SETTLE_MS = 100
MOVE_TIMEOUT_FACTOR = 2.5

# Half a second of margin inside the 15 second autonomous period.
AUTON_TIME_LIMIT_MS = 14500

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

step_number = 0

lift_volts = 0.0
lift_mode = ""
stall_timer = 0
stall_lock = False
thermal_lock = False
is_homed = False
screen_timer = 0
on_stop = False
lift_verdict = ""
lift_press_moved = False

pivot_up = False
grab_closed = False
pivot_btn_prev = False
grab_btn_prev = False


def clamp(v, lo, hi):
    if v < lo:
        return lo

    if v > hi:
        return hi

    return v


def lift_home():

    global lift_volts
    global lift_mode
    global stall_timer
    global stall_lock
    global thermal_lock
    global is_homed
    global on_stop

    lift.set_stopping(BRAKE)
    lift.set_max_torque(
        HOMING_TORQUE_PCT,
        PERCENT
    )

    controller.screen.set_cursor(1, 1)
    controller.screen.print(
        "HOMING LIFT...    "
    )

    lift.spin(
        REVERSE,
        HOMING_PCT,
        PERCENT
    )

    elapsed = 0
    settled = 0

    while elapsed < HOMING_TIMEOUT_MS:

        wait(20, MSEC)

        elapsed += 20

        if elapsed < HOMING_GRACE_MS:
            continue

        if abs(lift.velocity(RPM)) < STALL_VEL_RPM:

            settled += 20

            if settled >= HOMING_SETTLE_MS:
                break

        else:

            settled = 0

    lift.stop()

    lift_left.set_position(
        0,
        DEGREES
    )

    lift_right.set_position(
        0,
        DEGREES
    )

    lift.set_max_torque(
        MAX_TORQUE_PCT,
        PERCENT
    )

    lift_volts = 0.0
    lift_mode = ""
    stall_timer = 0
    stall_lock = False
    thermal_lock = False
    is_homed = True
    on_stop = True

    controller.screen.set_cursor(1, 1)
    controller.screen.print(
        "LIFT READY        "
    )


def lift_diagnostic():

    global lift_volts
    global lift_mode
    global stall_timer
    global stall_lock

    lift.stop()

    lift.set_max_torque(
        100,
        PERCENT
    )

    controller.screen.clear_screen()

    controller.screen.set_cursor(
        1,
        1
    )

    controller.screen.print(
        "DIAG: hands clear "
    )

    wait(1000, MSEC)

    results = []

    for driven, passive in (
        (lift_left, lift_right),
        (lift_right, lift_left)
    ):

        passive.set_stopping(
            COAST
        )

        passive.stop()

        driven.spin(
            FORWARD,
            8,
            VOLT
        )

        wait(
            400,
            MSEC
        )

        dv = driven.velocity(
            RPM
        )

        pv = passive.velocity(
            RPM
        )

        driven.set_stopping(
            BRAKE
        )

        driven.stop()

        passive.set_stopping(
            BRAKE
        )

        passive.stop()

        wait(
            600,
            MSEC
        )

        results.append(
            (dv, pv)
        )

    MOVED = 3

    fighting = False
    not_linked = False
    no_move = False

    for dv, pv in results:

        if abs(dv) < MOVED:

            no_move = True

        elif abs(pv) < MOVED:

            not_linked = True

        elif (dv > 0) != (pv > 0):

            fighting = True

    controller.screen.clear_screen()

    controller.screen.set_cursor(
        1,
        1
    )

    controller.screen.print(
        "L>{:+4.0f} R{:+4.0f}".format(
            results[0][0],
            results[0][1]
        )
    )

    controller.screen.set_cursor(
        2,
        1
    )

    controller.screen.print(
        "R>{:+4.0f} L{:+4.0f}".format(
            results[1][0],
            results[1][1]
        )
    )

    controller.screen.set_cursor(
        3,
        1
    )

    if fighting:

        controller.screen.print(
            "FIGHTING flip one "
        )

    elif not_linked:

        controller.screen.print(
            "NOT LINKED        "
        )

    elif no_move:

        controller.screen.print(
            "NO MOVE too heavy "
        )

    else:

        controller.screen.print(
            "OK same dir       "
        )

    wait(
        5000,
        MSEC
    )

    controller.screen.clear_screen()

    lift.set_max_torque(
        MAX_TORQUE_PCT,
        PERCENT
    )

    lift_volts = 0.0
    lift_mode = ""
    stall_timer = 0
    stall_lock = False


def ensure_homed():

    if not is_homed:

        lift_home()


def lift_missing():

    names = []

    if not lift_left.installed():

        names.append(
            "LEFT p" + str(
                LIFT_LEFT_PORT
            )
        )

    if not lift_right.installed():

        names.append(
            "RIGHT p" + str(
                LIFT_RIGHT_PORT
            )
        )

    return names


def lift_check():

    missing = lift_missing()

    if not missing:
        return

    brain.screen.clear_screen()

    brain.screen.set_cursor(
        1,
        1
    )

    brain.screen.print(
        "LIFT MOTOR NOT FOUND:"
    )

    row = 2

    for name in missing:

        brain.screen.set_cursor(
            row,
            1
        )

        brain.screen.print(
            "  " + name
        )

        row += 1

    brain.screen.set_cursor(
        row + 1,
        1
    )

    brain.screen.print(
        "Check cable and port"
    )

    controller.screen.set_cursor(
        3,
        1
    )

    controller.screen.print(
        "NO "
        + missing[0]
        + "     "
    )

    controller.rumble(
        "- - -"
    )

    wait(
        1500,
        MSEC
    )


def lift_telemetry(
    mode,
    pos
):

    global lift_verdict
    global lift_press_moved

    rows = []
    amps = []

    for name, port, m in (
        (
            "L",
            LIFT_LEFT_PORT,
            lift_left
        ),
        (
            "R",
            LIFT_RIGHT_PORT,
            lift_right
        )
    ):

        if m.installed():

            a = m.current(
                CurrentUnits.AMP
            )

            amps.append(a)

            rows.append(
                "{} p{:<2} {:>4.0f}rpm {:>4.1f}A {:>3.0f}C   ".format(
                    name,
                    port,
                    m.velocity(RPM),
                    a,
                    m.temperature(
                        TemperatureUnits.CELSIUS
                    )
                )
            )

        else:

            amps.append(
                -1.0
            )

            rows.append(
                "{} p{:<2} NOT FOUND              ".format(
                    name,
                    port
                )
            )

    if mode != "UP" and mode != "STALL":

        lift_press_moved = False

    moving = (
        abs(
            lift.velocity(
                RPM
            )
        )
        >= STALL_VEL_RPM
    )

    if mode == "UP" and moving:

        lift_press_moved = True

    if (
        mode == "UP"
        and lift_volts
        >= STALL_ARM_VOLTS
    ):

        la = amps[0]
        ra = amps[1]

        if la < 0 or ra < 0:

            lift_verdict = (
                "MOTOR NOT FOUND - see above"
            )

        elif (
            max(
                la,
                ra
            )
            > 0.6
            and min(
                la,
                ra
            )
            < 0.25
            * max(
                la,
                ra
            )
        ):

            if la > ra:

                side = "LEFT"

            else:

                side = "RIGHT"

            lift_verdict = (
                "ONLY "
                + side
                + " PULLS - other idle"
            )

        elif (
            min(
                la,
                ra
            )
            > 1.8
            and not moving
        ):

            if lift_press_moved:

                lift_verdict = (
                    "BOTH PULL, THEN STALLED"
                )

            else:

                lift_verdict = (
                    "BOTH MAXED, NO MOVE (B+UP)"
                )

        else:

            lift_verdict = (
                "BOTH PULLING"
            )

    brain.screen.set_cursor(
        1,
        1
    )

    brain.screen.print(
        "LIFT {:<6} pos {:>5.0f} deg      ".format(
            mode,
            pos
        )
    )

    brain.screen.set_cursor(
        2,
        1
    )

    brain.screen.print(
        rows[0]
    )

    brain.screen.set_cursor(
        3,
        1
    )

    brain.screen.print(
        rows[1]
    )

    brain.screen.set_cursor(
        4,
        1
    )

    if lift_verdict:

        message = lift_verdict

    else:

        message = (
            "hold L1 to test"
        )

    brain.screen.print(
        "> {:<30}".format(
            message
        )
    )


def lift_hold_here():

    lift.set_stopping(
        HOLD
    )

    lift_left.spin_to_position(
        lift_left.position(
            DEGREES
        ),
        DEGREES,
        HOLD_SPEED_PCT,
        PERCENT,
        wait=False
    )

    lift_right.spin_to_position(
        lift_right.position(
            DEGREES
        ),
        DEGREES,
        HOLD_SPEED_PCT,
        PERCENT,
        wait=False
    )


def lift_control():

    global lift_volts
    global lift_mode
    global stall_timer
    global stall_lock
    global thermal_lock
    global screen_timer
    global on_stop

    if (
        controller.buttonB.pressing()
        and controller.buttonDown.pressing()
    ):

        lift_home()
        return

    if (
        controller.buttonB.pressing()
        and controller.buttonUp.pressing()
    ):

        lift_diagnostic()
        return

    pos = lift.position(
        DEGREES
    )

    vel = abs(
        lift.velocity(
            RPM
        )
    )

    temp = max(
        lift_left.temperature(
            TemperatureUnits.CELSIUS
        ),
        lift_right.temperature(
            TemperatureUnits.CELSIUS
        )
    )

    if not THERMAL_GUARD:

        thermal_lock = False

    else:

        if temp >= TEMP_CUTOFF_C:

            thermal_lock = True

        if (
            thermal_lock
            and temp
            <= TEMP_RESUME_C
        ):

            thermal_lock = False

    up = (
        controller.buttonL1.pressing()
    )

    down = (
        controller.buttonL2.pressing()
    )

    if not up and not down:

        stall_lock = False

    blocked = (
        thermal_lock
        or stall_lock
    )

    at_top = (
        USE_MAX_LIMIT
        and pos
        >= LIFT_MAX_DEG
    )

    if (
        up
        and not blocked
        and not at_top
    ):

        mode = "UP"

    elif (
        down
        and not up
        and not blocked
    ):

        mode = "DOWN"

    elif thermal_lock:

        mode = "COOL"

    elif CALIBRATING or on_stop:

        mode = "REST"

    else:

        mode = "HOLD"

    if mode == "UP":

        on_stop = False

        lift_volts = min(
            UP_VOLTS,
            lift_volts
            + SLEW_VOLTS_PER_LOOP
        )

        lift_left.spin(
            FORWARD,
            lift_volts,
            VOLT
        )

        lift_right.spin(
            FORWARD,
            lift_volts,
            VOLT
        )

    else:

        lift_volts = 0.0

        if mode != lift_mode:

            if mode == "DOWN":

                lift.spin(
                    REVERSE,
                    DOWN_PCT,
                    PERCENT
                )

            elif mode == "HOLD":

                lift_hold_here()

            else:

                lift.set_stopping(
                    BRAKE
                )

                lift.stop()

    lift_mode = mode

    if mode == "UP":

        watching = (
            STALL_GUARD
            and lift_volts
            >= STALL_ARM_VOLTS
        )

        stall_limit = (
            STALL_MS
        )

    else:

        watching = (
            mode == "DOWN"
        )

        stall_limit = (
            STALL_DOWN_MS
        )

    if (
        watching
        and vel
        < STALL_VEL_RPM
    ):

        stall_timer += 20

    else:

        stall_timer = 0

    if (
        stall_timer
        > stall_limit
    ):

        stall_lock = True

        stall_timer = 0

        if mode == "DOWN":

            on_stop = True

    screen_timer += 20

    if (
        screen_timer
        >= SCREEN_UPDATE_MS
    ):

        screen_timer = 0

        if thermal_lock:

            label = "HOT"

        elif (
            stall_lock
            and not on_stop
        ):

            label = "STALL"

        elif (
            up
            and at_top
        ):

            label = "MAX"

        else:

            label = mode

        controller.screen.set_cursor(
            1,
            1
        )

        controller.screen.print(
            "L{:>3.0f}C {:>4.0f} {:<6}".format(
                temp,
                pos,
                label
            )
        )

        lift_telemetry(
            label,
            pos
        )


def intake_control():

    if (
        controller.buttonR1.pressing()
    ):

        intake_conveyor.spin(
            REVERSE,
            MECH_SPEED,
            PERCENT
        )

    elif (
        controller.buttonR2.pressing()
    ):

        intake_conveyor.spin(
            FORWARD,
            MECH_SPEED,
            PERCENT
        )

    else:

        intake_conveyor.stop()


def claw_screen():

    if pivot_up:
        a = "UP  "
    else:
        a = "DOWN"

    if grab_closed:
        b = "HOLD"
    else:
        b = "OPEN"

    controller.screen.set_cursor(
        2,
        1
    )

    controller.screen.print(
        "Wrist " + a + " Grip " + b
    )


def pivot_set(up):

    global pivot_up

    pivot_up = up

    claw_pivot.set(up)

    claw_screen()


def grab_set(closed):

    global grab_closed

    grab_closed = closed

    claw_grab.set(closed)

    claw_screen()


def claw_control():

    global pivot_btn_prev
    global grab_btn_prev

    pivot_btn = controller.buttonY.pressing()
    grab_btn = controller.buttonA.pressing()

    if (
        pivot_btn
        and not pivot_btn_prev
    ):

        pivot_set(
            not pivot_up
        )

    if (
        grab_btn
        and not grab_btn_prev
    ):

        grab_set(
            not grab_closed
        )

    pivot_btn_prev = pivot_btn
    grab_btn_prev = grab_btn


def drive_control():

    throttle = (
        controller.axis3.position()
    )

    steering = (
        controller.axis1.position()
    )

    if (
        abs(throttle)
        < DEADBAND
    ):

        throttle = 0

    if (
        abs(steering)
        < DEADBAND
    ):

        steering = 0

    if steering != 0:

        left_power = steering
        right_power = -steering

    else:

        left_power = throttle
        right_power = throttle

    if (
        left_power == 0
        and right_power == 0
    ):

        left_drive.stop()
        right_drive.stop()

    else:

        left_drive.spin(
            FORWARD,
            left_power,
            PERCENT
        )

        right_drive.spin(
            FORWARD,
            right_power,
            PERCENT
        )


def user_control():

    lift_check()

    ensure_homed()

    left_drive.set_stopping(
        BRAKE
    )

    right_drive.set_stopping(
        BRAKE
    )

    left_drive.set_max_torque(
        100,
        PERCENT
    )

    right_drive.set_max_torque(
        100,
        PERCENT
    )

    intake_conveyor.set_stopping(
        HOLD
    )

    intake_conveyor.stop()

    pivot_set(
        pivot_up
    )

    grab_set(
        grab_closed
    )

    while True:

        drive_control()

        lift_control()

        intake_control()

        claw_control()

        wait(
            20,
            MSEC
        )


def sign_of(x):
    if x < 0:
        return -1
    return 1


def out_of_time():
    return brain.timer.time(MSEC) >= AUTON_TIME_LIMIT_MS


def motor_deg_per_inch():
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


def time_left_ms():
    return AUTON_TIME_LIMIT_MS - brain.timer.time(MSEC)


def pause(ms):
    budget(ms)


def budget(ms):
    left = time_left_ms()
    if left <= 0:
        return
    wait(min(ms, left), MSEC)


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


def lift_to(target_deg):
    if USE_MAX_LIMIT and target_deg > LIFT_MAX_DEG:
        target_deg = LIFT_MAX_DEG
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
        grab_set(True)
    pivot_set(False)

    if HOME_LIFT_FIRST:
        step("home lift")
        lift_home()
        pause(100)

    if SPIN_TOGGLE_FIRST:
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
    pivot_set(True)
    budget(PNEUMATIC_SETTLE_MS)

    step("release pin")
    grab_set(False)
    budget(PNEUMATIC_SETTLE_MS)
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
    pivot_set(False)
    budget(PNEUMATIC_SETTLE_MS)
    lift_to(0)

    left_drive.stop()
    right_drive.stop()
    global lift_mode, on_stop
    lift_mode = ""
    on_stop = False

    status("DONE {:.1f}s".format(brain.timer.time(MSEC) / 1000.0))

pivot_up = PIVOT_START_UP
grab_closed = GRAB_START_CLOSED

claw_pivot.set(pivot_up)
claw_grab.set(grab_closed)

competition = Competition(
    user_control,
    autonomous
)
