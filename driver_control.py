# ============================================================
#  FINAL DRIVER CONTROL - MAIN FILE
#  VEXcode V5 (Python)
#
#  Drivetrain : PORT11/12 left, PORT13/14 right
#  Lift (DR4B): TWO motors, PORT10 left + PORT9 right (reversed)
#               green cartridge, 1:1, mirrored gear train
#  Intake     : PORT8, one motor drives intake + conveyor
#  Claw       : two pneumatic pistons, two separate jobs
#               port A = wrist, pivots the claw up and down
#               port B = fingers, grab and release the pin
#
#  Controls:
#    Left stick vertical  (axis3) - forward/backward
#    Right stick horiz.   (axis1) - pivot turning
#    L1 - lift up
#    L2 - lift down
#    R1 - intake + conveyor in
#    R2 - intake + conveyor out
#    A  - claw fingers grab / release (toggle)
#    Y  - claw wrist pivot up / down (toggle)
#    B + DOWN - re-home the lift
#    B + UP   - lift motor direction diagnostic
# ============================================================

from vex import *

brain = Brain()
controller = Controller()

# ============================================================
#  DRIVETRAIN MOTORS
# ============================================================

left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(left_motor_a, left_motor_b)

right_motor_a = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
right_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(right_motor_a, right_motor_b)


# ============================================================
#  LIFT MOTORS
# ============================================================

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


# ============================================================
#  INTAKE
# ============================================================

intake_conveyor = Motor(
    Ports.PORT8,
    GearSetting.RATIO_18_1,
    False
)


# ============================================================
#  CLAW
# ============================================================

# Two pistons doing two completely different jobs, so they get
# two separate buttons. Never wire these together.
#
# Port A = the wrist. Pivots the whole claw up and down.
# Port B = the fingers. Opens and closes on the pin.
#
# They have to stay independent because we need to pivot while
# still holding a pin, and let go without the wrist moving. The
# rules also let us carry a pin and a cup at the same time, so
# losing one grip should never cost us the other.
claw_pivot = DigitalOut(brain.three_wire_port.a)
claw_grab  = DigitalOut(brain.three_wire_port.b)


# ============================================================
#  TUNING CONSTANTS
# ============================================================

# ---- Drivetrain ----
DEADBAND = 5

# ---- Intake ----
MECH_SPEED = 100

# ---- Claw ----
# Where both pistons sit when the program boots. We want a
# known state every time, not wherever the air left them.
#
# Starting the wrist DOWN and the fingers OPEN is the safe
# combo: nothing is sticking up to hit the 18" sizing box at
# inspection, and we are ready to grab straight away.
#
# If a piston boots the wrong way, flip its flag here. Swapping
# the two air lines on that cylinder does the same thing.
PIVOT_START_UP    = False
GRAB_START_CLOSED = False

# ---- Lift torque ----
MAX_TORQUE_PCT = 100

# ---- Lift upper limit ----
USE_MAX_LIMIT = False
LIFT_MAX_DEG = 135

# ---- Lift drive ----
UP_VOLTS = 12.0
DOWN_PCT = 40
SLEW_VOLTS_PER_LOOP = 3.0

# ---- Lift hold ----
HOLD_SPEED_PCT = 100
CALIBRATING = False

# ---- Thermal protection ----
THERMAL_GUARD = False
TEMP_CUTOFF_C = 50
TEMP_RESUME_C = 45

# ---- Stall protection ----
STALL_GUARD = True
STALL_VEL_RPM = 2
STALL_MS = 1200
STALL_DOWN_MS = 300
STALL_ARM_VOLTS = 6.0

# ---- Homing ----
HOMING_PCT = 25
HOMING_TORQUE_PCT = 30
HOMING_GRACE_MS = 300
HOMING_SETTLE_MS = 250
HOMING_TIMEOUT_MS = 2500

# ---- Screen ----
SCREEN_UPDATE_MS = 250


# ============================================================
#  STATE
# ============================================================

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

pivot_up = False        # is the wrist raised
grab_closed = False     # are the fingers clamped
pivot_btn_prev = False  # Y last loop, so a hold counts as one press
grab_btn_prev = False   # A last loop, same idea


# ============================================================
#  UTILITIES
# ============================================================

def clamp(v, lo, hi):
    if v < lo:
        return lo

    if v > hi:
        return hi

    return v


# ============================================================
#  LIFT HOMING
# ============================================================

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


# ============================================================
#  LIFT DIAGNOSTIC
# ============================================================

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


# ============================================================
#  ENSURE HOMED
# ============================================================

def ensure_homed():

    if not is_homed:

        lift_home()


# ============================================================
#  LIFT MOTOR CHECK
# ============================================================

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


# ============================================================
#  LIFT TELEMETRY
# ============================================================

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


# ============================================================
#  LIFT HOLD
# ============================================================

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


# ============================================================
#  LIFT CONTROL
# ============================================================

def lift_control():

    global lift_volts
    global lift_mode
    global stall_timer
    global stall_lock
    global thermal_lock
    global screen_timer
    global on_stop

    # B + DOWN = re-home
    if (
        controller.buttonB.pressing()
        and controller.buttonDown.pressing()
    ):

        lift_home()
        return

    # B + UP = diagnostic
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

    # thermal guard
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

    # ==========================
    # APPLY LIFT COMMAND
    # ==========================

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


    # ==========================
    # STALL PROTECTION
    # ==========================

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


    # ==========================
    # DRIVER FEEDBACK
    # ==========================

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


# ============================================================
#  INTAKE CONTROL
#
#  R1 = intake in
#  R2 = intake out
# ============================================================

def intake_control():

    if (
        controller.buttonR1.pressing()
    ):

        intake_conveyor.spin(
            FORWARD,
            MECH_SPEED,
            PERCENT
        )

    elif (
        controller.buttonR2.pressing()
    ):

        intake_conveyor.spin(
            REVERSE,
            MECH_SPEED,
            PERCENT
        )

    else:

        intake_conveyor.stop()


# ============================================================
#  CLAW CONTROL
# ============================================================

def claw_screen():

    # Both pistons on one line so the driver can see the whole
    # claw at a glance without reading two separate messages.
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

    # Y = wrist up/down. A = fingers open/closed.
    # Both are toggles: tap once, it stays put until you tap
    # again. You do NOT have to hold the button to keep gripping,
    # which matters because air only moves when the state
    # changes -- holding a button would not use more air, but
    # forgetting to hold one would drop the pin.
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


# ============================================================
#  DRIVE CONTROL
#
#  Left stick = straight forward/backward
#  Right stick = pivot turn
#
#  If steering is being used, it overrides throttle.
# ============================================================

def drive_control():

    throttle = (
        controller.axis3.position()
    )

    steering = (
        controller.axis1.position()
    )

    # deadband
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


    # ==========================
    # PIVOT TURNING
    # ==========================

    if steering != 0:

        left_power = steering
        right_power = -steering

    else:

        left_power = throttle
        right_power = throttle


    # ==========================
    # APPLY DRIVE POWER
    # ==========================

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


# ============================================================
#  DRIVER CONTROL
# ============================================================

def user_control():

    # check lift motors
    lift_check()

    # home lift
    ensure_homed()


    # ==========================
    # DRIVETRAIN SETTINGS
    # ==========================

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


    # ==========================
    # INTAKE SETTINGS
    # ==========================

    intake_conveyor.set_stopping(
        HOLD
    )

    intake_conveyor.stop()


    # ==========================
    # CLAW INITIAL STATE
    # ==========================

    # Re-send both pistons at the start of driver control. The
    # auton period may have left them anywhere, and this also
    # puts the current state back on the controller screen.
    pivot_set(
        pivot_up
    )

    grab_set(
        grab_closed
    )


    # ==========================
    # MAIN DRIVER LOOP
    # ==========================

    while True:

        drive_control()

        lift_control()

        intake_control()

        claw_control()

        wait(
            20,
            MSEC
        )


# ============================================================
#  AUTONOMOUS
# ============================================================

def autonomous():

    ensure_homed()

    # Add autonomous routine here
    pass


# ============================================================
#  COMPETITION SETUP
# ============================================================

# Put the claw in a known state as soon as the program loads,
# before a match ever starts. Otherwise it sits wherever the
# air pressure left it from last run.
pivot_up = PIVOT_START_UP
grab_closed = GRAB_START_CLOSED

claw_pivot.set(pivot_up)
claw_grab.set(grab_closed)

competition = Competition(
    user_control,
    autonomous
)
