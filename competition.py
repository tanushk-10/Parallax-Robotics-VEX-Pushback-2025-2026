# ============================================================
#  COMPETITION MAIN - driver control + scoring auton
#  ONE program for both match periods. This is the match file.
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
import math

brain = Brain()
controller = Controller()

BUILD = "comp v1"
last_turn = ""   # last turn result, kept on the brain screen

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
GRAB_START_CLOSED = True   # MUST be True: holds the preload pin

# ---- Lift torque ----
MAX_TORQUE_PCT = 100

# ---- Lift upper limit ----
USE_MAX_LIMIT = False
LIFT_MAX_DEG = 675

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
#  AUTON SETTINGS  -  merged in from auton.py
# ============================================================

HOME_LIFT_FIRST = True
START_DELAY_MS = 0

# ---------------- DRIVETRAIN ----------------

WHEEL_CIRCUMFERENCE = 12.56  # 4 in wheel. A 3.25 in wheel is 10.21
# <<<<< FIX ME 2: NEVER MEASURED. Decides whether a 54 degree turn
#       is really 54.
#
#       SIDE TO SIDE, across the robot - NOT front to back:
#
#             FRONT (intake)
#          +-----------------+
#         (L)               (R)
#         (L)               (R)
#          +-----------------+
#             BACK (lift, claw, disc)
#          |<-- TRACK_WIDTH -->|
#
#       Centre of the left tread to centre of the right tread,
#       perpendicular to the way the robot drives. Four wheels:
#       either pair, same number.
#
#       Pivoting in place, the wheels trace a circle whose
#       DIAMETER is this gap, and that circle is what turns
#       "54 degrees" into inches of wheel travel.
#
#       TRIMMING IT. Run SELFTEST, watch "turn RIGHT 90", and
#       measure what it actually turned. Then:
#
#            new TRACK_WIDTH = 14.5 * 90 / (degrees it really turned)
#
#       turned  80 -> 16.31     turned  95 -> 13.74
#       turned  85 -> 15.35     turned 100 -> 13.05
#       turned  90 -> 14.50     turned 105 -> 12.43
#
#       Under-turns -> RAISE it. Over-turns -> LOWER it.
#
#       If RIGHT and LEFT are off by DIFFERENT amounts, this
#       constant will not fix that - both turns share it. That is
#       one side slipping or dragging, and it is mechanical.
#
#       (Front to back is a different constant: see
#        TURN_CENTER_FROM_BACK_IN below.)
TRACK_WIDTH = 10.6           # TRIMMED BY TEST, not by tape.
                             # tape 14.5 -> over-turned ~110 -> 11.86
                             # 11.86 -> still slightly over -> 11.25
                             # 11.25 -> still over -> 10.6 (current)
                             # still over? new = 10.6 * 90 / actual
DRIVE_MOTOR_RPM = 200        # green cartridge

# Motor turns per wheel turn. 3.5 = our 2:7 gearing, small gear on
# the motor (24t -> 84t). CONFIRMED: the selftest drives correctly
# with this value.
#
# The push test once read 2.137, which is what you get from pushing
# about 15 in instead of 24, or from the wheels slipping under your
# hand. The tooth count won. Do not "fix" this from a push test
# unless the selftest distances actually come out wrong.
DRIVE_GEAR_RATIO = 3.5


# ---------------- OUR ROBOT ----------------

ROBOT_LENGTH_IN = 18.0   # back bumper to front face
ROBOT_WIDTH_IN = 18.0    # side to side

# Back bumper to the point it pivots around (middle of the drive
# wheels). Half the length on most drivetrains.
TURN_CENTER_FROM_BACK_IN = ROBOT_LENGTH_IN / 2.0

# How far PAST the back bumper the held pin sits, with the lift up
# and the wrist over. 0 = inside our footprint. MEASURE THIS: under
# 2.8 the bumper hits the goal base instead of the pin going in.
# <<<<< FIX ME 4: NEVER MEASURED. At 0.0 the brain warns the bumper
#       hits the goal base instead of the pin going in.
CLAW_REACH_IN = 0.0

# Pin off the centre line, looking forward. + is right, - is left.
CLAW_SIDE_OFFSET_IN = 0.0

# Which side of the BACK the Toggle disc is on, looking FORWARD from
# inside the robot. Ours is LEFT. A tilted robot touches the wall
# with one back corner only, and the disc must be on that corner.
DISC_ON_RIGHT = False


# ---------------- THE ROUTE ----------------

MIRROR_TURNS = True     # True = blue RIGHT quadrant (or red LEFT)
START_ANGLE_DEG = 45.0  # tilt on the wall. 0 = square
DRIVE_OFF_WALL_IN = None  # None = worked out below. A number overrides.
# Extra degrees on the ONE auton turn, on top of whatever the route
# maths works out. Positive always means "turn further", whichever
# way it is already going.
#
# WHICH KNOB TO USE:
#   a commanded 90 in SELFTEST is not 90   -> TRACK_WIDTH, line 129
#                                             (that one scales EVERY turn)
#   90 is 90, but the auton still does not
#   end up pointing at the goal            -> TURN_TRIM_DEG, here
#                                             (only the auton turn)
#
# The second case means the geometry model is slightly off - the
# robot does not start exactly where the maths assumes - and no
# amount of TRACK_WIDTH fiddling fixes that without breaking the
# turns that are already right.
#
# Turn short of the goal by ~5 deg? Put 5 here. Overshoots it? -5.
TURN_TRIM_DEG = 40.0

OVERSHOOT_IN = 0.0      # extra on the last leg, to trim where it lands
BACK_AWAY_IN = 6.0      # pull off the goal at the end


# ---------------- TOGGLE ----------------

SPIN_TOGGLE_FIRST = True
TOGGLE_SPIN_MS = 1200
TOGGLE_SPIN_PCT = 100       # FIXED: was 60, you asked for stronger
# <<<<< FIX ME 5: nobody has checked which way rolls the Toggle to
#       our colour. Wrong way -> change to FORWARD.
TOGGLE_SPIN_DIR = REVERSE


# ---------------- LIFT ----------------

HOME_LIFT_FIRST = True
SCORE_LIFT_DEG = 350.0       # MEASURED on the robot
# FIXED: the lift never rose because it used spin_to_position at 60
# percent. It now goes up on raw volts, like driver_control.py.
LIFT_VOLTS = 12.0       # UP runs on raw volts; this arm needs all of it
LIFT_DOWN_PCT = 35      # DOWN is gentle; gravity does the work
LIFT_TOL_DEG = 5
LIFT_TIMEOUT_MS = 2500


# ---------------- CLAW ----------------

PRELOAD_PIN = True         # grip closed at boot, on the preloaded pin
PNEUMATIC_SETTLE_MS = 300  # air takes a moment to move the piston


# ---------------- DRIVE TUNING ----------------

KP_DRIVE_PER_IN = 5.0      # percent power per inch still to go
KP_STRAIGHT_PER_IN = 4.0   # percent per inch one side leads the other
KP_TURN_PER_DEG = 1.2      # percent power per degree of heading error
MIN_DRIVE_PCT = 8          # floor, or it parks just short of target
DRIVE_TOL_IN = 0.5         # close enough on distance
TURN_TOL_DEG = 2.0         # close enough on turns
SETTLE_SPEED_IN_S = 1.0    # "stopped" = wheels slower than this
SETTLE_MS = 100            # ...while close, for this long
MOVE_TIMEOUT_FACTOR = 2.5  # times the ideal duration, before giving up

AUTON_TIME_LIMIT_MS = 14500  # hard stop. Never raise past 15000.


# ============================================================
#  FIELD  -  Game Manual v1.0, Appendix A, sheets A10 and A12
# ============================================================

FIELD_INSIDE_IN = 140.41   # inside wall to wall

GOAL_X_IN = 46.66          # our Alliance Goal, on the goal grid
GOAL_Y_IN = 23.11          # ...23.11 in out from our wall
GOAL_BASE_DIAMETER_IN = 5.61

START_X_IN = 70.20         # middle of the wall = middle of the Toggle

# Cup/pin/cup groups sit against the wall either side of the Toggle.
# Starting in contact with one is illegal <SG1>b.
WALL_GAP_LEFT_IN = 51.40
WALL_GAP_RIGHT_IN = 89.01

PI = 3.14159265


# ============================================================
#  ROUTE MATH  -  all derived. Change the settings, not this.
#
#  Local frame: we face INTO the field. "out" is away from our wall,
#  "lat" is to our right. Our goal sits 23.11 out and 23.55 to one
#  side, which side depending on the quadrant.
# ============================================================

_SIDE = -1 if MIRROR_TURNS else 1
_HALF = ROBOT_LENGTH_IN / 2.0

GOAL_OUT_IN = GOAL_Y_IN
GOAL_LAT_IN = -_SIDE * (FIELD_INSIDE_IN / 2.0 - GOAL_X_IN)

_A = math.radians(_SIDE * START_ANGLE_DEG)

# Rearmost corner on the wall, so the centre sits this far out.
START_OUT_IN = _HALF * (abs(math.cos(_A)) + abs(math.sin(_A)))

if DRIVE_OFF_WALL_IN is None:
    if abs(START_ANGLE_DEG) < 1.0:
        # Square: stop level with the goal, so the turn is a clean 90.
        DRIVE_OFF_WALL_IN = GOAL_OUT_IN - TURN_CENTER_FROM_BACK_IN
    else:
        # Tilted: clear the wall to pivot, plus real margin. At 45 deg
        # the max() term is exactly zero, so this margin IS the leg -
        # it was 2.0 once and the robot barely twitched.
        DRIVE_OFF_WALL_IN = max(
            0.0, math.sqrt(2.0) * _HALF - START_OUT_IN) + 8.0

_c_out = START_OUT_IN + DRIVE_OFF_WALL_IN * math.cos(_A)
_c_lat = DRIVE_OFF_WALL_IN * math.sin(_A)

_v_out = GOAL_OUT_IN - _c_out
_v_lat = GOAL_LAT_IN - _c_lat
_range = math.sqrt(_v_out * _v_out + _v_lat * _v_lat)

# Point the BACK down that line; the front ends up 180 from it.
TURN_TO_GOAL_DEG = math.degrees(math.atan2(_v_lat, _v_out) + math.pi - _A)
while TURN_TO_GOAL_DEG > 180.0:
    TURN_TO_GOAL_DEG -= 360.0
while TURN_TO_GOAL_DEG < -180.0:
    TURN_TO_GOAL_DEG += 360.0

# Hand trim, applied in whichever direction the turn already goes.
if TURN_TO_GOAL_DEG < 0:
    TURN_TO_GOAL_DEG -= TURN_TRIM_DEG
else:
    TURN_TO_GOAL_DEG += TURN_TRIM_DEG

# Negative = driven in REVERSE, claw first.
DRIVE_TO_GOAL_IN = -(_range - TURN_CENTER_FROM_BACK_IN
                     - CLAW_REACH_IN + OVERSHOOT_IN)

# Startup checks, shown on the brain.
_TOUCH_RIGHT = (_SIDE * START_ANGLE_DEG) > 0
DISC_AT_WALL = (abs(START_ANGLE_DEG) < 1.0
                or _TOUCH_RIGHT == DISC_ON_RIGHT)
DISC_GAP_IN = 0.0 if DISC_AT_WALL else 2.0 * _HALF * abs(math.sin(_A))
TURN_CLEARS_WALL = _c_out >= math.sqrt(2.0) * _HALF
BUMPER_HITS_GOAL = (CLAW_REACH_IN - OVERSHOOT_IN
                    < GOAL_BASE_DIAMETER_IN / 2.0)
START_SPOT_BAD = (START_X_IN - ROBOT_WIDTH_IN / 2.0 <= WALL_GAP_LEFT_IN
                  or START_X_IN + ROBOT_WIDTH_IN / 2.0 >= WALL_GAP_RIGHT_IN)



# ============================================================
#  STATE
# ============================================================

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
#  R1 = intake  (pulls a game piece in)
#  R2 = outtake (spits it back out)
#
#  Note the motor directions look backwards here. On our build
#  REVERSE is the direction that actually pulls inward, so R1
#  gets REVERSE. The button labels are what matter -- R1 always
#  means "take it in" no matter which way the motor has to turn
#  to do that. If somebody flips the intake gearbox later, swap
#  these two spin directions, not the buttons.
# ============================================================

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

def sign_of(x):
    if x < 0:
        return -1
    return 1


def out_of_time():
    # Every move checks this so nothing runs past the whistle.
    # Off on the bench in STEP_MODE and SELFTEST, where the clock
    # keeps running while you measure and would kill every move.
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
    # P control fades to nothing near the target and parks short.
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

    # Redrawn every refresh. A one-off print gets wiped by the
    # clear_screen above before anyone can read it.
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
    # How much of the autonomous budget is left.
    return AUTON_TIME_LIMIT_MS - brain.timer.time(MSEC)


def pause(ms):
    # Never wait past the whistle. Field control disables the robot at
    # 15 s, so a wait that runs over does nothing except make the DONE
    # time look bad - but it also eats budget the tail steps need.
    budget(ms)


def budget(ms):
    # Wait, but never longer than the time we have left.
    left = time_left_ms()
    if left <= 0:
        return
    wait(min(ms, left), MSEC)


def drive_inches(inches, speed=50, timeout_ms=None):
    # Positive forward, negative backward.
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

        # Done only once close AND stopped. Quitting the moment we
        # are close leaves it rolling and it coasts past the mark.
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

        # One side ahead of the other means we are curving, and every
        # later move would inherit the error.
        correction = ((l - r) / dpi) * KP_STRAIGHT_PER_IN

        left_drive.spin(FORWARD, power - correction, PERCENT)
        right_drive.spin(FORWARD, power + correction, PERCENT)

        wait(20, MSEC)
        elapsed += 20

    left_drive.stop()
    right_drive.stop()


def turn_degrees(angle, speed=40, timeout_ms=None):
    # Positive turns right. One robot degree is a little arc of the
    # turning circle, which is where TRACK_WIDTH comes in.
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

    # What the ENCODERS think we turned, and whether the loop
    # finished or timed out. Stored, not just printed, so status()
    # can keep redrawing it - a one-off print is wiped by the next
    # step's clear_screen before anyone can read it.
    #
    #   says 90, robot turned 80  -> TRACK_WIDTH wrong, trim it
    #   says 60, robot turned 60  -> never finished: power or timeout
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
    # UP runs on raw volts. spin_to_position asks for a velocity and
    # the built-in loop backs off long before a 1:1 DR4B breaks free -
    # driver_control.py found the same thing. DOWN is gentle, because
    # gravity does that work and slamming the stop does not help.
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

    # Hold where each motor actually ended up. Holding both to one
    # shared number makes them fight over gear backlash.
    lift_left.spin_to_position(lift_left.position(DEGREES), DEGREES,
                               100, PERCENT, wait=False)
    lift_right.spin_to_position(lift_right.position(DEGREES), DEGREES,
                                100, PERCENT, wait=False)

    if abs(pos - target_deg) >= LIFT_TOL_DEG:
        # On a row status() does not immediately wipe, or a short arm
        # reads as a drive problem.
        brain.screen.set_cursor(10, 1)
        brain.screen.print("LIFT SHORT {:.0f} of {:.0f}   ".format(
            pos, target_deg))


def spin_toggle():
    # Roll the Toggle to our colour, then stop and drive away - it
    # only counts once nothing is touching it <SC4>.
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

    brain.timer.clear()          # the 15 s clock starts here

    if START_DELAY_MS > 0:
        status("Starting in {:.0f}s".format(START_DELAY_MS / 1000.0))
        wait(START_DELAY_MS, MSEC)

    # Grip the preload, wrist down while driving to keep the load low.
    # Through grab_set/pivot_set so the driver period inherits
    # the right state and the first A press is not inverted.
    if PRELOAD_PIN:
        grab_set(True)
    pivot_set(False)

    if HOME_LIFT_FIRST:
        step("home lift")
        lift_home()
        pause(100)

    # Toggle first, while the disc is still parked at the bar. The
    # yellow pin already in this quadrant's neutral goal becomes ours
    # once it is set <SC5>.
    if SPIN_TOGGLE_FIRST:
        step("spin toggle")
        spin_toggle()
        pause(100)

    step("off wall {:.1f}in".format(DRIVE_OFF_WALL_IN))
    drive_inches(DRIVE_OFF_WALL_IN, 80)
    pause(150)

    # The only turn, and the least repeatable move we make with no
    # inertial sensor, so it is slow.
    step("turn {:.0f}deg".format(TURN_TO_GOAL_DEG))
    turn_degrees(TURN_TO_GOAL_DEG, 50)
    pause(150)

    # Arm up before the approach, so the pin clears the 3.25 in goal
    # instead of hitting its side.
    step("raise to {:.0f}deg".format(SCORE_LIFT_DEG))
    lift_to(SCORE_LIFT_DEG)
    pause(100)

    # Reverse, because the claw is on the back.
    if abs(DRIVE_TO_GOAL_IN) > 0.1:
        step("reverse {:.1f}in".format(abs(DRIVE_TO_GOAL_IN)))
        drive_inches(DRIVE_TO_GOAL_IN, 70)
        pause(150)

    # Wrist over the goal FIRST, then let go. Order matters.
    step("wrist over goal")
    pivot_set(True)
    budget(PNEUMATIC_SETTLE_MS)

    step("release pin")
    grab_set(False)
    budget(PNEUMATIC_SETTLE_MS)
    pause(200)

    # Forward pulls us off the goal, since we reversed into it.
    # Out of time: stop here rather than stepping through moves that
    # cannot move anything anyway.
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
    # Hand the lift back to the driver loop cleanly: it re-commands
    # on the next mode change, and on_stop False stops it sitting in
    # REST when the driver first touches L1.
    global lift_mode, on_stop
    lift_mode = ""
    on_stop = False

    status("DONE {:.1f}s".format(brain.timer.time(MSEC) / 1000.0))


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

TEST_AUTON = False
TEST_DRIVER = False
TEST_BOTH = True

if TEST_BOTH:
    autonomous()
    user_control()

elif TEST_AUTON:
    autonomous()

elif TEST_DRIVER:
    user_control()

else:
    competition = Competition(
        user_control,
        autonomous
    )
