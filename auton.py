# ============================================================
#  AUTON  -  VEX V5RC Override  -  VEXcode V5 Python
#
#  Preloaded pin, scored in our Alliance Goal, plus the Toggle.
#  Our lift, claw and Toggle disc are all on the BACK of the robot,
#  so the back does all the work: at the Toggle first, then at the
#  goal. We never turn around.
#
#  WHERE TO PUT THE ROBOT
#    Blue, RIGHT quadrant. Against the right-hand wall, under that
#    wall's Toggle, rotated 45 degrees so the back-LEFT corner (the
#    one the disc is on) touches the wall. Pin in the claw, intake
#    empty, nothing touching the Toggle.
#
#  HOW TO RUN IT
#    Press Run. Two second countdown, then it goes. No field control
#    needed, no button. That is STANDALONE below.
#
#  WHICH MODE
#    SELFTEST  - test each subsystem one at a time (start here)
#    BRING_UP  - skip lift homing and Toggle, go straight to driving
#    MEASURE   - do not drive; push the robot to read the gear ratio
#    STEP_MODE - one move per press of A, for tape measuring
#    none of them - run the real routine
#
#  EVERYTHING YOU EDIT IS IN THE SETTINGS BLOCK BELOW.
#  Below that is implementation. Reference notes are at the bottom.
# ============================================================

from vex import *
import math

brain = Brain()
controller = Controller()

BUILD = "v11 turn+drop"


# ############################################################
# #                                                          #
# #                      S E T T I N G S                     #
# #                                                          #
# ############################################################

# ============================================================
#  >>>  START HERE  <<<
#
#  Things still WRONG or UNKNOWN, in the order to fix them.
#  Search this file for  "FIX ME"  to jump to each one.
#
#    [ ] 1. SELFTEST = True, run it. See which parts work.
#    [ ] 2. TRACK_WIDTH        - never measured. Then TURN_SCALE
#                                (0.65 now) on field tiles.
#    [ ] 3. SCORE_LIFT_DEG     - guess 120 (90 was too low).
#    [ ] 4. CLAW_REACH_IN      - never measured. Bumper hits goal.
#    [ ] 5. TOGGLE_SPIN_DIR    - unknown which way rolls it right.
#    [ ] 6. DRIVE_GEAR_RATIO   - confirm 3.5 with MEASURE_MODE.
#
#  Already fixed, do not undo:
#    DRIVE_GEAR_RATIO 0.391 -> 3.5   (turn was 10 deg not 65)
#    TURN_SCALE 1.0 -> 0.65          (video: -54 turned 80-90)
#    claw drops on port A, like Y    (pistons were swapped)
#    lift now raises on raw volts    (it never rose before)
#    TOGGLE_SPIN_PCT 60 -> 100       (intake was weak)
# ============================================================

# ---------------- MODES ----------------

SELFTEST = False       # <<<<< FIX ME 1: SET True AND RUN THIS FIRST
#       tests every subsystem one at a time
BRING_UP = False       # skip lift home + Toggle, drive immediately
MEASURE_MODE = False   # do not drive; read the gear ratio by hand
STEP_MODE = False      # one move per press of A
STANDALONE = True      # run on the Run button, no Competition object

MEASURE_DISTANCE_IN = 24.0   # how far you push it in MEASURE_MODE
BENCH_START_DELAY_MS = 2000  # countdown before it moves
START_DELAY_MS = 0           # extra delay inside the routine; 0 at a comp


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
#       Tape measure it and leave it. A turn that is wrong ON THE
#       FIELD is TURN_SCALE below, not this. (If you change this,
#       re-check TURN_SCALE.)
#       (Front to back is a different constant: see
#        TURN_CENTER_FROM_BACK_IN below.)
TRACK_WIDTH = 12.5           # SIDE TO SIDE, left wheel to right wheel

# Turns only - straight drives are not affected. How many times the
# geometry's wheel travel a turn really takes on field tiles.
# FIXED from the field video (IMG_4863, 2026-09-25): the -54 turn
# actually turned 80-90 degrees - too FAR, not too little - so
# 1.0 -> 0.65. (It was briefly 2.0 on a report that it turned half;
# the video disproved that, and 2.0 would spin it ~160.) Tune it ON
# FIELD TILES, not the shop floor:
#     new TURN_SCALE = old TURN_SCALE * asked angle / real angle
#     e.g. asked 54, turned 60 -> 0.65 * 54 / 60 = 0.59
# If MEASURE_MODE later shows DRIVE_GEAR_RATIO is wrong, fix that
# first and put this back to 1.0 - the ratio moves turns too.
TURN_SCALE = 0.65
DRIVE_MOTOR_RPM = 200        # green cartridge

# Motor turns per wheel turn. 3.5 = our 2:7 gearing, small gear on
# the motor (24t -> 84t). Measured, not guessed: a commanded 65 deg
# turn gave about 10 deg of real rotation, and only a true ratio near
# 3.5 does that. Confirm with MEASURE_MODE.
# <<<<< FIX ME 6: confirm with MEASURE_MODE. This was the bug behind
#       your 10 degree turn - it was 0.391, off by 9x.
DRIVE_GEAR_RATIO = 3.5


# ---------------- OUR ROBOT ----------------

ROBOT_LENGTH_IN = 18.0   # back bumper to front face
ROBOT_WIDTH_IN = 18.0    # side to side

# Back bumper to the point it pivots around (middle of the drive
# wheels). Half the length on most drivetrains.
TURN_CENTER_FROM_BACK_IN = ROBOT_LENGTH_IN / 2.0

# How far PAST the back bumper the held pin sits, with the lift up
# at SCORE_LIFT_DEG. 0 = inside our footprint. MEASURE THIS: under
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
# <<<<< FIX ME 3: STILL A GUESS. Was 90, which did not get the pin
#       high enough in auton, so raised to 120. Read the real number
#       off MEASURE_MODE: raise by hand until the pin clears the rim.
#       Asking for more than the arm's travel is safe: it stops
#       pushing at the top stop (LIFT_STALL_* below) and the brain
#       shows "LIFT SHORT <reached> of <asked>" - use that number.
SCORE_LIFT_DEG = 120.0
# FIXED: the lift never rose because it used spin_to_position at 60
# percent. It now goes up on raw volts, like driver_control.py.
LIFT_VOLTS = 12.0       # UP runs on raw volts; this arm needs all of it
LIFT_DOWN_PCT = 35      # DOWN is gentle; gravity does the work
LIFT_TOL_DEG = 5
LIFT_TIMEOUT_MS = 2500
# Going up at full volts and still not moving = the arm is on its top
# stop (or out of torque). More pushing only burns auton time.
LIFT_STALL_RPM = 2      # same as STALL_VEL_RPM in driver_control.py
LIFT_STALL_MS = 300


# ---------------- CLAW ----------------
#
# Works like button Y in driver control: ONE piston, port A, holds
# the pin and lets go of it. On the robot Y is what drops the pin,
# even though driver_control.py calls port A the "wrist".
# The preload starts in the claw, closed. Auton opens it once, after
# the lift is up and we have reversed to the goal. That is all.

PRELOAD_PIN = True         # claw shut on the preloaded pin at boot
# The port A value that HOLDS the pin. Every program boots port A
# off and the claw starts shut on the pin, so off = holding.
# If the pin falls out the moment the program starts, flip to True.
CLAW_HOLD = False
# Port B, button A's piston. Off, exactly as driver_control.py
# boots it - the pose the drivers score from with Y. (It used to
# boot ON in this file, and the claw was not over the goal.)
# Auton never moves it.
PISTON_B_ON = False
PNEUMATIC_SETTLE_MS = 300  # air takes a moment to move the piston


# ---------------- DRIVE TUNING ----------------

# FIXED: was 5.0. At 5 the 8 in leg never got past 40% and timed
# out short, and the route released the pin after the 14.5 s limit.
KP_DRIVE_PER_IN = 10.0     # percent power per inch still to go
KP_STRAIGHT_PER_IN = 4.0   # percent per inch one side leads the other
KP_TURN_PER_DEG = 1.2      # percent power per degree, x TURN_SCALE
MIN_DRIVE_PCT = 8          # floor, or it parks just short of target
DRIVE_TOL_IN = 0.5         # close enough on distance
TURN_TOL_DEG = 2.0         # close enough on turns
SETTLE_SPEED_IN_S = 1.0    # "stopped" = wheels slower than this
SETTLE_MS = 100            # ...while close, for this long
MOVE_TIMEOUT_FACTOR = 2.5  # times the ideal duration, before giving up

AUTON_TIME_LIMIT_MS = 14500  # hard stop. Never raise past 15000.


# ############################################################
# #                 END OF SETTINGS                          #
# #   Below here is implementation. Field numbers come from   #
# #   the Game Manual drawings and should not need editing.   #
# ############################################################


# ============================================================
#  DEVICES  -  ports must match driver_control.py
# ============================================================

left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(left_motor_a, left_motor_b)

right_motor_a = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
right_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(right_motor_a, right_motor_b)

lift_left = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
lift_right = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
lift = MotorGroup(lift_left, lift_right)

# The Toggle disc is chain driven off the SAME motor as the intake,
# so this one motor turns both.
intake_conveyor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)

# driver_control.py calls these claw_pivot (A) and claw_grab (B).
claw = DigitalOut(brain.three_wire_port.a)      # holds / drops pin: Y
piston_b = DigitalOut(brain.three_wire_port.b)  # button A's piston


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

is_homed = False
step_number = 0
bench_run = False
claw_holding = PRELOAD_PIN   # is the claw shut
piston_b_on = PISTON_B_ON


# ============================================================
#  HELPERS
# ============================================================

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
    # Every move checks this so nothing runs past the whistle.
    # Off on the bench in STEP_MODE and SELFTEST, where the clock
    # keeps running while you measure and would kill every move.
    if SELFTEST:
        return False
    if STEP_MODE and bench_run:
        return False
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

    # Redrawn every refresh. A one-off banner gets wiped by the
    # clear_screen above before anyone can read it.
    brain.screen.set_cursor(12, 1)
    brain.screen.print("BUILD " + BUILD + "   ")


def step(text):
    global step_number
    step_number += 1
    status("{}. {}".format(step_number, text))

    # Bench only. Under field control nobody can press A.
    if not (STEP_MODE and bench_run):
        return
    brain.screen.set_cursor(4, 1)
    brain.screen.print("STEP MODE - press A")
    while not controller.buttonA.pressing():
        wait(20, MSEC)
    while controller.buttonA.pressing():
        wait(20, MSEC)


def pause(ms):
    wait(ms, MSEC)


def countdown(ms):
    left = ms
    while left > 0:
        status("Starting in {:.0f}s".format((left + 999) / 1000))
        wait(250, MSEC)
        left -= 250


# ============================================================
#  MOVEMENT
# ============================================================

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
    # turning circle, which is where TRACK_WIDTH comes in, scaled by
    # TURN_SCALE for how the robot really turns on the tiles.
    wheel_in_per_robot_deg = PI * TRACK_WIDTH / 360.0 * TURN_SCALE
    if timeout_ms is None:
        # Scaled too, so the time limit matches the real wheel travel.
        timeout_ms = move_timeout_ms(
            abs(angle) * wheel_in_per_robot_deg, speed)
    dpi = motor_deg_per_inch()
    per_robot_deg = wheel_in_per_robot_deg * dpi
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

        # x TURN_SCALE keeps the push per inch of wheel travel the
        # same whatever TURN_SCALE is, so the turn's timing does not
        # change when it is retuned.
        power = clamp(error_deg * KP_TURN_PER_DEG * TURN_SCALE,
                      -speed, speed)
        if not close:
            power = apply_min_power(power)
        power = power * direction

        left_drive.spin(FORWARD, power, PERCENT)
        right_drive.spin(FORWARD, -power, PERCENT)

        wait(20, MSEC)
        elapsed += 20

    left_drive.stop()
    right_drive.stop()


# ============================================================
#  CLAW
# ============================================================

def claw_set(hold):
    # Port A, the piston button Y fires in driver control.
    global claw_holding
    claw_holding = hold
    claw.set(CLAW_HOLD if hold else not CLAW_HOLD)


def piston_b_set(on):
    global piston_b_on
    piston_b_on = on
    piston_b.set(on)


def drop_pin():
    # One press of Y: let go of the pin.
    claw_set(False)
    wait(PNEUMATIC_SETTLE_MS, MSEC)


def claw_close():
    claw_set(True)
    wait(PNEUMATIC_SETTLE_MS, MSEC)


# ============================================================
#  LIFT
# ============================================================

def lift_home():
    # Run the arm down until it stops moving, and call that zero.
    global is_homed

    lift.set_stopping(BRAKE)
    lift.set_max_torque(30, PERCENT)
    lift.spin(REVERSE, 25, PERCENT)

    elapsed = 0
    settled = 0
    while elapsed < 2000:
        wait(20, MSEC)
        elapsed += 20
        if elapsed < 300:          # or it "finds" the bottom instantly
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
    stalled = 0
    pos = lift_position()
    while elapsed < LIFT_TIMEOUT_MS and not out_of_time():
        pos = lift_position()
        if going_up and pos >= target_deg - LIFT_TOL_DEG:
            break
        if not going_up and pos <= target_deg + LIFT_TOL_DEG:
            break
        # At the top stop. Skip the first 300 ms, or a standing start
        # reads as a stall.
        if (going_up and elapsed >= 300
                and abs(lift.velocity(RPM)) < LIFT_STALL_RPM):
            stalled += 20
            if stalled >= LIFT_STALL_MS:
                break
        else:
            stalled = 0
        wait(20, MSEC)
        elapsed += 20

    # Hold where each motor actually ended up. Holding both to one
    # shared number makes them fight over gear backlash.
    lift_left.spin_to_position(lift_left.position(DEGREES), DEGREES,
                               100, PERCENT, wait=False)
    lift_right.spin_to_position(lift_right.position(DEGREES), DEGREES,
                                100, PERCENT, wait=False)

    # Short only. Coasting PAST the target on 12 V is harmless and
    # used to be reported as "SHORT" too.
    if going_up:
        short = pos < target_deg - LIFT_TOL_DEG
    else:
        short = pos > target_deg + LIFT_TOL_DEG
    if short:
        # On a row status() does not immediately wipe, or a short arm
        # reads as a drive problem.
        brain.screen.set_cursor(10, 1)
        brain.screen.print("LIFT SHORT {:.0f} of {:.0f}   ".format(
            pos, target_deg))


# ============================================================
#  TOGGLE
# ============================================================

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


# ============================================================
#  THE ROUTINE
# ============================================================

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

    # The preload is already in the claw. Keep it shut; leave port B
    # where it booted. Both already set on load, so no air wait.
    if PRELOAD_PIN:
        claw_set(True)
    piston_b_set(PISTON_B_ON)

    if HOME_LIFT_FIRST and not BRING_UP:
        step("home lift")
        lift_home()
        pause(100)

    # Toggle first, while the disc is still parked at the bar. The
    # yellow pin already in this quadrant's neutral goal becomes ours
    # once it is set <SC5>.
    if SPIN_TOGGLE_FIRST and not BRING_UP:
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
# THIS IS CODE FOR LIFT DURING AUTO - TK
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

    # Lift up, backed up to the goal: just drop it. Same as one press
    # of Y. (This used to fire port A, then flip port B - the pistons
    # were labelled the wrong way round, so it let go first and moved
    # the claw after.)
    step("drop pin")
    drop_pin()
    pause(200)

    # Forward pulls us off the goal, since we reversed into it.
    step("clear goal {:.0f}in".format(BACK_AWAY_IN))
    drive_inches(BACK_AWAY_IN, 80)
    pause(100)

    # Claw stays open, ready for the driver's next pin (Y shuts it).
    step("reset")
    lift_to(0)

    left_drive.stop()
    right_drive.stop()
    status("DONE {:.1f}s".format(brain.timer.time(MSEC) / 1000.0))


# ============================================================
#  SELFTEST  -  one subsystem at a time
#
#  Nothing here depends on the field or the route maths. Watch the
#  screen; whatever does not happen is the broken thing.
# ============================================================

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
    pause(700)

    step("turn LEFT 90")
    turn_degrees(-90.0, 40)
    pause(700)

    step("lift UP")
    lift_to(SCORE_LIFT_DEG)
    pause(900)

    step("lift DOWN")
    lift_to(0.0)
    pause(900)

    step("claw DROP (Y)")
    drop_pin()
    pause(900)

    step("claw SHUT (Y)")
    claw_close()
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


# ============================================================
#  MEASURE MODE
#
#  Finds DRIVE_GEAR_RATIO by experiment instead of counting teeth,
#  and absorbs any error in WHEEL_CIRCUMFERENCE while it does.
#  Also shows the lift angle, for SCORE_LIFT_DEG.
#  Y opens/shuts the claw (port A), A flips port B - same buttons
#  as driver control.
# ============================================================

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

    claw_set(PRELOAD_PIN)
    piston_b_set(PISTON_B_ON)
    y_prev = False
    a_prev = False
    loops = 0

    brain.screen.clear_screen()

    while True:
        y = controller.buttonY.pressing()
        a = controller.buttonA.pressing()
        if y and not y_prev:
            claw_set(not claw_holding)
        if a and not a_prev:
            piston_b_set(not piston_b_on)
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
        brain.screen.print("Y claw {}  A port B {}  ".format(
            "SHUT" if claw_holding else "OPEN",
            "ON " if piston_b_on else "OFF"))


# ============================================================
#  DRIVER CONTROL
#
#  Minimal on purpose. This file is a test program; the real driver
#  code is driver_control.py. This is only here so you can drive the
#  robot off the field after a run.
# ============================================================

def user_control():
    left_drive.set_stopping(BRAKE)
    right_drive.set_stopping(BRAKE)
    lift.set_stopping(HOLD)
    lift_moving = False
    y_prev = False
    a_prev = False

    while True:
        throttle = controller.axis3.position()
        steering = controller.axis1.position()
        if abs(throttle) < 5:
            throttle = 0
        if abs(steering) < 5:
            steering = 0

        left_drive.spin(FORWARD, throttle + steering, PERCENT)
        right_drive.spin(FORWARD, throttle - steering, PERCENT)

        # Same as driver control: Y toggles the claw, A toggles port
        # B, once per press.
        y = controller.buttonY.pressing()
        a = controller.buttonA.pressing()
        if y and not y_prev:
            claw_set(not claw_holding)
        if a and not a_prev:
            piston_b_set(not piston_b_on)
        y_prev = y
        a_prev = a

        if controller.buttonL1.pressing():
            lift.spin(FORWARD, 50, PERCENT)
            lift_moving = True
        elif controller.buttonL2.pressing():
            lift.spin(REVERSE, 30, PERCENT)
            lift_moving = True
        elif lift_moving:
            # Stop ONCE on release. Calling stop() every loop
            # re-captures the hold point and the arm creeps down.
            lift.stop()
            lift_moving = False

        wait(20, MSEC)


# ============================================================
#  STARTUP CHECKS
# ============================================================

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


# ============================================================
#  ENTRY POINT  -  one place, one decision
# ============================================================

# Both pistons to a known state the moment the program loads: claw
# shut on the preload, the same way driver_control.py boots.
claw_set(PRELOAD_PIN)
piston_b_set(PISTON_B_ON)

if MEASURE_MODE:
    measure_mode()          # never returns
elif STANDALONE:
    show_problems()
    countdown(BENCH_START_DELAY_MS)
    if SELFTEST:
        selftest()
    else:
        autonomous()
    user_control()
else:
    # Real match: the field runs autonomous() then user_control().
    show_problems()
    competition = Competition(user_control, autonomous)


# ============================================================
#  NOTES
#
#  CALIBRATION, in order:
#   1. MEASURE_MODE = True. Push the robot exactly 24 in. Read
#      DRIVE_GEAR_RATIO off the screen. Should land near 3.5.
#   2. TRACK_WIDTH: tape measure, wheel centre to wheel centre.
#      Then TURN_SCALE, on field tiles: run the route, read the
#      real turn angle, scale by asked / real.
#   3. SELFTEST = True. Watch which subsystems work.
#   4. STEP_MODE = True. Run each move, measure what it actually did.
#   5. CLAW_REACH_IN: lift up, measure back bumper to the centre
#      of the held pin.
#   6. SCORE_LIFT_DEG: raise by hand until the pin clears the goal
#      rim, read the angle in MEASURE_MODE, add a few degrees.
#   7. Run it ten times from the same spot. Nine out of ten is good.
#
#  RULES THIS IS BUILT AROUND (Game Manual v1.0, body numbering):
#   <SG1>  start: 18 in cube, touching the tiles AND the perimeter on
#          our side of the Autonomous Line, not touching Goals,
#          Loaders, Load Zones or Toggles, not sharing a Quadrant
#          with our partner, nothing moving, and touching no scoring
#          object except the one preload.
#   <SG5>  one Pin as a Preload, and only one. Not a cup.
#   <SG6>  possession capped at one Pin and one Cup.
#   <SG7>  do not touch anything on the opponent's side of the
#          Autonomous Line during auton.
#   <SC4>  a Toggle counts as set only when fully seated AND no robot
#          is touching it, so we spin it then drive away.
#   <SC5>  yellow pins in a Quadrant belong to whoever owns that
#          Quadrant's Toggle.
#   <SC7>  Autonomous Bonus is 12 points. ANY violation hands it to
#          the other alliance, so a legal start beats a fast one.
#   <SC8>  the Autonomous Win Point needs seven pins. A one pin auton
#          cannot get it; this plays for the Bonus.
#
#  AT A REAL MATCH: field control runs ONE program for both periods.
#  Set STANDALONE = False, or better, copy autonomous() and its
#  helpers into driver_control.py over its empty stub. Carry the
#  claw start with it (claw_set / piston_b_set at ENTRY POINT) so
#  the preload is held from the moment the program loads - nobody
#  can shut the claw once the robot is on the field.
# ============================================================
