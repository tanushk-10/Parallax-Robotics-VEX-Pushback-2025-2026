# ============================================================
#  AUTONOMOUS  -  STANDALONE PROGRAM
#  VEXcode V5 (Python)   ||   Download to its own slot
#
#  Matched to the driver control file:
#    Drivetrain : PORT11/12 left (reversed), PORT13/14 right
#    Lift (DR4B): PORT10 left, PORT9 right (reversed)
#
#  Homes the lift first so the arm is seated on its bottom
#  stop, then runs a 15-second movement routine using the
#  motors' built-in encoders (not timers).
#
#  If you change motor ports or directions in driver control,
#  change them here too. They must match.
# ============================================================

from vex import *

brain = Brain()
controller = Controller()

# ---------------- DRIVETRAIN MOTORS ----------------
left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(left_motor_a, left_motor_b)

right_motor_a = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
right_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(right_motor_a, right_motor_b)

# ---------------- LIFT MOTORS ----------------
lift_left  = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
lift_right = Motor(Ports.PORT9,  GearSetting.RATIO_18_1, True)
lift = MotorGroup(lift_left, lift_right)


# ============================================================
#  TUNING CONSTANTS
# ============================================================

# ---- MEASURE MODE ----
# True  = the program does NOT drive. It zeroes the encoders and
#         shows the gear ratio live on the brain screen while
#         you push the robot forward by hand. Push it exactly
#         MEASURE_DISTANCE_IN inches, read GEAR RATIO off the
#         screen, put it in DRIVE_GEAR_RATIO below, then set
#         this back to False.
# False = run the autonomous routine.
MEASURE_MODE        = False
MEASURE_DISTANCE_IN = 24.0

# ---- Drivetrain geometry ----
WHEEL_CIRCUMFERENCE = 12.56   # inches. 4" wheel = 12.56, 3.25" = 10.21
TRACK_WIDTH         = 12.5    # inches, center of left wheel to right
DRIVE_GEAR_RATIO    = 0.391   # motor turns per wheel turn.
                              # 0.286 = 84t on motor -> 24t on wheel
                              # 3.5   = 24t on motor -> 84t on wheel
                              # VERIFY WITH MEASURE MODE. Don't guess.

# ---- Safety ----
START_DELAY_MS      = 2000    # pause before moving so you can step
                              # back. Set to 0 for competition.
AUTON_TIME_LIMIT_MS = 14500   # hard stop; the auton period is 15 s
HOME_LIFT_FIRST     = True    # seat the arm before driving.
                              # Set False to test drive moves alone.

# ---- Movement control ----
# These are in real-world units (inches, robot degrees) so they
# stay correct no matter what DRIVE_GEAR_RATIO turns out to be.
KP_DRIVE_PER_IN    = 5.0   # percent power per inch of distance error
KP_STRAIGHT_PER_IN = 4.0   # percent per inch one side leads the other
KP_TURN_PER_DEG    = 1.2   # percent power per degree of heading error
MIN_DRIVE_PCT      = 8     # floor power, or it stalls just shy of target
DRIVE_TOL_IN       = 0.5   # "close enough" on distance
TURN_TOL_DEG       = 2.0   # "close enough" on turns (robot degrees)
PI = 3.14159265

# ---- Lift homing (copied from driver control) ----
HOMING_PCT        = 25
HOMING_TORQUE_PCT = 30
HOMING_GRACE_MS   = 300
HOMING_SETTLE_MS  = 250
HOMING_TIMEOUT_MS = 2500
STALL_VEL_RPM     = 2


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
    return brain.timer.time(MSEC) >= AUTON_TIME_LIMIT_MS


def reset_drive_encoders():
    left_drive.set_position(0, DEGREES)
    right_drive.set_position(0, DEGREES)


def motor_deg_per_inch():
    # How many degrees the MOTOR turns for one inch of travel.
    return (360.0 / WHEEL_CIRCUMFERENCE) * DRIVE_GEAR_RATIO


def apply_min_power(power):
    # Proportional control fades to zero near the target, which
    # leaves the robot stalled just short of it. This enforces a
    # floor so it always finishes the move.
    if power > 0 and power < MIN_DRIVE_PCT:
        return MIN_DRIVE_PCT
    if power < 0 and power > -MIN_DRIVE_PCT:
        return -MIN_DRIVE_PCT
    return power


def status(text):
    # Shows the current step on BOTH screens so you can see
    # where the routine is if something goes wrong.
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    brain.screen.print(text)
    brain.screen.set_cursor(2, 1)
    brain.screen.print("t = {:.1f} s".format(brain.timer.time(MSEC) / 1000.0))

    controller.screen.set_cursor(1, 1)
    controller.screen.print("{:<18}".format(text))


# ============================================================
#  MEASURE MODE
#
#  Finds DRIVE_GEAR_RATIO by experiment instead of by counting
#  teeth. Motors are set to coast so the robot rolls freely.
#  Push it forward MEASURE_DISTANCE_IN inches and the screen
#  shows the ratio that makes that distance come out right.
#  This also absorbs any error in WHEEL_CIRCUMFERENCE.
# ============================================================
def measure_mode():
    left_drive.set_stopping(COAST)
    right_drive.set_stopping(COAST)
    left_drive.stop()
    right_drive.stop()
    reset_drive_encoders()

    brain.screen.clear_screen()

    while True:
        l = left_drive.position(DEGREES)
        r = right_drive.position(DEGREES)
        avg = (l + r) / 2.0

        # inches = avg / ((360/circ) * ratio)   ->   solve for ratio
        ratio = avg * WHEEL_CIRCUMFERENCE / (MEASURE_DISTANCE_IN * 360.0)

        brain.screen.set_cursor(1, 1)
        brain.screen.print("MEASURE MODE: push robot     ")
        brain.screen.set_cursor(2, 1)
        brain.screen.print("forward {:.0f} in, then read:   ".format(MEASURE_DISTANCE_IN))
        brain.screen.set_cursor(4, 1)
        brain.screen.print("motor deg  L {:>6.0f}  R {:>6.0f}   ".format(l, r))
        brain.screen.set_cursor(6, 1)
        brain.screen.print("DRIVE_GEAR_RATIO = {:.3f}      ".format(ratio))
        brain.screen.set_cursor(8, 1)
        brain.screen.print("(L and R should be close)     ")

        wait(100, MSEC)


# ============================================================
#  LIFT HOMING  (same logic as driver control)
# ============================================================
def lift_home():
    lift.set_stopping(BRAKE)
    lift.set_max_torque(HOMING_TORQUE_PCT, PERCENT)
    lift.spin(REVERSE, HOMING_PCT, PERCENT)

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
    lift.set_position(0, DEGREES)
    lift.set_stopping(BRAKE)     # rest on the stop while we drive


# ============================================================
#  MOVEMENT
# ============================================================

def drive_inches(inches, speed=50, timeout_ms=3000):
    # Positive = forward, negative = backward.
    dpi = motor_deg_per_inch()
    target = inches * dpi
    reset_drive_encoders()

    elapsed = 0
    while elapsed < timeout_ms and not out_of_time():
        l = left_drive.position(DEGREES)
        r = right_drive.position(DEGREES)
        travelled = (l + r) / 2.0
        error_in = (target - travelled) / dpi

        if abs(error_in) < DRIVE_TOL_IN:
            break

        power = clamp(error_in * KP_DRIVE_PER_IN, -speed, speed)
        power = apply_min_power(power)

        # Straightness correction: if one side has gone further
        # than the other, slow that side down. Without this the
        # robot curves and every later move inherits the error.
        drift_in = (l - r) / dpi
        correction = drift_in * KP_STRAIGHT_PER_IN

        left_drive.spin(FORWARD, power - correction, PERCENT)
        right_drive.spin(FORWARD, power + correction, PERCENT)

        wait(20, MSEC)
        elapsed += 20

    left_drive.stop()
    right_drive.stop()


def turn_degrees(angle, speed=40, timeout_ms=2500):
    # Positive = turn right (clockwise), negative = turn left.
    # Each wheel travels along an arc of the turning circle:
    # one robot degree = (PI * TRACK_WIDTH / 360) inches of arc.
    dpi = motor_deg_per_inch()
    motor_deg_per_robot_deg = (PI * TRACK_WIDTH / 360.0) * dpi
    direction = sign_of(angle)
    target_deg = abs(angle)

    reset_drive_encoders()

    elapsed = 0
    while elapsed < timeout_ms and not out_of_time():
        l = left_drive.position(DEGREES) * direction
        r = right_drive.position(DEGREES) * direction * -1
        progress_deg = ((l + r) / 2.0) / motor_deg_per_robot_deg
        error_deg = target_deg - progress_deg

        if abs(error_deg) < TURN_TOL_DEG:
            break

        power = clamp(error_deg * KP_TURN_PER_DEG, -speed, speed)
        power = apply_min_power(power) * direction

        left_drive.spin(FORWARD, power, PERCENT)
        right_drive.spin(FORWARD, -power, PERCENT)

        wait(20, MSEC)
        elapsed += 20

    left_drive.stop()
    right_drive.stop()


def pause(ms):
    # Short settle between moves. Momentum carries the robot past
    # its stopping point, and starting the next move mid-coast
    # throws off the encoder zero.
    wait(ms, MSEC)


# ============================================================
#  THE ROUTINE
#
#  Drives a 24" x 18" rectangle: four straights, four right
#  turns. The robot should end up exactly where it started,
#  facing the same way. Any leftover error is visible at a
#  glance, and four turns in a row make turn error obvious.
#
#  Roughly 10 seconds of movement, leaving margin under 15.
# ============================================================
def autonomous():
    left_drive.set_stopping(BRAKE)
    right_drive.set_stopping(BRAKE)

    brain.timer.clear()
    status("Starting in 2s")
    wait(START_DELAY_MS, MSEC)

    brain.timer.clear()          # the 15 s clock starts here

    if HOME_LIFT_FIRST:
        status("Homing lift")
        lift_home()

    status("1/8 fwd 24")
    drive_inches(24, 50)
    pause(150)

    status("2/8 turn R 90")
    turn_degrees(90, 40)
    pause(150)

    status("3/8 fwd 18")
    drive_inches(18, 50)
    pause(150)

    status("4/8 turn R 90")
    turn_degrees(90, 40)
    pause(150)

    status("5/8 fwd 24")
    drive_inches(24, 50)
    pause(150)

    status("6/8 turn R 90")
    turn_degrees(90, 40)
    pause(150)

    status("7/8 fwd 18")
    drive_inches(18, 50)
    pause(150)

    status("8/8 turn R 90")
    turn_degrees(90, 40)
    pause(150)

    left_drive.stop()
    right_drive.stop()
  #  status("DONE")


# ============================================================
#  ENTRY POINT
# ============================================================
if MEASURE_MODE:
    measure_mode()
else:
    autonomous()


# ============================================================
#  SETUP AND CALIBRATION  (in order)
#
#  STEP 0 - MEASURE THE GEAR RATIO   (MEASURE_MODE = True)
#    Run the program. Nothing moves. Put tape at the robot's
#    front edge, push it forward exactly 24 inches in a straight
#    line, read DRIVE_GEAR_RATIO off the brain screen. Type that
#    number into DRIVE_GEAR_RATIO above. Set MEASURE_MODE = False.
#    If L and R differ by a lot, one side is slipping or one
#    motor isn't counting -- check cables.
#
#  STEP A - DISTANCE
#    Comment out every move in autonomous() except the first
#    drive_inches(24, 50). Mark the floor, run, measure.
#    Should be within an inch. If not, re-do step 0 carefully.
#
#  STEP B - TURNS
#    Comment out everything except one turn_degrees(90, 40).
#    Tape a line along the robot's front edge, run, check
#    against a square.
#      Under-turned -> increase TRACK_WIDTH by 0.5"
#      Over-turned  -> decrease TRACK_WIDTH by 0.5"
#
#  STEP C - FULL RECTANGLE
#    Uncomment everything. The robot should finish where it
#    started, facing the same way.
#
#  FOR COMPETITION
#    Set START_DELAY_MS to 0 and MEASURE_MODE to False. Copy
#    everything from "TUNING CONSTANTS" through the end of
#    autonomous() into your main driver file, replacing its
#    empty autonomous() -- field control can only trigger the
#    routine through the Competition object in that file.
# ============================================================