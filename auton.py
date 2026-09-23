# ============================================================
#  AUTONOMOUS - 1 PIN SCORING ROUTINE
#  VEX V5RC Override (2026-2027)  |  VEXcode V5 Python
#
#  What this does, in plain terms:
#    We start with a pin already clamped in the claw. We drive
#    off the wall, turn toward our goal, drive up to it, lift,
#    flick the wrist over the goal, let go, and back off.
#
#  Why a preloaded pin: picking a pin up off the floor in 15
#  seconds is the most likely thing to fail. Starting with it
#  in our hand removes that whole problem, and a pin that
#  scores every match beats a two pin auton that works half
#  the time.
#
#  ---- RULES THIS ROUTINE IS BUILT AROUND ----
#  All quoted from the Override Game Manual, Version 1.0,
#  released July 2, 2026. Check these against the current
#  manual before a competition - the numbering in the table of
#  contents is off by one from the numbering in the rule body,
#  and the numbers below are the BODY ones.
#
#  <SG5> "Each Robot gets one Pin as a Preload." Red Alliance
#  preloads are red/yellow Pins, blue Alliance preloads are
#  blue/yellow Pins. So our one preloaded pin is legal. One is
#  also the maximum - we cannot start with two.
#
#  <SG1> Starting a Match. We must start:
#    - no bigger than 18 x 18 x 18 inches
#    - touching at most that one preload, no other pins or cups
#    - not touching Goals, Loaders, Load Zones or Toggles
#    - not sharing a Quadrant with our alliance partner
#    - completely stationary, nothing moving
#    - touching the field tiles AND the Field Perimeter, on our
#      own side of the Autonomous Line
#  That last one is why the routine starts by driving off the
#  wall: the rules put us against the wall to begin with.
#
#  <SG7> During autonomous we may NOT contact tiles, scoring
#  objects or field elements on the opponent's side of the
#  Autonomous Line. The Autonomous Line is a pair of white tape
#  lines running DIAGONALLY across the field, around the
#  Midfield. Crossing it hands the Autonomous Bonus straight to
#  the other alliance, so keep the route short and inside our
#  own Quadrant.
#
#  <SG6> Possession is capped at one Pin and one Cup at a time.
#
#  Scoring: each Scored red or blue Pin is 5 points, each
#  Scored yellow Pin is 10 points, each Robot in the Midfield
#  is 8 points. A pin has two halves, and each visible half
#  scores on its own - so our red/yellow preload can score the
#  red half for us plus the yellow half if we own that
#  Quadrant's Toggle.
#
#  <SC8> The Autonomous Win Point needs SEVEN pins scored,
#  three goals holding two pins each, and neither robot
#  touching the Field Perimeter. A one pin auton cannot get it.
#  This routine plays for the Autonomous Bonus instead, which
#  just means outscoring the other alliance in those 15 s.
#
#  PORTS (must match driver_control.py exactly)
#    Drive  : PORT11/12 left, PORT13/14 right
#    Lift   : PORT10 left, PORT9 right
#    Intake : PORT8
#    Claw   : three-wire A = wrist pivot, B = grip
#
#  There is NO inertial sensor on this robot, so every turn is
#  measured off the drive encoders. Turns drift if a wheel
#  slips, so keep them few and keep them slow.
#
#  THREE MODES, set them right below:
#    MEASURE_MODE - find DRIVE_GEAR_RATIO by pushing the robot
#    STEP_MODE    - run one move at a time, press A to continue
#    normal       - run the whole routine on the 15 s clock
#
#  READ THE SETUP NOTES AT THE BOTTOM BEFORE THE FIRST RUN.
# ============================================================

from vex import *

brain = Brain()
controller = Controller()


# ============================================================
#  DEVICES  -  keep identical to driver_control.py
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

intake_conveyor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)

claw_pivot = DigitalOut(brain.three_wire_port.a)   # wrist, up and down
claw_grab = DigitalOut(brain.three_wire_port.b)    # fingers, grip and release


# ============================================================
#  MODES
# ============================================================

# Push the robot by hand and read the true gear ratio off the screen.
MEASURE_MODE = False
MEASURE_DISTANCE_IN = 24.0

# Run one move per button press so you can tape measure each one.
STEP_MODE = False

# Standing still before the routine starts. MUST be 0 at a competition.
START_DELAY_MS = 0

# Seat the arm on its bottom stop first. Costs about a second.
HOME_LIFT_FIRST = True


# ============================================================
#  WHICH SIDE OF THE FIELD
# ============================================================

# Flip this when we switch alliance sides. It mirrors every turn so
# the same routine works from either starting tile.
MIRROR_TURNS = False


# ============================================================
#  DRIVETRAIN GEOMETRY
# ============================================================

WHEEL_CIRCUMFERENCE = 12.56   # 4 inch wheel. A 3.25 inch wheel is 10.21
TRACK_WIDTH = 12.5            # inches, left wheel center to right
DRIVE_GEAR_RATIO = 0.391      # motor turns per wheel turn - VERIFY THIS

PI = 3.14159265


# ============================================================
#  THE ROUTE  -  these are the numbers you will actually tune
# ============================================================

# ---- FIELD FACTS, measured off the official drawings ----
# Game Manual v1.0, Appendix A, sheet A10 "FIELD ELEMENT LOCATIONS".
# All dimensions on that sheet are inches [millimetres].
#
# The field is 140.41 inches inside wall to wall. Every goal sits on
# a grid at 23.11 / 46.66 / 70.20 / 93.75 / 117.30 inches, measured
# from the wall behind the audience-left corner.
#
# The two white diagonals cut the field into four TRIANGULAR
# quadrants: BOTTOM, LEFT, TOP, RIGHT. Each one holds exactly one
# alliance goal and one neutral goal. The tall goal sits dead centre
# at 70.20, 70.20, inside the Midfield.
#
# Red owns the BOTTOM and LEFT quadrants, blue owns TOP and RIGHT.
# The Autonomous Line is the diagonal from the top-left corner to the
# bottom-right corner. In coordinates, we are on the red side while
#     y < 140.41 - x
# Our goals:
#     red goal in the BOTTOM quadrant : x 46.66, y 23.11
#     red goal in the LEFT quadrant   : x 23.11, y 46.66
#
FIELD_INSIDE_IN = 140.41

# Distance from the wall we start against to the centre of the
# alliance goal in that same quadrant. This is the number that
# actually sets our drive distance, and it is short.
GOAL_FROM_WALL_IN = 23.11

# Alliance goals are only 3.25 inches tall, the lowest on the field,
# which is why we aim at ours instead of a neutral goal.


# ---- OUR ROBOT: measure these two, they are the only unknowns ----

# Back bumper to front face at the start of the match. <SG1> caps us
# at 18 inches, so this is 18 or less.
ROBOT_LENGTH_IN = 18.0

# How far in front of the robot the held pin sits once the wrist is
# over the goal. Zero if the pin is inside our footprint.
CLAW_REACH_IN = 0.0

# A little past centre, so the pin drops into the goal rather than
# onto its rim.
OVERSHOOT_IN = 1.5


# ---- THE ROUTE ----

# We start with our back on the wall, lined up with our goal, so the
# whole auton is drive forward, drop, reverse. No turn at all. A turn
# with no inertial sensor is the least repeatable thing we can do, so
# not needing one is worth more than any tuning.
#
# Forward travel = how far the goal centre is from the wall, minus
# the robot we are already occupying, plus a little overshoot.
DRIVE_TO_GOAL_IN = (GOAL_FROM_WALL_IN
                    - ROBOT_LENGTH_IN
                    - CLAW_REACH_IN
                    + OVERSHOOT_IN)

# Only set this if we cannot line up square with the goal on the
# starting tile. Positive is right, negative is left. Leave it at 0.
TURN_TO_GOAL_DEG = 0.0

# Back off after releasing. <SC8> wants us off the perimeter at the
# end of autonomous, and it leaves the driver in free space. Keep it
# smaller than DRIVE_TO_GOAL_IN or we reverse back into the wall.
BACK_AWAY_IN = 6.0


# ============================================================
#  LIFT AND CLAW POSITIONS
# ============================================================

# How high the arm goes to clear the goal rim. MEASURE THIS - the
# 90 below is a placeholder and is probably far too high.
#
# The goals are short. From the manual's glossary: Alliance goals
# are 3.25 inches tall, the neutral goals inside a Quadrant are
# 5.8 inches, and the centre goal in the Midfield is 8.7 inches.
# We only need the pin to clear the rim of whichever one we are
# aiming at, so this number is small.
SCORE_LIFT_DEG = 90.0

# Arm speed during auton. Slow is repeatable.
LIFT_AUTON_PCT = 60

# Give up on a lift move rather than grinding against something.
LIFT_TIMEOUT_MS = 2000

# Air takes a moment to actually move the piston.
PNEUMATIC_SETTLE_MS = 300

# Start clamped, because a pin is loaded in the claw before the match.
PRELOAD_PIN = True


# ============================================================
#  SAFETY AND MOVEMENT CONTROL
# ============================================================

# Hard stop before the whistle. Never raise this past 15000.
AUTON_TIME_LIMIT_MS = 14500

KP_DRIVE_PER_IN = 5.0      # percent power per inch still to go
KP_STRAIGHT_PER_IN = 4.0   # percent per inch that one side leads
KP_TURN_PER_DEG = 1.2      # percent power per degree of heading error
MIN_DRIVE_PCT = 8          # floor power, or it parks just shy of target
DRIVE_TOL_IN = 0.5         # close enough on distance
TURN_TOL_DEG = 2.0         # close enough on turns


# ============================================================
#  STATE
# ============================================================

is_homed = False
step_number = 0


# ============================================================
#  SMALL HELPERS
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
    # Every move checks this, so we never run past the whistle.
    return brain.timer.time(MSEC) >= AUTON_TIME_LIMIT_MS


def reset_drive_encoders():
    left_drive.set_position(0, DEGREES)
    right_drive.set_position(0, DEGREES)


def motor_deg_per_inch():
    # Degrees the motor turns to move the robot one inch.
    return (360.0 / WHEEL_CIRCUMFERENCE) * DRIVE_GEAR_RATIO


def apply_min_power(power):
    # P control fades to nothing near the target and leaves us parked
    # just short of it, so hold a floor under the power.
    if power > 0 and power < MIN_DRIVE_PCT:
        return MIN_DRIVE_PCT
    if power < 0 and power > -MIN_DRIVE_PCT:
        return -MIN_DRIVE_PCT
    return power


def mirror(angle):
    # One routine, both alliance sides.
    if MIRROR_TURNS:
        return -angle
    return angle


def status(text):
    # Shows how far the routine got, on both screens.
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    brain.screen.print(text)
    brain.screen.set_cursor(2, 1)
    brain.screen.print(
        "t = {:.1f} s".format(brain.timer.time(MSEC) / 1000.0)
    )

    controller.screen.set_cursor(1, 1)
    controller.screen.print("{:<18}".format(text))


def step(text):
    # Announces the move. In STEP_MODE it waits for A first, so you can
    # put a tape measure on every single one.
    global step_number

    step_number += 1
    status("{}. {}".format(step_number, text))

    if not STEP_MODE:
        return

    brain.screen.set_cursor(4, 1)
    brain.screen.print("STEP MODE - press A")

    while not controller.buttonA.pressing():
        wait(20, MSEC)
    while controller.buttonA.pressing():
        wait(20, MSEC)


def pause(ms):
    # Let the robot stop coasting before the next move zeroes encoders.
    wait(ms, MSEC)


# ============================================================
#  CLAW
# ============================================================

def wrist_up():
    claw_pivot.set(True)
    wait(PNEUMATIC_SETTLE_MS, MSEC)


def wrist_down():
    claw_pivot.set(False)
    wait(PNEUMATIC_SETTLE_MS, MSEC)


def grip_close():
    claw_grab.set(True)
    wait(PNEUMATIC_SETTLE_MS, MSEC)


def grip_open():
    claw_grab.set(False)
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

        # Ignore the first moment or it finds the bottom instantly.
        if elapsed < 300:
            continue

        if abs(lift.velocity(RPM)) < 5:
            settled += 20
            if settled >= 250:
                break
        else:
            settled = 0

    lift.stop()

    # Zero both encoders, because the hold reads each motor separately.
    lift_left.set_position(0, DEGREES)
    lift_right.set_position(0, DEGREES)

    lift.set_max_torque(100, PERCENT)
    is_homed = True


def lift_to(target_deg):
    # Send the arm to a height and hold it with the motor's own PID.
    lift.set_stopping(HOLD)

    lift_left.spin_to_position(
        target_deg, DEGREES, LIFT_AUTON_PCT, PERCENT, wait=False
    )
    lift_right.spin_to_position(
        target_deg, DEGREES, LIFT_AUTON_PCT, PERCENT, wait=False
    )

    # Wait for it to arrive, but never longer than the timeout.
    elapsed = 0
    while elapsed < LIFT_TIMEOUT_MS and not out_of_time():
        if abs(lift_left.position(DEGREES) - target_deg) < 5:
            break
        wait(20, MSEC)
        elapsed += 20


# ============================================================
#  MOVEMENT
# ============================================================

def drive_inches(inches, speed=50, timeout_ms=3000):
    # Positive is forward, negative is backward.
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

        # If one side has gone further, ease off that side, or we curve
        # and every later move inherits the error.
        drift_in = (l - r) / dpi
        correction = drift_in * KP_STRAIGHT_PER_IN

        left_drive.spin(FORWARD, power - correction, PERCENT)
        right_drive.spin(FORWARD, power + correction, PERCENT)

        wait(20, MSEC)
        elapsed += 20

    left_drive.stop()
    right_drive.stop()


def turn_degrees(angle, speed=40, timeout_ms=2500):
    # Positive turns right. One robot degree is a small arc of the
    # turning circle, which is where TRACK_WIDTH comes in.
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


# ============================================================
#  MEASURE MODE
#
#  Finds DRIVE_GEAR_RATIO by experiment instead of counting teeth,
#  and absorbs any error in WHEEL_CIRCUMFERENCE while it does.
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

        ratio = avg * WHEEL_CIRCUMFERENCE / (MEASURE_DISTANCE_IN * 360.0)

        brain.screen.set_cursor(1, 1)
        brain.screen.print("MEASURE MODE: push robot     ")
        brain.screen.set_cursor(2, 1)
        brain.screen.print(
            "forward {:.0f} in, then read:  ".format(MEASURE_DISTANCE_IN)
        )
        brain.screen.set_cursor(4, 1)
        brain.screen.print(
            "motor deg  L {:>6.0f} R {:>6.0f} ".format(l, r)
        )
        brain.screen.set_cursor(6, 1)
        brain.screen.print("DRIVE_GEAR_RATIO = {:.3f}    ".format(ratio))
        brain.screen.set_cursor(8, 1)
        brain.screen.print("(L and R should be close)    ")

        wait(100, MSEC)


# ============================================================
#  THE AUTONOMOUS ROUTINE
# ============================================================

def autonomous():
    global step_number

    step_number = 0

    left_drive.set_stopping(BRAKE)
    right_drive.set_stopping(BRAKE)

    # Intake locked so nothing shakes loose on the way there.
    intake_conveyor.set_stopping(HOLD)
    intake_conveyor.stop()

    if START_DELAY_MS > 0:
        brain.timer.clear()
        status("Starting in {:.0f}s".format(START_DELAY_MS / 1000.0))
        wait(START_DELAY_MS, MSEC)

    brain.timer.clear()   # the 15 second clock starts here

    # Clamp down on the preloaded pin before we move a wheel.
    if PRELOAD_PIN:
        grip_close()

    # Wrist down while driving keeps our height legal and the load low.
    wrist_down()

    if HOME_LIFT_FIRST:
        step("home lift")
        lift_home()
        pause(100)

    # Arm up before we move. It is only a 3.25 inch goal and only a
    # few inches of driving, so we may as well be at height before
    # the wheels turn rather than doing two things at once.
    step("raise to {:.0f}deg".format(SCORE_LIFT_DEG))
    lift_to(SCORE_LIFT_DEG)
    pause(100)

    # Normally zero. Only runs if we could not line up on the tile.
    if TURN_TO_GOAL_DEG != 0:
        step("turn {:.0f}deg".format(mirror(TURN_TO_GOAL_DEG)))
        turn_degrees(mirror(TURN_TO_GOAL_DEG), 40)
        pause(150)

    # Slow. This is a few inches, and overshooting pushes the goal.
    step("to goal {:.1f}in".format(DRIVE_TO_GOAL_IN))
    drive_inches(DRIVE_TO_GOAL_IN, 30)
    pause(150)

    # Wrist over the goal first, then let go. This order matters.
    step("wrist over goal")
    wrist_up()

    step("release pin")
    grip_open()
    pause(200)

    step("back off {:.0f}in".format(BACK_AWAY_IN))
    drive_inches(-BACK_AWAY_IN, 40)
    pause(100)

    # Tidy up: wrist in, arm down, everything stopped.
    step("reset")
    wrist_down()
    lift_to(0)

    left_drive.stop()
    right_drive.stop()
    lift.stop()

    status("DONE {:.1f}s".format(brain.timer.time(MSEC) / 1000.0))


# ============================================================
#  DRIVER CONTROL
#
#  Deliberately minimal. This file is an auton test program, so
#  this is only here to drive the robot back off the field after
#  a run. The real driver code lives in driver_control.py.
# ============================================================

def user_control():
    left_drive.set_stopping(BRAKE)
    right_drive.set_stopping(BRAKE)

    while True:
        throttle = controller.axis3.position()
        steering = controller.axis1.position()

        if abs(throttle) < 5:
            throttle = 0
        if abs(steering) < 5:
            steering = 0

        left_drive.spin(FORWARD, throttle + steering, PERCENT)
        right_drive.spin(FORWARD, throttle - steering, PERCENT)

        # Just enough claw and lift to unload the robot and park it.
        if controller.buttonA.pressing():
            claw_grab.set(False)
        if controller.buttonY.pressing():
            claw_pivot.set(False)

        if controller.buttonL1.pressing():
            lift.spin(FORWARD, 50, PERCENT)
        elif controller.buttonL2.pressing():
            lift.spin(REVERSE, 30, PERCENT)
        else:
            lift.stop()

        wait(20, MSEC)


# ============================================================
#  STARTUP
# ============================================================

# Measure mode never returns, so no Competition object gets made and
# this stops being a competition program. Set it back to False after.
if MEASURE_MODE:
    measure_mode()

# Put both pistons in a known state the moment the program loads.
claw_pivot.set(False)
claw_grab.set(PRELOAD_PIN)

competition = Competition(user_control, autonomous)


# ============================================================
#  SETUP NOTES  -  do these in order, once
#
#  1. GEAR RATIO. Set MEASURE_MODE = True, download, run. Push the
#     robot straight forward exactly 24 inches. Read DRIVE_GEAR_RATIO
#     off the brain screen, put it in the constant above, set
#     MEASURE_MODE back to False. The 0.391 sitting there now is a
#     guess inherited from the old file and is probably wrong.
#
#  2. TRACK WIDTH. Tape measure, center of the left wheel to center of
#     the right wheel. Turns stay wrong until this is right.
#
#  3. CHECK A STRAIGHT LINE. STEP_MODE = True, run the first move, and
#     measure how far it actually went. If it asks for 14 and gives 12,
#     the gear ratio is still off. Fix that before anything else,
#     because every other number depends on it.
#
#  4. CHECK A TURN. Chalk a line on the floor, run the turn step, see
#     whether 45 degrees is really 45. No inertial sensor here, so this
#     is pure encoder math and wheel slip shows up as turn error.
#
#  5. THE ROUTE. The drive distance is worked out for us now, from
#     the manual's 23.11 inch goal position minus our own length, so
#     the only things to measure are ROBOT_LENGTH_IN and
#     CLAW_REACH_IN. Set the robot back against the wall in the
#     bottom quadrant, lined up with our alliance goal, pin loaded.
#
#  6. SCORING HEIGHT. Raise the arm by hand until the pin clears the
#     goal rim, read the position off the screen, put it in
#     SCORE_LIFT_DEG. Add a few degrees of margin.
#
#  7. TIME IT. Run the whole thing and read the DONE time. Anything
#     over about 12 seconds is too tight. Drop BACK_AWAY_IN or set
#     HOME_LIFT_FIRST = False to buy time back.
#
#  8. RUN IT TEN TIMES. Same tile, same pin, ten runs. Nine out of ten
#     is a good auton. Six is a coin flip, and the fix is almost always
#     a shorter route with fewer turns.
#
#  WHEN IT WORKS: copy autonomous() and its helpers into
#  driver_control.py, replacing the empty autonomous() stub there.
#  Keep this file for tuning.
# ============================================================
