# ============================================================
#  AUTONOMOUS - 1 PIN SCORING ROUTINE
#  VEX V5RC Override (2026-2027)  |  VEXcode V5 Python
#
#  What this does, in plain terms:
#    We start with a pin already clamped in the claw, back against
#    the wall right under our Quadrant's Toggle. We lift, drive
#    straight off the wall, turn 90 degrees toward our Alliance
#    Goal, drive up to it, flick the wrist over the goal, let go,
#    and back off.
#
#  ---- WHERE TO PUT THE ROBOT (red, BOTTOM quadrant) ----
#  Audience view, red Alliance Station on the left. Back of the
#  robot flat against the audience-side wall, centred left-right
#  on the yellow Toggle bar on top of that wall, facing the far
#  wall. Pin clamped in the claw. That is the red robot's spot in
#  the manual's Figure SG-1.
#
#  Why not straight behind the goal, which needs no turn: a cup,
#  a yellow pin and a cup sit against the wall there
#  (x = 43.50 / 46.66 / 49.82, Appendix A sheet A12). <SG1> says
#  we may not start touching any scoring object but our preload,
#  so that spot is illegal, and any Violation in autonomous gives
#  the Autonomous Bonus to the other alliance <SC7>.
#
#  Blue: same spot on the far wall (under the far Toggle), facing
#  the audience. Same turns, see MIRROR_TURNS.
#  Our partner must start in the OTHER quadrant <SG1>.
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
#  <SC7> Autonomous Bonus: 12 points to whoever is ahead when
#  autonomous ends, 6 each on a tie (even 0-0). Robots in the
#  Midfield do NOT count toward it. ANY Violation during
#  autonomous hands it to the other alliance, so a legal start
#  and staying on our side matter more than speed.
#
#  <SC2> A pin only counts once it is nested in a Goal, and a
#  Goal holds ONE pin half directly (a second pin needs a Cup
#  between them). Our Alliance Goal starts empty, so the preload
#  is the one pin we can put in it without handling a cup.
#
#  <SC4>/<SC5> Toggles start with yellow facing into the field.
#  Turning our Quadrant's Toggle to red makes every yellow half
#  placed in that Quadrant ours: the yellow/yellow pin already
#  sitting in our Quadrant's neutral goal at the start (20 pts)
#  plus our preload's yellow half (10). Not done here - the robot
#  needs a mechanism that can turn the Toggle on top of the wall.
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
#  HOW IT STARTS
#    Run pressed on the brain, nothing plugged in: the screens say
#      "press A". Press A on the controller (or tap the brain
#      screen), it waits BENCH_START_DELAY_MS, runs the auton,
#      then the small driver control at the bottom so you can
#      drive it off the field. Nothing moves until you press.
#    Field control or a competition switch plugged in: a normal
#      competition program. The field starts autonomous() and
#      user_control() itself. Plugging the cable in while it is
#      waiting for A is fine - the field takes over.
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

# Bench runs only (Run pressed on the brain, no field control or
# competition switch): pause between pressing A and the robot
# moving, so hands are clear. Never used in a real match.
BENCH_START_DELAY_MS = 1000

# Seat the arm on its bottom stop first. Costs about a second.
HOME_LIFT_FIRST = True


# ============================================================
#  WHICH SIDE OF THE FIELD
# ============================================================

# Mirrors every turn (right becomes left).
#
# Do NOT flip this just because we switched alliance. Red BOTTOM and
# blue TOP are the same spot with the field rotated 180 degrees, and
# a rotation keeps right turns as right turns. Same for red LEFT and
# blue RIGHT. The routine runs unchanged on either alliance.
#
# Flip it only when we start in the OTHER quadrant of our alliance:
# tuned in BOTTOM (red) / TOP (blue) and now starting in LEFT (red) /
# RIGHT (blue), or the reverse. Those two are mirror images across
# the quadrant divider: start under THAT quadrant's Toggle (on the
# alliance-station wall) and the one turn becomes a right turn.
MIRROR_TURNS = False


# ============================================================
#  DRIVETRAIN GEOMETRY
# ============================================================

# Wheel travel and track width are the driver control program's own
# DriveTrain numbers (12.56 in travel = 4 in wheel, 12.5 in track).
WHEEL_CIRCUMFERENCE = 12.56   # 4 inch wheel. A 3.25 inch wheel is 10.21
TRACK_WIDTH = 12.5            # inches, left wheel center to right

# Motor turns per wheel turn. Our drive is geared 2:7 with the small
# gear on the motor (e.g. 24T motor -> 84T wheel): 7 motor turns for
# every 2 wheel turns = 3.5. Top speed is then only about 12 in/s
# (200 rpm / 3.5 * 12.56 in / 60), which the gains below are set for.
DRIVE_GEAR_RATIO = 3.5

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
# Everything below is worked out for red BOTTOM. The other three
# quadrants are the same numbers rotated (blue) or mirrored (the
# other quadrant of our alliance, see MIRROR_TURNS).
FIELD_INSIDE_IN = 140.41

# Our Alliance Goal, sheet A10. Alliance goals are 3.25 in tall,
# the lowest on the field, and ours starts the match EMPTY, so it
# is the one goal the preload can go straight into.
GOAL_X_IN = 46.66
GOAL_Y_IN = 23.11

# Goal base is 5.61 in across the flats (sheet A13). The hole on
# top is only 2.37 in across and the pin's tip is 1.40 in, so the
# pin has to arrive within about half an inch of the goal centre.
GOAL_BASE_DIAMETER_IN = 5.61

# Where our robot's centre starts along the wall: the Toggle's
# centre, which is the middle of the wall (sheet A10). Easy to line
# up by eye, and it is the only clear stretch of that wall: the
# cup-pin-cup groups against it end at x = 51.40 and start again at
# x = 89.01 (sheet A12), so an 18 in robot fits anywhere with its
# centre between 60.4 and 80.0.
START_X_IN = 70.20

# Clear stretch of wall between those two groups, for the check below.
WALL_GAP_LEFT_IN = 51.40
WALL_GAP_RIGHT_IN = 89.01


# ---- OUR ROBOT: measure these, they are the only unknowns ----

# Back bumper to front face, and side to side, at the start of the
# match. <SG1> caps both at 18 inches.
ROBOT_LENGTH_IN = 18.0
ROBOT_WIDTH_IN = 18.0

# Back bumper to the point the robot spins around in a turn: the
# middle of the drive wheels. Half the length for most drivetrains.
TURN_CENTER_FROM_BACK_IN = ROBOT_LENGTH_IN / 2.0

# How far in front of the front bumper the held pin sits once the
# lift is at SCORE_LIFT_DEG and the wrist is up over the goal.
# Zero if the pin is inside our footprint.
#
# MEASURE THIS BEFORE THE FIRST RUN. Below 2.8 in the bumper runs
# into the goal base and shoves the goal, unless the front of the
# robot is open so the goal can slide in under the claw. The brain
# screen warns about it when the program starts.
CLAW_REACH_IN = 0.0

# How far the pin sits to the RIGHT of the robot's centre line,
# looking forward. Negative if it is to the left. 0 = centred.
CLAW_SIDE_OFFSET_IN = 0.0

# Extra forward travel on the last leg, to trim where the pin lands.
# 0 aims at the goal's centre, which is what we want: the hole is so
# small (see above) that aiming past centre lands the pin on the rim.
# Tune in half inches only after measuring where it actually lands.
OVERSHOOT_IN = 0.0


# ---- THE ROUTE ----
#
#   leg 1: straight off the wall, until our turning point is level
#          with the goal centre (minus any sideways claw offset)
#   turn : 90 degrees left, to face the goal
#   leg 2: up to the goal, until the pin is over its centre
#
# Checked against every pin, cup and goal on sheet A12 for an 18 in
# robot: on leg 1 the nearest object is a wall cup almost 10 in to
# our left, the turn's swept circle misses everything by 7+ in, and
# leg 2 is clear floor up to the goal. Nothing on the route is near
# the Autonomous Line <SG7>.
#
# A turn with no inertial sensor is the least repeatable move we
# make, so there is exactly one, at slow speed.

_SIDE = -1 if MIRROR_TURNS else 1   # mirrored quadrant flips "right"

DRIVE_OFF_WALL_IN = (GOAL_Y_IN
                     - TURN_CENTER_FROM_BACK_IN
                     - _SIDE * CLAW_SIDE_OFFSET_IN)

# Positive is right, negative is left.
TURN_TO_GOAL_DEG = -90.0

DRIVE_TO_GOAL_IN = ((START_X_IN - GOAL_X_IN)
                    - (ROBOT_LENGTH_IN - TURN_CENTER_FROM_BACK_IN)
                    - CLAW_REACH_IN
                    + OVERSHOOT_IN)

# Back off after releasing, into open floor. It leaves the driver in
# free space and us off the Field Perimeter <SC8>.
BACK_AWAY_IN = 6.0

# True when leg 2 would push the front bumper into the goal base.
# Checked at startup; see CLAW_REACH_IN.
BUMPER_HITS_GOAL = (CLAW_REACH_IN - OVERSHOOT_IN
                    < GOAL_BASE_DIAMETER_IN / 2.0)

# True when the starting spot would touch the cup-pin-cup groups on
# the wall, which is illegal <SG1>.
START_SPOT_BAD = (START_X_IN - ROBOT_WIDTH_IN / 2.0 <= WALL_GAP_LEFT_IN
                  or START_X_IN + ROBOT_WIDTH_IN / 2.0 >= WALL_GAP_RIGHT_IN)


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

# Set for our slow 3.5 drive gearing (12 in/s flat out): the robot
# holds full speed until about 4 in / 25 degrees out, then eases in.
# With the old numbers it crept so slowly that every move ran out of
# time short of its target.
KP_DRIVE_PER_IN = 20.0     # percent power per inch still to go
KP_STRAIGHT_PER_IN = 10.0  # percent per inch that one side leads
KP_TURN_PER_DEG = 3.0      # percent power per degree of heading error
MIN_DRIVE_PCT = 8          # floor power, or it parks just shy of target

# Green cartridge on the drive motors.
DRIVE_MOTOR_RPM = 200

# Each move gets this many times as long as it would take at full
# speed, plus a second, before giving up.
MOVE_TIMEOUT_FACTOR = 2.5
# The goal's hole leaves the pin about half an inch of slack, and a
# degree off in the turn is a quarter inch at the claw, so these are
# tight on purpose.
DRIVE_TOL_IN = 0.25        # close enough on distance
TURN_TOL_DEG = 1.0         # close enough on turns
SETTLE_SPEED_IN_S = 1.0    # "stopped" = wheels slower than this
SETTLE_MS = 100            # ...while close enough, for this long


# ============================================================
#  STATE
# ============================================================

is_homed = False
step_number = 0
bench_run = False   # True when started by hand from the brain, not by a match


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
    # Not in a bench STEP_MODE run: there the clock keeps running
    # while we wait for A, and after 14.5 s every move would silently
    # do nothing.
    if STEP_MODE and bench_run:
        return False
    return brain.timer.time(MSEC) >= AUTON_TIME_LIMIT_MS


def reset_drive_encoders():
    left_drive.set_position(0, DEGREES)
    right_drive.set_position(0, DEGREES)


def motor_deg_per_inch():
    # Degrees the motor turns to move the robot one inch.
    return (360.0 / WHEEL_CIRCUMFERENCE) * DRIVE_GEAR_RATIO


def top_speed_in_s(pct):
    # How fast the robot rolls at pct percent, in inches per second.
    return DRIVE_MOTOR_RPM * pct / 100.0 / DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE / 60.0


def move_timeout_ms(inches, pct):
    # Time limit for a move, scaled to its length and our real speed,
    # so a slow robot is not cut off halfway and a fast one does not
    # grind for seconds against something it hit.
    return int(1000 + MOVE_TIMEOUT_FACTOR * abs(inches) / top_speed_in_s(pct) * 1000)


def wheel_speed_in_s():
    # How fast the wheels are going, either direction, in inches per
    # second (RPM * 6 = motor degrees per second).
    rpm = (abs(left_drive.velocity(RPM)) + abs(right_drive.velocity(RPM))) / 2.0
    return rpm * 6.0 / motor_deg_per_inch()


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

    # Only ever on the bench. Under field control nobody can press A
    # during autonomous, so a STEP_MODE left on would stall at step 1.
    if not (STEP_MODE and bench_run):
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

    # Wait for it to arrive, but never longer than the timeout. Both
    # motors are averaged so one sluggish side cannot fool it.
    elapsed = 0
    pos = lift_position()
    while elapsed < LIFT_TIMEOUT_MS and not out_of_time():
        pos = lift_position()
        if abs(pos - target_deg) < 5:
            break
        wait(20, MSEC)
        elapsed += 20

    # From here each motor holds ITS OWN reading, like driver_control.py.
    # Holding both at one shared number makes them fight over gear
    # backlash, and if the arm fell short it stops them grinding at
    # full current against whatever is in the way.
    lift_left.spin_to_position(lift_left.position(DEGREES), DEGREES,
                               LIFT_AUTON_PCT, PERCENT, wait=False)
    lift_right.spin_to_position(lift_right.position(DEGREES), DEGREES,
                                LIFT_AUTON_PCT, PERCENT, wait=False)

    if abs(pos - target_deg) >= 5:
        # Say so, or a short arm looks like a drive problem.
        brain.screen.set_cursor(3, 1)
        brain.screen.print("LIFT SHORT: {:.0f} of {:.0f} deg".format(pos, target_deg))


def lift_position():
    # Average of both lift motors, in degrees above the homed zero.
    return (lift_left.position(DEGREES) + lift_right.position(DEGREES)) / 2.0


# ============================================================
#  MOVEMENT
# ============================================================

def drive_inches(inches, speed=50, timeout_ms=None):
    # Positive is forward, negative is backward.
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
        travelled = (l + r) / 2.0
        error_in = (target - travelled) / dpi

        # Done only once we are close AND have stopped moving, for
        # SETTLE_MS. Quitting the moment we get close leaves the robot
        # rolling, and it coasts past the mark.
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


def turn_degrees(angle, speed=40, timeout_ms=None):
    # Positive turns right. One robot degree is a small arc of the
    # turning circle, which is where TRACK_WIDTH comes in.
    if timeout_ms is None:
        timeout_ms = move_timeout_ms(abs(angle) * PI * TRACK_WIDTH / 360.0, speed)
    dpi = motor_deg_per_inch()
    motor_deg_per_robot_deg = (PI * TRACK_WIDTH / 360.0) * dpi
    direction = sign_of(angle)
    target_deg = abs(angle)

    reset_drive_encoders()

    elapsed = 0
    settled = 0
    while elapsed < timeout_ms and not out_of_time():
        l = left_drive.position(DEGREES) * direction
        r = right_drive.position(DEGREES) * direction * -1
        progress_deg = ((l + r) / 2.0) / motor_deg_per_robot_deg
        error_deg = target_deg - progress_deg

        # Done only once we are inside the tolerance AND have stopped
        # rotating, for SETTLE_MS. Quitting the moment we get inside
        # leaves the robot still turning, and it coasts past the angle.
        close = abs(error_deg) < TURN_TOL_DEG
        if close and wheel_speed_in_s() < SETTLE_SPEED_IN_S:
            settled += 20
            if settled >= SETTLE_MS:
                break
        else:
            settled = 0

        # Keeps correcting gently while close instead of letting go.
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


# ============================================================
#  MEASURE MODE
#
#  Finds DRIVE_GEAR_RATIO by experiment instead of counting teeth,
#  and absorbs any error in WHEEL_CIRCUMFERENCE while it does.
# ============================================================

def measure_mode():
    # Also shows the lift angle, for SCORE_LIFT_DEG, and lets you set
    # the claw to its scoring pose to measure CLAW_REACH_IN (setup
    # notes 5 and 6): Y flips the wrist, A opens/closes the grip.
    # Start the program with the arm all the way down: that is zero.
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
        # Buttons are checked every 20 ms so a quick tap is not missed;
        # the screen only redraws every 100 ms.
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
        brain.screen.set_cursor(10, 1)
        brain.screen.print("lift {:>5.0f} deg (SCORE_LIFT_DEG) ".format(lift_position()))
        brain.screen.set_cursor(11, 1)
        brain.screen.print("Y wrist {}   A grip {}   ".format(
            "UP  " if wrist else "DOWN", "CLOSED" if grip else "OPEN  "))


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

    # The 15 second clock starts here, BEFORE any start delay. The
    # delay eats into the real autonomous period, so it has to count
    # against AUTON_TIME_LIMIT_MS too.
    brain.timer.clear()

    if START_DELAY_MS > 0:
        status("Starting in {:.0f}s".format(START_DELAY_MS / 1000.0))
        wait(START_DELAY_MS, MSEC)

    # Re-assert the claw: grip on the preloaded pin, wrist down while
    # driving to keep the load low. Both were already set when the
    # program loaded, so there is no need to wait for air to move.
    if PRELOAD_PIN:
        claw_grab.set(True)
    claw_pivot.set(False)

    if HOME_LIFT_FIRST:
        step("home lift")
        lift_home()
        pause(100)

    # Leg 1: straight off the wall, arm still down so the robot is
    # stable and short while it drives and turns.
    step("off wall {:.1f}in".format(DRIVE_OFF_WALL_IN))
    drive_inches(DRIVE_OFF_WALL_IN, 80)
    pause(150)

    # The only turn. Slow, because it is measured off the wheels.
    step("turn {:.0f}deg".format(mirror(TURN_TO_GOAL_DEG)))
    turn_degrees(mirror(TURN_TO_GOAL_DEG), 50)
    pause(150)

    # Arm up now, before the approach, so the pin clears the 3.25 in
    # goal on the way in instead of hitting its side.
    step("raise to {:.0f}deg".format(SCORE_LIFT_DEG))
    lift_to(SCORE_LIFT_DEG)
    pause(100)

    # Leg 2. The gains ease it in over the last few inches, because
    # overshooting pushes the goal.
    if DRIVE_TO_GOAL_IN > 0:
        step("to goal {:.1f}in".format(DRIVE_TO_GOAL_IN))
        drive_inches(DRIVE_TO_GOAL_IN, 70)
        pause(150)

    # Wrist over the goal first, then let go. This order matters.
    step("wrist over goal")
    wrist_up()

    step("release pin")
    grip_open()
    pause(200)

    step("back off {:.0f}in".format(BACK_AWAY_IN))
    drive_inches(-BACK_AWAY_IN, 80)
    pause(100)

    # Tidy up: wrist in, arm down (lift_to leaves it holding there),
    # wheels stopped.
    step("reset")
    wrist_down()
    lift_to(0)

    left_drive.stop()
    right_drive.stop()

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

    # HOLD, not the default. At 1:1 anything else lets the arm drop
    # the moment L1/L2 is let go. Set here too, not just in auton, in
    # case driver control starts without autonomous having run.
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

        # Just enough claw and lift to unload the robot and park it.
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
            # Stop ONCE on release. Calling stop() every loop re-captures
            # the hold point each time and the arm creeps down.
            lift.stop()
            lift_moving = False

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


def setup_problems():
    # Settings that will break the auton or break a rule. Each is
    # (short name for the controller, full line for the brain).
    problems = []
    if START_SPOT_BAD:
        problems.append(("START_X_IN", "START_X_IN: robot hits wall cups"))
    if BUMPER_HITS_GOAL:
        problems.append(("CLAW_REACH_IN", "CLAW_REACH_IN: bumper hits goal"))
    if STEP_MODE:
        problems.append(("STEP_MODE on", "STEP_MODE on (ignored in a match)"))
    if START_DELAY_MS > 0:
        problems.append(("START_DELAY_MS", "START_DELAY_MS is not 0"))
    return problems


def show_problems():
    # Brain lines 6+ and controller line 3, which status() leaves alone
    # on the controller, so the first problem stays up there.
    problems = setup_problems()
    row = 6
    for _short, full in problems:
        brain.screen.set_cursor(row, 1)
        brain.screen.print("CHECK " + full)
        row += 1
    if problems:
        controller.screen.set_cursor(3, 1)
        controller.screen.print("! {:<16}".format(problems[0][0]))


def screen_pressed():
    try:
        return brain.screen.pressing()
    except Exception:
        return False


def match_control_connected():
    # True when a field controller or competition switch is plugged
    # in, so the match (not us) decides when auton and driver run.
    # If we cannot tell, behave like a normal competition program:
    # that is the safe choice at a real match.
    try:
        return Competition.is_field_control() or Competition.is_competition_switch()
    except Exception:
        # On the bench this shows up as "auton never ran, only the
        # park-it driver control". This line says why.
        brain.screen.set_cursor(4, 1)
        brain.screen.print("Can't read field status: waiting")
        brain.screen.set_cursor(5, 1)
        brain.screen.print("for a match to start auton")
        return True


def wait_for_go():
    # Bench only. Nothing moves until someone presses A (or taps the
    # brain screen), so a program started during match setup, before
    # the field cable goes in, just sits here. Returns False if field
    # control or a switch gets plugged in meanwhile: the field will
    # run autonomous() itself.
    status("BENCH: press A")
    brain.screen.set_cursor(3, 1)
    brain.screen.print("Press A on the controller or tap")
    brain.screen.set_cursor(4, 1)
    brain.screen.print("this screen to run the auton.")
    show_problems()

    while not (controller.buttonA.pressing() or screen_pressed()):
        if match_control_connected():
            return False
        wait(20, MSEC)
    while controller.buttonA.pressing() or screen_pressed():
        wait(20, MSEC)

    status("Hands off: {:.1f}s".format(BENCH_START_DELAY_MS / 1000.0))
    wait(BENCH_START_DELAY_MS, MSEC)
    return True


def driver_period():
    # The brain calls this for the driver period of a match, AND the
    # moment Run is pressed with no field control or switch plugged
    # in (it treats that as driver control). In that bench case run the
    # auton first, like last season's Auton60.py did on Run, then hand
    # over so the robot can be driven off the field.
    global bench_run
    if not match_control_connected():
        bench_run = True
        if wait_for_go():
            autonomous()
    user_control()


show_problems()

# Real match or competition switch: the field runs autonomous() for
# 15 s, then driver_period() for the driver period.
competition = Competition(driver_period, autonomous)


# ============================================================
#  SETUP NOTES  -  do these in order, once
#
#  1. GEAR RATIO. Set MEASURE_MODE = True, download, run. Push the
#     robot straight forward exactly 24 inches. Read DRIVE_GEAR_RATIO
#     off the brain screen, put it in the constant above, set
#     MEASURE_MODE back to False. 3.5 is our 2:7 gearing worked out
#     on paper; the push test should read close to it. If it reads
#     about 0.29 instead, the gearing is the other way round.
#
#  2. TRACK WIDTH. Tape measure, center of the left wheel to center of
#     the right wheel. The 90 degree turn is wrong until this is right.
#
#  3. CHECK A STRAIGHT LINE. STEP_MODE = True, run the first move
#     ("off wall"), and measure how far it actually went. If it asks
#     for 14.1 and gives 12, the gear ratio is still off. Fix that
#     before anything else, because every other number depends on it.
#
#  4. CHECK THE TURN. Still in STEP_MODE, run the turn step and see
#     whether 90 degrees is really 90 (line the robot up with a tile
#     seam before and after). No inertial sensor here, so this is pure
#     encoder math. Turned too little: raise TRACK_WIDTH by 0.5. Too
#     much: lower it by 0.5.
#
#  5. OUR ROBOT'S NUMBERS. ROBOT_LENGTH_IN and ROBOT_WIDTH_IN, and
#     TURN_CENTER_FROM_BACK_IN if the drive wheels are not centred
#     front to back. Then put the arm at SCORE_LIFT_DEG with the wrist
#     up and measure from the front bumper to the centre of the held
#     pin: that is CLAW_REACH_IN. If the pin is not centred side to
#     side, measure that too: CLAW_SIDE_OFFSET_IN. Every leg of the
#     route is worked out from these and the manual's field drawings.
#
#  6. SCORING HEIGHT. MEASURE_MODE shows the lift angle too. Start it
#     with the arm all the way down, raise the arm by hand until the
#     pin clears the 3.25 in goal, read the angle, put it in
#     SCORE_LIFT_DEG. Add a few degrees of margin.
#
#  7. STARTING SPOT. Back flat against the audience-side wall (red) or
#     the far wall (blue), centred on that wall's Toggle, pin clamped.
#     Check nothing on the robot touches the Toggle, which sits on
#     top of the 11.5 in wall: touching it at the start is illegal
#     <SG1>. The brain lists anything in the settings that is wrong.
#
#  8. TIME IT. Run the whole thing and read the DONE time. Anything
#     over about 12 seconds is too tight. Drop BACK_AWAY_IN or set
#     HOME_LIFT_FIRST = False to buy time back.
#
#  9. RUN IT TEN TIMES. Same spot, same pin, ten runs. Nine out of ten
#     is a good auton. Six is a coin flip, and the fix is almost always
#     the turn: re-check steps 2 and 4.
#
#  AT A REAL MATCH: field control runs ONE program for both the 15 s
#  auton and the driver period; you cannot switch programs between
#  them. This file's driver control is only a minimal park-it mode,
#  and driver_control.py's autonomous() is an empty stub. So once the
#  auton works, copy autonomous() and its helpers into
#  driver_control.py, replacing that stub, and run that at matches.
#  Bring the preload clamp with it: this file closes the grip the
#  moment it loads (claw_grab.set(PRELOAD_PIN) above), while
#  driver_control.py boots with the grip OPEN, and once the robot is
#  on the field and disabled nobody can close it before the match.
#  Keep this file for tuning.
# ============================================================
