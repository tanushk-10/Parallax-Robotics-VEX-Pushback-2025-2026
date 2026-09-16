# ============================================================
#  COMPETITION MAIN  -  DRIVER CONTROL + AUTONOMOUS
#  VEXcode V5 (Python)
#
#  Merges driver_control.py with the encoder-based autonomous
#  routine from "auto movement.v5python" (auton branch, 2026-09-16).
#  Field control can only trigger autonomous through the
#  Competition object at the bottom of THIS file, so this is the
#  one to download for matches.
#
#  Drivetrain : PORT11/12 left (reversed), PORT13/14 right
#  Lift (DR4B): PORT10 left, PORT9 right (reversed)
#               green cartridge, 1:1, mirrored gear train
#
#  Controls:
#    Left stick vertical  (axis3) - throttle
#    Right stick horiz.   (axis1) - steering
#    L1 - lift up
#    L2 - lift down
#    B + DOWN - re-home the lift
#    B + UP   - lift motor direction diagnostic
# ============================================================

from vex import *

brain = Brain()
controller = Controller()

# drive train motors
left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(left_motor_a, left_motor_b)

right_motor_a = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
right_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(right_motor_a, right_motor_b)

# lift motors
lift_left  = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
lift_right = Motor(Ports.PORT9,  GearSetting.RATIO_18_1, True)
lift = MotorGroup(lift_left, lift_right)


# ============================================================
#  TUNING CONSTANTS
# ============================================================

# ---- Drivetrain ----
DEADBAND  = 5       # ignore joystick noise below this percent
TURN_GAIN = 1.0     # raise toward 1.5 for sharper turning

# ---- Lift: protection ----
# Torque cap limits current so the motors can't sit at stall
# current. At 1:1 a banded DR4B needs everything the motors
# have just to break loose from rest, so this is uncapped and
# the thermal guard plus stall timer do the protecting instead.
# If the motors run hot in normal use, the answer is more
# rubber band, not a lower number here.
MAX_TORQUE_PCT = 100

# ---- Lift: soft limits (motor degrees; at 1:1 = arm degrees)
# MIN must be at or below the homed zero, or the down button is
# dead everywhere below it.
LIFT_MIN_DEG = -5       # tolerance below the homed zero
LIFT_MAX_DEG = 135      # <-- MEASURE AND REPLACE (see bottom)

# ---- Lift: speeds ----
UP_PCT   = 100          # effectively capped by MAX_TORQUE_PCT
DOWN_PCT = 40           # gravity helps; don't slam the bottom

# ---- Lift: position hold ----
# The lift holds whatever height you release it at, by driving
# back to a remembered setpoint. This is what makes it stay
# level with nobody touching it -- a fixed trim percentage
# cannot do that, because the power needed to hold changes
# with arm angle and band tension.
HOLD_KP        = 1.4    # percent power per degree of sag
HOLD_KD        = 6.0    # damping; raise if it oscillates
HOLD_MAX_PCT = 85     # ceiling on hold effort
HOLD_DEADBAND_DEG = 1.5 # don't fight sensor noise
HOLD_THRESHOLD_DEG = 15 # below this, rest on the stop instead

# ---- Lift: smoothing ----
SLEW_PER_LOOP = 12      # max percent change per 20 ms loop

# ---- Lift: thermal guard (Celsius) ----
# DISABLED at driver request while chasing a no-move problem.
# Set back to True once the lift actually lifts. Note the V5
# firmware still derates the motors on its own near 55C, so
# with this off you get no warning before that happens -- the
# lift just quietly goes weak. Temperature is still shown on
# the controller screen; watch it.
THERMAL_GUARD = False
TEMP_CUTOFF_C = 50      # V5 motors self-limit near 55C
TEMP_RESUME_C = 45

# ---- Lift: stall detection ----
# Only armed while the driver is actually holding a button AND
# real power is already applied, so the slew ramp can't be
# mistaken for a stall.
# Left ON. This is the guard most likely to look like "the
# lift won't move" -- it gives up after half a second of no
# motion. To rule it out, set False for ONE brief test only,
# then put it back. With it off, a jammed lift will happily
# cook both motors.
STALL_GUARD   = True
STALL_VEL_RPM = 2
STALL_MS      = 1200
# Must stay below DOWN_PCT, or a jam while lowering is never
# detected -- the command never exceeds the threshold.
STALL_ARM_PCT = 30

# ---- Lift: homing ----
HOMING_PCT        = 25    # gentle downward power while homing
HOMING_TORQUE_PCT = 30    # low, so we touch the stop softly
HOMING_GRACE_MS   = 300   # ignore velocity while it gets moving
HOMING_SETTLE_MS  = 250   # not moving this long = at bottom
HOMING_TIMEOUT_MS = 2500  # give up rather than grind forever

# ---- Controller screen ----
# The controller screen cannot keep up with a 20 ms loop.
# Updating it too often causes visible lag.
SCREEN_UPDATE_MS = 250



# ============================================================
#  AUTONOMOUS TUNING CONSTANTS
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
START_DELAY_MS      = 0       # bench testing: set 2000 so you can
                              # step back. MUST be 0 for competition.
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

# ============================================================
#  STATE
# ============================================================
lift_cmd     = 0.0      # command actually being applied
hold_target  = 0.0      # height the lift is trying to keep (deg)
hold_prev_err = 0.0     # previous hold error, for damping
stall_timer  = 0        # ms spent stalled
thermal_lock = False    # True = lift disabled, too hot
is_homed     = False    # has the lift found its bottom yet
screen_timer = 0        # ms since last screen update


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


# ============================================================
#  AUTONOMOUS HELPERS
# ============================================================

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
#  LIFT HOMING
#
#  Drives the lift down at low power until it stops moving
#  against its own bottom stop, then calls that zero. This
#  means the lift can start at ANY height -- you never have to
#  remember to rest it down before running the program.
# ============================================================
def lift_home():
    global lift_cmd, hold_target, hold_prev_err
    global stall_timer, thermal_lock, is_homed

    lift.set_stopping(BRAKE)
    lift.set_max_torque(HOMING_TORQUE_PCT, PERCENT)

    controller.screen.set_cursor(1, 1)
    controller.screen.print("HOMING LIFT...    ")

    lift.spin(REVERSE, HOMING_PCT, PERCENT)

    elapsed = 0
    settled = 0
    while elapsed < HOMING_TIMEOUT_MS:
        wait(20, MSEC)
        elapsed += 20

        # Give it a moment to start moving before judging it as being 
        # stopped, or it "finds" the bottom instantly by uitself
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


    lift.set_max_torque(MAX_TORQUE_PCT, PERCENT)
    lift_cmd      = 0.0
    hold_target   = 0.0
    hold_prev_err = 0.0
    stall_timer   = 0
    thermal_lock  = False
    is_homed      = True

    controller.screen.set_cursor(1, 1)
    controller.screen.print("LIFT READY        ")


# ============================================================
#  LIFT DIAGNOSTIC  -- hold B + UP
#
#  Spins each lift motor ALONE at low power and reports the
#  direction each one actually turns. On a mirrored gear train
#  both must read the SAME SIGN here -- that is the whole point
#  of the reverse flag on lift_right.
#
#  Opposite signs = the motors are fighting each other. Net
#  torque is near zero and both draw stall current, which feels
#  exactly like "the lift is underpowered." Fix it by flipping
#  ONE of the booleans at the top of this file, not by raising
#  torque.
# ============================================================
def lift_diagnostic():
    lift.stop()
    lift.set_max_torque(40, PERCENT)

    controller.screen.clear_screen()
    controller.screen.set_cursor(1, 1)
    controller.screen.print("DIAG: hands clear ")
    wait(1000, MSEC)

    results = []
    for name, motor in (("L", lift_left), ("R", lift_right)):
        motor.spin(FORWARD, 25, PERCENT)
        wait(400, MSEC)
        v = motor.velocity(RPM)
        motor.stop()
        wait(300, MSEC)
        results.append((name, v))

    lift.set_max_torque(MAX_TORQUE_PCT, PERCENT)

    lv = results[0][1]
    rv = results[1][1]

    controller.screen.clear_screen()
    controller.screen.set_cursor(1, 1)
    controller.screen.print("L{:>4.0f}  R{:>4.0f}   ".format(lv, rv))
    controller.screen.set_cursor(2, 1)
  
    if abs(lv) < 5 or abs(rv) < 5:
        controller.screen.print("motor is dead!     ")
    elif (lv > 0) == (rv > 0):
        controller.screen.print("OK - same dir   ")
    else:
        controller.screen.print("motors are colliding  ")

    wait(4000, MSEC)
    controller.screen.clear_screen()


def ensure_homed():
    # Homes only once per power cycle. Called at the start of
    # BOTH autonomous and driver control, because on a real
    # field the robot is disabled until a period begins --
    # motors commanded before that would go nowhere.
    if not is_homed:
        lift_home()


# ============================================================
#  LIFT CONTROL  -- called every loop
# ============================================================
def hold_power(pos):
    global hold_prev_err

    err = hold_target - pos
    if abs(err) < HOLD_DEADBAND_DEG:
        err = 0.0

    d = err - hold_prev_err
    hold_prev_err = err

    return clamp(HOLD_KP * err + HOLD_KD * d,
                 -HOLD_MAX_PCT, HOLD_MAX_PCT)


def lift_control():
    global lift_cmd, hold_target, hold_prev_err
    global stall_timer, thermal_lock, screen_timer

    # --- manual re-home: hold B + DOWN ---
    # Use if the lift gets out of sync mid-practice (someone
    # moved it by hand, a shaft slipped, etc).
    if controller.buttonB.pressing() and controller.buttonDown.pressing():
        lift_home()
        return

    # --- motor direction diagnostic: hold B + UP ---
    if controller.buttonB.pressing() and controller.buttonUp.pressing():
        lift_diagnostic()
        return

    pos  = lift.position(DEGREES)
    vel  = abs(lift.velocity(RPM))
    temp = max(lift_left.temperature(TemperatureUnits.CELSIUS),
               lift_right.temperature(TemperatureUnits.CELSIUS))

    # --- thermal guard (hysteresis so it doesn't chatter) ---
    if not THERMAL_GUARD:
        thermal_lock = False
    else:
        if temp >= TEMP_CUTOFF_C:
            thermal_lock = True
        if thermal_lock and temp <= TEMP_RESUME_C:
            thermal_lock = False

    up   = controller.buttonL1.pressing()
    down = controller.buttonL2.pressing()
    driving = False

    # --- decide target command ---
    if thermal_lock:
        target = 0                                  # let it cool
        hold_target = pos
    elif up and pos < LIFT_MAX_DEG:
        target = UP_PCT
        hold_target = pos      # setpoint follows the arm...
        driving = True
    elif down and pos > LIFT_MIN_DEG:
        target = -DOWN_PCT
        hold_target = pos      # ...so release captures the height
        driving = True
    elif pos > HOLD_THRESHOLD_DEG or hold_target > HOLD_THRESHOLD_DEG:
        target = hold_power(pos)
    else:
        target = 0             # resting on the bottom stop
        hold_target = pos

    # --- stall protection ---
    # Only counts while the driver is holding a button and real
    # power is already on the motors, so the slew ramp is not
    # mistaken for a stall. Counting the ramp is what made the
    # lift give up before it ever moved.
    if (STALL_GUARD and driving
            and abs(lift_cmd) > STALL_ARM_PCT and vel < STALL_VEL_RPM):
        stall_timer += 20
    else:
        stall_timer = 0

    if stall_timer > STALL_MS:
        # Something is in the way or we are against a hard stop.
        # Quit pushing, but keep the arm where it is.
        hold_target = pos
        target = hold_power(pos)

    # --- slew limiting (no instant current spikes) ---
    if target > lift_cmd:
        lift_cmd = min(target, lift_cmd + SLEW_PER_LOOP)
    elif target < lift_cmd:
        lift_cmd = max(target, lift_cmd - SLEW_PER_LOOP)

    # --- apply ---
    # Stopping mode matters as much as the command here. Above
    # the bottom stop we stop in HOLD, so the motor's own
    # position loop pins the arm between trim corrections
    # instead of letting it creep down. Resting on the stop we
    # use BRAKE, so it is not fighting the frame all match.
    if abs(lift_cmd) < 2.0:
        # Overheated: BRAKE, not HOLD. HOLD keeps the motor
        # energized against gravity, so the arm would never
        # actually cool down -- the guard would defeat itself.
        if pos > HOLD_THRESHOLD_DEG and not thermal_lock:
            lift.set_stopping(HOLD)
        else:
            lift.set_stopping(BRAKE)
        lift.stop()
    else:
        lift.spin(FORWARD, lift_cmd, PERCENT)

    # --- driver feedback (throttled) ---
    screen_timer += 20
    if screen_timer >= SCREEN_UPDATE_MS:
        screen_timer = 0
        controller.screen.set_cursor(1, 1)
        if thermal_lock:
            controller.screen.print("LIFT HOT - COOLING ")
        else:
            controller.screen.print(
                "Lft {:>3.0f}C P{:>4.0f}  ".format(temp, pos))


# ============================================================
#  DRIVE CONTROL  -- called every loop
# ============================================================
def drive_control():
    throttle = controller.axis3.position()
    steering = controller.axis1.position()

    # deadband
    if abs(throttle) < DEADBAND:
        throttle = 0
    if abs(steering) < DEADBAND:
        steering = 0

    left_power  = throttle + (steering * TURN_GAIN)
    right_power = throttle - (steering * TURN_GAIN)

    # Scale both sides down together if either exceeds 100
    biggest = max(abs(left_power), abs(right_power))
    if biggest > 100:
        left_power  = left_power  * 100 / biggest
        right_power = right_power * 100 / biggest

    if left_power == 0 and right_power == 0:
        left_drive.stop()
        right_drive.stop()
    else:
        left_drive.spin(FORWARD, left_power, PERCENT)
        right_drive.spin(FORWARD, right_power, PERCENT)


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
#  DRIVER CONTROL
# ============================================================
def user_control():
    ensure_homed()

    left_drive.set_stopping(BRAKE)
    right_drive.set_stopping(BRAKE)

    while True:
        drive_control()
        lift_control()
        wait(20, MSEC)   # MUST stay inside the loop


# ============================================================
#  AUTONOMOUS  -  THE ROUTINE
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

    if START_DELAY_MS > 0:
        brain.timer.clear()
        status("Starting in {:.0f}s".format(START_DELAY_MS / 1000.0))
        wait(START_DELAY_MS, MSEC)

    brain.timer.clear()          # the 15 s clock starts here

    if HOME_LIFT_FIRST:
        status("Homing lift")
        ensure_homed()           # once per power cycle, same as driver

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
    status("AUTON DONE")


# ============================================================
#  COMPETITION
#  Must be at global scope, at the bottom of the file.
#
#  MEASURE_MODE is a bench diagnostic: it never returns, so the
#  Competition object is never created and the program is NOT a
#  competition program while it is on. Set it back to False.
# ============================================================
if MEASURE_MODE:
    measure_mode()

competition = Competition(user_control, autonomous)


# ============================================================
#  CALIBRATION: finding LIFT_MAX_DEG
#
#  1. Set LIFT_MAX_DEG to 9999 temporarily.
#  2. Run the program and let the lift home.
#  3. Raise the lift BY HAND to its safe top position --
#     stop before anything binds or bottoms out.
#  4. Read the P value on the controller screen.
#  5. Subtract about 10 degrees for margin, put that number
#     into LIFT_MAX_DEG.
#  6. Re-run and confirm the lift stops on its own.
#
#  BANDING (matters more than any of this code at 1:1):
#  With power off, the lift should roughly balance at mid
#  height. If it slams down, add bands. If it flies up, remove
#  some. Anchor bands near the bottom pivot, offset from the
#  pivot point, so tension is highest at the bottom of travel.
#
#  Watch the temperature readout while driving. Past 40C in
#  normal use means the bands are not carrying enough load.
#
#  TUNING THE HOLD:
#  Band the arm first -- the hold loop is trim, not a crane.
#  Then, with the arm at mid height, let go of both buttons:
#    - sags slowly downward  -> raise HOLD_KP by 0.4
#    - bounces or buzzes     -> raise HOLD_KD by 2, or drop
#                               HOLD_KP by 0.4
#    - drifts a degree or two and settles -> correct, leave it
#  If it holds at mid height but sags with a block at full
#  extension, raise HOLD_MAX_PCT before touching HOLD_KP.
# ============================================================


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
#    Set START_DELAY_MS to 0 and MEASURE_MODE to False. Download
#    THIS file; the Competition object above is what lets field
#    control trigger autonomous() and user_control().
# ============================================================
