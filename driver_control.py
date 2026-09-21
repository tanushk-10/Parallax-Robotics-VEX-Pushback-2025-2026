# ============================================================
#  FINAL DRIVER CONTROL - MAIN FILE
#  VEXcode V5 (Python)
#
#  Drivetrain : PORT11/12 left, PORT13/14 right
#  Lift (DR4B): PORT10 left, PORT9 right
#               green cartridge, 1:1, mirrored gear train
#
#  Controls:
#    Left stick vertical  (axis3) - throttle
#    Right stick horiz.   (axis1) - steering
#    L1 - lift up
#    L2 - lift down
#    B + DOWN - re-home the lift
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

    if abs(throttle) < DEADBAND:
        throttle = 0
    if abs(steering) < DEADBAND:
        steering = 0

    if steering != 0:
        left_power = steering
        right_power = -steering
    else:
        left_power = throttle
        right_power = throttle

    if left_power == 0 and right_power == 0:
        left_drive.stop()
        right_drive.stop()
    else:
        left_drive.spin(FORWARD, left_power, PERCENT)
        right_drive.spin(FORWARD, right_power, PERCENT)

# ============================================================
#  DRIVER CONTROL
# ============================================================
def user_control():
    ensure_homed()

    left_drive.set_stopping(BRAKE)
    right_drive.set_stopping(BRAKE)

    left_drive.set_max_torque(100, PERCENT)
    right_drive.set_max_torque(100, PERCENT)

    while True:
        drive_control()
        lift_control()
        wait(20, MSEC)   # MUST stay inside the loop


# ============================================================
#  AUTONOMOUS
# ============================================================
def autonomous():
    ensure_homed()
    # ... your auton routine here ...
    pass


# ============================================================
#  COMPETITION
#  Must be at global scope, at the bottom of the file.
# ============================================================
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
