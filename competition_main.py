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
#  Lift (DR4B): TWO motors, PORT10 left + PORT9 right (reversed)
#               green cartridge, 1:1, mirrored gear train
#  Intake     : PORT7, one motor drives intake + conveyor
#               (last season's driver control, unchanged)
#  Claw       : pneumatic solenoid, three-wire port A
#
#  Controls:
#    Left stick vertical  (axis3) - throttle
#    Right stick horiz.   (axis1) - steering
#    L1 - lift up
#    L2 - lift down
#    R1 - intake + conveyor in
#    R2 - intake + conveyor out
#    A  - claw toggle (pneumatic)
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
# LIFT_CARTRIDGE: green (18:1) is what is on the robot now. Red
# cartridges (GearSetting.RATIO_36_1) give DOUBLE the lifting
# torque for half the speed. If the lift still stalls after this
# code change, that swap is the cheapest real fix: change the
# cartridges, change this one line, nothing else moves.
LIFT_CARTRIDGE = GearSetting.RATIO_18_1

# BOTH lift motors are mapped right here. The numbers must match
# the ports the two lift cables are REALLY plugged into on the
# brain. A motor on the wrong port gives no error at all: that
# motor just never moves, and the other one tries to lift alone.
# The brain screen names any lift motor it cannot find.
LIFT_LEFT_PORT  = 10
LIFT_RIGHT_PORT = 9
lift_left  = Motor(getattr(Ports, "PORT" + str(LIFT_LEFT_PORT)),  LIFT_CARTRIDGE, False)
lift_right = Motor(getattr(Ports, "PORT" + str(LIFT_RIGHT_PORT)), LIFT_CARTRIDGE, True)
lift = MotorGroup(lift_left, lift_right)

# intake + conveyor
# Exactly as in last season's driver control file: one motor on
# PORT7, green cartridge, not reversed.
intake_conveyor = Motor(Ports.PORT7, GearSetting.RATIO_18_1, False)

# claw (pneumatic)
# Solenoid driver cable in three-wire port A on the brain.
claw = DigitalOut(brain.three_wire_port.a)


# ============================================================
#  TUNING CONSTANTS
# ============================================================

# ---- Drivetrain ----
DEADBAND  = 5       # ignore joystick noise below this percent
TURN_GAIN = 1.0     # raise toward 1.5 for sharper turning

# ---- Intake ----
MECH_SPEED = 100        # R1 in / R2 out, same as last season

# ---- Claw (pneumatic) ----
# State of the solenoid when the program starts. Whether "on"
# means open or closed depends on how the cylinder is plumbed;
# if the claw starts the wrong way round, flip this.
CLAW_START_ON = False

# ---- Lift: protection ----
# Full torque. At 1:1 a DR4B needs everything the motors have.
# If the motors run hot in normal use, the answer is more rubber
# band (or red cartridges), not a lower number here.
MAX_TORQUE_PCT = 100

# ---- Lift: upper soft limit (motor degrees; at 1:1 = arm degrees)
# OFF until someone measures it. 135 was a placeholder guess, and
# a wrong guess here stops the lift early, which looks exactly
# like a lift that "won't go all the way up". With it off, the
# top of travel is protected by the stall guard instead.
# Measure it (CALIBRATION, bottom of file), then set True.
USE_MAX_LIMIT = False
LIFT_MAX_DEG  = 135     # only used when USE_MAX_LIMIT is True
# There is NO lower soft limit. The old one trusted the homed
# zero; if homing ever zeroed the lift too high (bands holding
# the arm up, someone's hand in the way), the down button went
# dead below that point. Down is now always allowed and the
# down-stall check stops it pushing into the bottom stop.

# ---- Lift: drive ----
# UP is driven by VOLTAGE, not velocity. 12 V is every bit of
# thrust a V5 motor has, with no speed controller in between
# deciding to give less. DOWN stays on velocity control so the
# motors actively brake the descent instead of letting it fall.
UP_VOLTS = 12.0
DOWN_PCT = 40                 # gravity helps; don't slam the bottom
SLEW_VOLTS_PER_LOOP = 3.0     # 0 -> 12 V in 80 ms, softens the spike

# ---- Lift: position hold ----
# Whenever no button is pressed, each motor servos to the exact
# position it was released at, using the motor's own position
# controller with full current available. The only time the lift
# is NOT held is when the code knows it is sitting on its bottom
# stop (just homed, or it just ran down into the stop).
# The previous version only held above 15 degrees and used BRAKE
# below that and after every stall. At 1:1 BRAKE is close to
# free-fall -- that was the "goes up a little, then slams down".
HOLD_SPEED_PCT = 100          # speed allowed while correcting sag
# True = hold switched off so the arm can be moved by hand while
# measuring LIFT_MAX_DEG. The arm will NOT stay up. Bench only.
CALIBRATING = False

# ---- Lift: thermal guard (Celsius) ----
# DISABLED at driver request while chasing a no-move problem.
# Set back to True once the lift actually lifts. Note the V5
# firmware still derates the motors on its own near 55C, so
# with this off you get no warning before that happens -- the
# lift just quietly goes weak. Temperature is still shown on
# the controller screen; watch it. A lift that has been stalled
# a few times in a row IS hot: let it cool before judging power.
THERMAL_GUARD = False
TEMP_CUTOFF_C = 50      # V5 motors self-limit near 55C
TEMP_RESUME_C = 45

# ---- Lift: stall detection ----
# A stall never drops the arm. Going UP, the lift HOLDS where it
# stalled, the controller shows STALL, and the driver lets go and
# presses again to retry. Each press gets STALL_MS of full push.
# Set STALL_GUARD False to remove that limit entirely -- the lift
# will then push for as long as L1 is held, and a jammed lift will
# cook both motors in well under a minute.
# Going DOWN, the check is always on: it is how the code knows
# the arm has reached the bottom stop.
STALL_GUARD     = True
STALL_VEL_RPM   = 2
STALL_MS        = 1200  # going up
STALL_DOWN_MS   = 300   # going down: that is just the bottom stop
STALL_ARM_VOLTS = 6.0   # don't judge a stall during the ramp

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
lift_volts   = 0.0      # upward voltage actually being applied
lift_mode    = ""       # REST / UP / DOWN / HOLD / COOL, as last commanded
stall_timer  = 0        # ms spent stalled
stall_lock   = False    # stalled: hold until the driver lets go
thermal_lock = False    # True = lift disabled, too hot
is_homed     = False    # has the lift found its bottom yet
screen_timer = 0        # ms since last screen update
on_stop      = False    # True = arm is known to be on its bottom stop
lift_verdict = ""       # last per-motor verdict shown on the brain
lift_press_moved = False  # did the arm move at all during this L1 press
claw_on       = False   # current solenoid state
claw_btn_prev = False   # button A last loop, for press detection


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
    global lift_volts, lift_mode, stall_timer, stall_lock
    global thermal_lock, is_homed, on_stop

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

        # Give it a moment to start moving before judging it as
        # stopped, or it "finds" the bottom instantly by itself.
        if elapsed < HOMING_GRACE_MS:
            continue

        if abs(lift.velocity(RPM)) < STALL_VEL_RPM:
            settled += 20
            if settled >= HOMING_SETTLE_MS:
                break
        else:
            settled = 0

    lift.stop()
    # Zero BOTH motors, not just the group's first one: the hold
    # uses each motor's own encoder.
    lift_left.set_position(0, DEGREES)
    lift_right.set_position(0, DEGREES)

    lift.set_max_torque(MAX_TORQUE_PCT, PERCENT)
    lift_volts   = 0.0
    lift_mode    = ""       # force the next loop to re-command
    stall_timer  = 0
    stall_lock   = False
    thermal_lock = False
    is_homed     = True
    on_stop      = True     # homing ends on the stop

    controller.screen.set_cursor(1, 1)
    controller.screen.print("LIFT READY        ")


# ============================================================
#  LIFT DIAGNOSTIC  -- hold B + UP   (start with the lift DOWN)
#
#  Answers one question: are the two lift motors helping each
#  other or fighting? Fighting motors feel EXACTLY like "not
#  enough thrust": net torque near zero, both at stall current.
#
#  How: power ONE motor while the other coasts, and read the
#  speed of the one that is NOT powered. It is being dragged
#  along through the gear train, so its reading shows which way
#  the mechanism really turned.
#    passive reads the same sign as the driven one -> they agree
#    passive reads the opposite sign                -> FIGHTING
#  (The old test read each motor's own speed while driving it.
#  A motor always calls its own forward "positive", so that test
#  said OK no matter what.)
#
#  Verdicts:
#    OK same dir    - wiring/reverse flags are right. The problem
#                     is torque: bands, gearing, cartridges.
#    FIGHTING       - flip ONE lift reverse flag at the top.
#    NO MOVE        - one motor alone can't budge the lift, so the
#                     test can't tell. Take the load off (lift the
#                     arm by hand a little) and run it again.
#    NOT LINKED     - one side moved, the other read nothing:
#                     loose shaft, stripped gear, bad cable.
# ============================================================
def lift_diagnostic():
    global lift_volts, lift_mode, stall_timer, stall_lock

    lift.stop()
    lift.set_max_torque(100, PERCENT)

    controller.screen.clear_screen()
    controller.screen.set_cursor(1, 1)
    controller.screen.print("DIAG: hands clear ")
    wait(1000, MSEC)

    results = []    # (driven rpm, passive rpm) for L driven, then R driven
    for driven, passive in ((lift_left, lift_right), (lift_right, lift_left)):
        passive.set_stopping(COAST)
        passive.stop()
        driven.spin(FORWARD, 8, VOLT)
        wait(400, MSEC)
        dv = driven.velocity(RPM)
        pv = passive.velocity(RPM)
        driven.set_stopping(BRAKE)
        driven.stop()
        passive.set_stopping(BRAKE)
        passive.stop()
        wait(600, MSEC)
        results.append((dv, pv))

    MOVED = 3   # rpm; below this counts as "did not move"
    fighting   = False
    not_linked = False
    no_move    = False
    for dv, pv in results:
        if abs(dv) < MOVED:
            no_move = True
        elif abs(pv) < MOVED:
            not_linked = True
        elif (dv > 0) != (pv > 0):
            fighting = True

    controller.screen.clear_screen()
    controller.screen.set_cursor(1, 1)
    controller.screen.print("L>{:+4.0f} R{:+4.0f}".format(results[0][0], results[0][1]))
    controller.screen.set_cursor(2, 1)
    controller.screen.print("R>{:+4.0f} L{:+4.0f}".format(results[1][0], results[1][1]))
    controller.screen.set_cursor(3, 1)
    if fighting:
        controller.screen.print("FIGHTING flip one ")
    elif not_linked:
        controller.screen.print("NOT LINKED        ")
    elif no_move:
        controller.screen.print("NO MOVE too heavy ")
    else:
        controller.screen.print("OK same dir       ")

    wait(5000, MSEC)
    controller.screen.clear_screen()

    lift.set_max_torque(MAX_TORQUE_PCT, PERCENT)
    lift_volts  = 0.0
    lift_mode   = ""        # force the next loop to re-command
    stall_timer = 0
    stall_lock  = False


def ensure_homed():
    # Homes only once per power cycle. Called at the start of
    # BOTH autonomous and driver control, because on a real
    # field the robot is disabled until a period begins --
    # motors commanded before that would go nowhere.
    if not is_homed:
        lift_home()


# ============================================================
#  LIFT MOTOR CHECK + LIVE READOUT  (brain screen)
#
#  Answers "is the second motor actually doing anything?".
#  The brain screen shows, for EACH lift motor: whether the brain
#  can see it, its speed, the current it is drawing and its
#  temperature. Hold L1 and read the bottom line:
#    BOTH PULLING          - both motors are working. If the lift
#                            still won't rise it is torque: bands,
#                            red cartridges, gearing.
#    ONLY LEFT/RIGHT PULLS - the other motor is idle. Bad cable,
#                            dead port, dead motor, or its shaft
#                            is not gripping the gear.
#    NOT FOUND             - the brain cannot see that motor on
#                            the port number set at the top.
#    BOTH PULL, THEN STALLED - it rose, then stopped with both at
#                            full current: top of travel, or the
#                            motors ran out of torque part way up.
#    BOTH MAXED, NO MOVE   - both at full current and it never
#                            moved: fighting each other (run B+UP)
#                            or the load is simply too heavy.
# ============================================================
def lift_missing():
    names = []
    if not lift_left.installed():
        names.append("LEFT p" + str(LIFT_LEFT_PORT))
    if not lift_right.installed():
        names.append("RIGHT p" + str(LIFT_RIGHT_PORT))
    return names


def lift_check():
    # Run once when driver control starts. Does not block the
    # program: a robot with one dead lift motor can still drive.
    missing = lift_missing()
    if not missing:
        return
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    brain.screen.print("LIFT MOTOR NOT FOUND:")
    row = 2
    for name in missing:
        brain.screen.set_cursor(row, 1)
        brain.screen.print("  " + name)
        row += 1
    brain.screen.set_cursor(row + 1, 1)
    brain.screen.print("Check the cable and the port")
    brain.screen.set_cursor(row + 2, 1)
    brain.screen.print("numbers at the top of the code.")
    controller.screen.set_cursor(3, 1)
    controller.screen.print("NO " + missing[0] + "     ")
    controller.rumble("- - -")
    wait(1500, MSEC)


def lift_telemetry(mode, pos):
    global lift_verdict, lift_press_moved

    rows = []
    amps = []
    for name, port, m in (("L", LIFT_LEFT_PORT, lift_left),
                          ("R", LIFT_RIGHT_PORT, lift_right)):
        if m.installed():
            a = m.current(CurrentUnits.AMP)
            amps.append(a)
            rows.append("{} p{:<2} {:>4.0f}rpm {:>4.1f}A {:>3.0f}C   ".format(
                name, port, m.velocity(RPM), a,
                m.temperature(TemperatureUnits.CELSIUS)))
        else:
            amps.append(-1.0)
            rows.append("{} p{:<2} NOT FOUND              ".format(name, port))

    # Only judge while really pushing up; keep the last verdict on
    # screen after L1 is released so it can be read.
    if mode != "UP" and mode != "STALL":
        lift_press_moved = False           # new press starts fresh
    moving = abs(lift.velocity(RPM)) >= STALL_VEL_RPM
    if mode == "UP" and moving:
        lift_press_moved = True

    if mode == "UP" and lift_volts >= STALL_ARM_VOLTS:
        la, ra = amps[0], amps[1]
        if la < 0 or ra < 0:
            lift_verdict = "MOTOR NOT FOUND - see above"
        elif max(la, ra) > 0.6 and min(la, ra) < 0.25 * max(la, ra):
            lift_verdict = "ONLY " + ("LEFT" if la > ra else "RIGHT") + " PULLS - other idle"
        elif min(la, ra) > 1.8 and not moving:
            if lift_press_moved:
                # it did rise, then ran out: a hard stop or not enough torque
                lift_verdict = "BOTH PULL, THEN STALLED"
            else:
                # never moved at all with both maxed: fighting, or far too heavy
                lift_verdict = "BOTH MAXED, NO MOVE (B+UP)"
        else:
            lift_verdict = "BOTH PULLING"

    brain.screen.set_cursor(1, 1)
    brain.screen.print("LIFT {:<6} pos {:>5.0f} deg      ".format(mode, pos))
    brain.screen.set_cursor(2, 1)
    brain.screen.print(rows[0])
    brain.screen.set_cursor(3, 1)
    brain.screen.print(rows[1])
    brain.screen.set_cursor(4, 1)
    brain.screen.print("> {:<30}".format(lift_verdict if lift_verdict else "hold L1 to test"))


# ============================================================
#  LIFT CONTROL  -- called every loop
# ============================================================
def lift_hold_here():
    # Each motor holds ITS OWN encoder reading. Handing both the
    # same number makes them fight over a degree of gear backlash
    # for the whole match.
    lift.set_stopping(HOLD)
    lift_left.spin_to_position(lift_left.position(DEGREES), DEGREES,
                               HOLD_SPEED_PCT, PERCENT, wait=False)
    lift_right.spin_to_position(lift_right.position(DEGREES), DEGREES,
                                HOLD_SPEED_PCT, PERCENT, wait=False)


def lift_control():
    global lift_volts, lift_mode, stall_timer, stall_lock
    global thermal_lock, screen_timer, on_stop

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

    # Letting go of both buttons is what clears a stall.
    if not up and not down:
        stall_lock = False

    # --- decide what the lift should be doing ---
    # The ONLY things that can refuse an UP press are listed right
    # here: thermal lock (off by default), a stall that has not
    # been released yet, and the upper soft limit (off by default).
    blocked = thermal_lock or stall_lock
    at_top  = USE_MAX_LIMIT and pos >= LIFT_MAX_DEG
    if up and not blocked and not at_top:
        mode = "UP"
    elif down and not up and not blocked:
        mode = "DOWN"
    elif thermal_lock:
        mode = "COOL"          # de-energize so it can actually cool
    elif CALIBRATING or on_stop:
        mode = "REST"          # on the bottom stop (or bench mode)
    else:
        mode = "HOLD"          # anywhere else: never let it fall

    # --- apply ---
    if mode == "UP":
        # Re-sent every loop because the voltage ramps.
        on_stop = False
        lift_volts = min(UP_VOLTS, lift_volts + SLEW_VOLTS_PER_LOOP)
        # Each motor is commanded by name, not through the group,
        # so there is no doubt that BOTH get the full voltage.
        lift_left.spin(FORWARD, lift_volts, VOLT)
        lift_right.spin(FORWARD, lift_volts, VOLT)
    else:
        lift_volts = 0.0
        if mode != lift_mode:
            # Commanded ONCE on the change, not every loop. A hold
            # that is re-issued every 20 ms re-captures the position
            # each time and the arm creeps down.
            if mode == "DOWN":
                lift.spin(REVERSE, DOWN_PCT, PERCENT)
            elif mode == "HOLD":
                lift_hold_here()
            else:
                lift.set_stopping(BRAKE)
                lift.stop()
    lift_mode = mode

    # --- stall protection ---
    # UP: armed only once real power is on, so the ramp is not
    # mistaken for a stall, and only if STALL_GUARD is on.
    # DOWN: always armed. Not moving while driving down means the
    # arm is on the bottom stop, so stop pushing and rest there.
    if mode == "UP":
        watching = STALL_GUARD and lift_volts >= STALL_ARM_VOLTS
        stall_limit = STALL_MS
    else:
        watching = (mode == "DOWN")
        stall_limit = STALL_DOWN_MS

    if watching and vel < STALL_VEL_RPM:
        stall_timer += 20
    else:
        stall_timer = 0

    if stall_timer > stall_limit:
        stall_lock  = True
        stall_timer = 0
        if mode == "DOWN":
            on_stop = True

    # --- driver feedback (throttled) ---
    screen_timer += 20
    if screen_timer >= SCREEN_UPDATE_MS:
        screen_timer = 0
        if thermal_lock:
            label = "HOT"
        elif stall_lock and not on_stop:
            label = "STALL"
        elif up and at_top:
            label = "MAX"
        else:
            label = mode
        controller.screen.set_cursor(1, 1)
        controller.screen.print(
            "L{:>3.0f}C {:>4.0f} {:<6}".format(temp, pos, label))
        lift_telemetry(label, pos)


# ============================================================
#  INTAKE CONTROL  -- called every loop
#
#  Last season's intake, unchanged: R1 runs intake + conveyor in,
#  R2 runs them out, release stops (HOLD, set in user_control).
#
#  Override note: rule <SG6> allows only ONE cup and ONE pin on
#  the robot. This intake does not stop itself, so not pulling in
#  a second cup is on the driver.
# ============================================================
def intake_control():
    if controller.buttonR1.pressing():
        intake_conveyor.spin(FORWARD, MECH_SPEED, PERCENT)
    elif controller.buttonR2.pressing():
        intake_conveyor.spin(REVERSE, MECH_SPEED, PERCENT)
    else:
        intake_conveyor.stop()


# ============================================================
#  CLAW CONTROL  -- called every loop
#
#  Pneumatic claw on three-wire port A. Button A toggles it: one
#  press flips the solenoid, and it stays there until the next
#  press. Acts on the press itself, so holding A does not make it
#  chatter.
# ============================================================
def claw_set(on):
    global claw_on
    claw_on = on
    claw.set(on)
    controller.screen.set_cursor(2, 1)
    controller.screen.print("CLAW ON " if on else "CLAW OFF")


def claw_control():
    global claw_btn_prev
    pressed = controller.buttonA.pressing()
    if pressed and not claw_btn_prev:
        claw_set(not claw_on)
    claw_btn_prev = pressed


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
    lift_check()          # names a lift motor the brain cannot see
    ensure_homed()

    left_drive.set_stopping(BRAKE)
    right_drive.set_stopping(BRAKE)
    intake_conveyor.set_stopping(HOLD)     # as last season
    intake_conveyor.stop()
    claw_set(claw_on)                      # re-assert + show state

    while True:
        drive_control()
        lift_control()
        intake_control()
        claw_control()
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

    # Keep the intake still so the preload stays put. It is
    # never driven in this routine.
    intake_conveyor.set_stopping(HOLD)
    intake_conveyor.stop()

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

claw_on = CLAW_START_ON
claw.set(claw_on)

competition = Competition(user_control, autonomous)


# ============================================================
#  CALIBRATION: finding LIFT_MAX_DEG   (optional)
#
#  The lift works without this. Do it when you want the lift to
#  stop by itself at the top instead of leaning on the stall guard.
#  1. Set CALIBRATING = True. That turns the hold off so the arm
#     can be moved by hand. It will NOT stay up on its own.
#  2. Run the program and let the lift home.
#  3. Raise the lift BY HAND to its safe top position --
#     stop before anything binds or bottoms out.
#  4. Read the middle number on the controller's top line
#     (temperature, POSITION, state).
#  5. Subtract about 10 degrees for margin and put that number
#     into LIFT_MAX_DEG. Set USE_MAX_LIMIT = True and
#     CALIBRATING = False.
#  6. Re-run and confirm the lift stops on its own, showing MAX.
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
#  IF THE LIFT STALLS OR WON'T GO ALL THE WAY UP
#  Work down this list in order. Read the word at the end of the
#  controller's top line while holding L1:
#    MAX   - not a stall. Only appears with USE_MAX_LIMIT on:
#            LIFT_MAX_DEG is too small; re-measure it.
#    STALL - the motors ran out of torque. Let go, then:
#  0. Read the BRAIN screen while holding L1. It shows both lift
#     motors separately and says if only one is pulling or if one
#     is not found on its port. Fix that first.
#  1. Hold B + UP with the lift down and read the verdict. If it
#     says FIGHTING, flip one lift reverse flag. Nothing else will
#     help until that is fixed.
#  2. Check the temperature on the same line. Past ~50C the V5
#     firmware cuts motor power on its own. A lift that has been
#     stalled a few times is hot. Let it cool 10 minutes.
#  3. Bands. With the program stopped, the arm should roughly
#     balance at mid height. If it drops, add bands.
#  4. Still stalls with good bands: two green motors at 1:1 are
#     simply not enough for this lift. Swap to red cartridges and
#     set LIFT_CARTRIDGE = GearSetting.RATIO_36_1 (double torque),
#     or gear the lift down (12T driving 36T or 60T). No code
#     setting can add torque the motors do not have -- UP already
#     sends the full 12 V.
#
#  THE HOLD
#  There is nothing to tune. The motors hold the release position
#  themselves. If the arm sags while holding, that is torque too:
#  same list, from step 2.
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
