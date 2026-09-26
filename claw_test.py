# ============================================================
#  CLAW TEST - BENCH PROGRAM
#  VEXcode V5 (Python)
#
#  For testing the two claw pneumatics on their own. You do NOT
#  need the drivetrain, the lift, or even the rest of the robot.
#  All this needs is:
#
#    - the V5 brain, powered on
#    - a charged air tank, open to the solenoids
#    - solenoid cable for the WRIST in three-wire port A
#    - solenoid cable for the FINGERS in three-wire port B
#    - a controller, paired
#
#  Our claw has two separate pistons doing two separate jobs:
#
#    Port A -> WRIST   : pivots the whole claw up and down
#    Port B -> FINGERS : opens and closes on the pin
#
#  They are independent on purpose. We need to pivot while
#  already holding a pin, and let go without the wrist moving.
#
#  Controls:
#    A     - fingers open / close
#    Y     - wrist up / down
#    X     - run an automatic cycle test (checks for air leaks)
#    B     - stop the cycle test early
#    DOWN  - reset both back to their startup state
#
#  NOTE: this is NOT a competition program. There is no
#  Competition object at the bottom, so it starts running the
#  moment you hit the play button. Do not bring this file to a
#  match -- it is a bench tool only.
# ============================================================

from vex import *

brain = Brain()
controller = Controller()


# ============================================================
#  PNEUMATICS
# ============================================================

# Same ports as driver_control.py. If you change one, change it
# in both files or the claw will behave differently in testing
# than it does on the field.
claw_pivot = DigitalOut(brain.three_wire_port.a)
claw_grab  = DigitalOut(brain.three_wire_port.b)


# ============================================================
#  SETTINGS
# ============================================================

# Where both pistons sit when the program boots. These should
# match driver_control.py exactly, so what you feel on the
# bench is what you get in a match.
PIVOT_START_UP    = False
GRAB_START_CLOSED = False

# Automatic cycle test. Fires both pistons back and forth this
# many times so you can listen for leaks and watch whether the
# claw still closes fully as tank pressure drops.
#
# Each cycle is four moves, so the run takes roughly
# CYCLE_COUNT * 4 * CYCLE_HOLD_MS. At the numbers below that is
# about 24 seconds. Press B to stop early.
CYCLE_COUNT    = 10
CYCLE_HOLD_MS  = 600   # how long to sit in each position


# ============================================================
#  STATE
# ============================================================

pivot_up = False
grab_closed = False

pivot_btn_prev = False
grab_btn_prev = False
cycle_btn_prev = False
reset_btn_prev = False


# ============================================================
#  SCREENS
# ============================================================

def show():
    # Draws the current state on both screens. The brain screen
    # gets the full picture because it is easier to read while
    # you have your hands in the claw.

    if pivot_up:
        a = "UP"
    else:
        a = "DOWN"

    if grab_closed:
        b = "CLOSED"
    else:
        b = "OPEN"

    brain.screen.clear_screen()

    brain.screen.set_cursor(1, 1)
    brain.screen.print("CLAW TEST")

    brain.screen.set_cursor(3, 1)
    brain.screen.print("WRIST   (port A): " + a)

    brain.screen.set_cursor(4, 1)
    brain.screen.print("FINGERS (port B): " + b)

    brain.screen.set_cursor(6, 1)
    brain.screen.print("A = grip   Y = wrist")

    brain.screen.set_cursor(7, 1)
    brain.screen.print("X = cycle  DOWN = reset")

    brain.screen.set_cursor(8, 1)
    brain.screen.print("B = stop cycle test")

    controller.screen.clear_screen()
    controller.screen.set_cursor(1, 1)
    controller.screen.print("Wrist " + a)
    controller.screen.set_cursor(2, 1)
    controller.screen.print("Grip  " + b)


# ============================================================
#  PISTON CONTROL
# ============================================================

def pivot_set(up):
    global pivot_up
    pivot_up = up
    claw_pivot.set(up)
    show()


def grab_set(closed):
    global grab_closed
    grab_closed = closed
    claw_grab.set(closed)
    show()


def reset_both():
    # Back to the known startup state, same as a fresh boot.
    #
    # Sets the pistons directly instead of calling pivot_set and
    # grab_set, because those each redraw the screen and the
    # controller screen drops text if you write to it twice in a
    # row this fast. One draw at the end is enough.
    global pivot_up
    global grab_closed

    pivot_up = PIVOT_START_UP
    grab_closed = GRAB_START_CLOSED

    claw_pivot.set(pivot_up)
    claw_grab.set(grab_closed)

    show()


# ============================================================
#  AUTOMATIC CYCLE TEST
#
#  Runs the pistons back and forth on their own. Two things to
#  watch while it runs:
#
#    1. Listen. A steady hiss between shots means a leak at a
#       fitting. Air is the thing we run out of in a match, and
#       a slow leak will empty the tank before the match ends.
#
#    2. Watch the last few cycles. If the grip closes fine at
#       cycle 1 but weakly at cycle 10, the tank was not fully
#       charged or the cylinder is undersized for the job.
# ============================================================

def hold(ms):
    # A wait you can interrupt. Sits for ms milliseconds but
    # checks B the whole time, so a test gone wrong can be
    # stopped without yanking the battery.
    #
    # Returns True if B was pressed, meaning "stop everything".
    waited = 0
    while waited < ms:
        if controller.buttonB.pressing():
            return True
        wait(20, MSEC)
        waited += 20
    return False


def cycle_test():
    for i in range(CYCLE_COUNT):

        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        brain.screen.print("CYCLE TEST - B to stop")
        brain.screen.set_cursor(3, 1)
        brain.screen.print(
            "cycle {} of {}".format(i + 1, CYCLE_COUNT)
        )

        controller.screen.clear_screen()
        controller.screen.set_cursor(1, 1)
        controller.screen.print(
            "cycle {}/{}".format(i + 1, CYCLE_COUNT)
        )
        controller.screen.set_cursor(2, 1)
        controller.screen.print("B = stop")

        # Fingers first, wrist second, so you can hear them as
        # two distinct shots instead of one blur.
        #
        # Every step is checked for the stop button. Bailing out
        # mid-cycle is fine because reset_both() below puts both
        # pistons back to a known state on the way out.
        claw_grab.set(True)
        if hold(CYCLE_HOLD_MS):
            break

        claw_pivot.set(True)
        if hold(CYCLE_HOLD_MS):
            break

        claw_grab.set(False)
        if hold(CYCLE_HOLD_MS):
            break

        claw_pivot.set(False)
        if hold(CYCLE_HOLD_MS):
            break

    controller.rumble(". .")
    reset_both()

    # Wait for B to come back up. Otherwise the main loop sees
    # it still held and we would fall straight back in here.
    while controller.buttonB.pressing():
        wait(20, MSEC)


# ============================================================
#  MAIN LOOP
# ============================================================

def main():
    global pivot_btn_prev
    global grab_btn_prev
    global cycle_btn_prev
    global reset_btn_prev

    reset_both()

    while True:

        pivot_btn = controller.buttonY.pressing()
        grab_btn = controller.buttonA.pressing()
        cycle_btn = controller.buttonX.pressing()
        reset_btn = controller.buttonDown.pressing()

        # Every one of these fires on the press, not while held.
        # Without that check the piston would re-fire 50 times a
        # second for as long as your thumb is down.
        if pivot_btn and not pivot_btn_prev:
            pivot_set(not pivot_up)

        if grab_btn and not grab_btn_prev:
            grab_set(not grab_closed)

        if cycle_btn and not cycle_btn_prev:
            cycle_test()

        if reset_btn and not reset_btn_prev:
            reset_both()

        pivot_btn_prev = pivot_btn
        grab_btn_prev = grab_btn
        cycle_btn_prev = cycle_btn
        reset_btn_prev = reset_btn

        wait(20, MSEC)


main()


# ============================================================
#  WHAT TO CHECK WHILE TESTING
#
#  1. Does each button move the RIGHT piston? If A moves the
#     wrist and Y moves the fingers, the two solenoid cables
#     are swapped in ports A and B. Swap the cables rather than
#     the code, so this file and driver_control.py stay in
#     agreement.
#
#  2. Is each piston the right way round? If pressing A OPENS
#     the fingers when it should close them, either flip
#     GRAB_START_CLOSED and the meaning of the button, or just
#     swap the two air lines on that cylinder. Swapping the
#     tubing is usually faster and it fixes both files at once.
#
#  3. Does the grip actually hold a pin? Clamp one, then try to
#     pull it out by hand. It should not come free easily. If it
#     does, the problem is grip geometry or air pressure, not
#     code.
#
#  4. Run the cycle test with the tank at full charge and count
#     how many cycles you get before the grip goes weak. That
#     number is your real budget for a match. If it is under
#     about 30, plan on fewer claw actions or a second tank.
#
#  5. Once everything here behaves, the same port and button
#     layout is already in driver_control.py, so it will act the
#     same way on the full robot.
# ============================================================
