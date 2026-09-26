# Claw bench test.
#
# Lets us test the two claw pistons without the rest of the robot. All
# that has to be plugged in is the brain, a charged air tank and the two
# solenoids:
#
#   three-wire A  ->  wrist    (pivots the whole claw up and down)
#   three-wire B  ->  fingers  (grip and release)
#
# They are on separate pistons on purpose. We need to pivot while already
# holding a pin, and let go without the wrist moving.
#
# Controls
#   A     grip open / close
#   Y     wrist up / down
#   X     10-cycle leak test
#   B     stop the cycle test
#   DOWN  reset both to their boot state
#
# Bench tool only. There is no Competition object at the bottom, so it
# starts the moment you hit play. Do not load this for a match.

from vex import *

brain = Brain()
controller = Controller()

claw_pivot = DigitalOut(brain.three_wire_port.a)
claw_grab  = DigitalOut(brain.three_wire_port.b)

# Boot state for both pistons. Keep these matched to driver_control.py,
# otherwise what we feel on the bench is not what happens on the field.
PIVOT_START_UP    = False
GRAB_START_CLOSED = False

# Leak test. Two things to watch. A steady hiss between shots means a
# fitting is leaking. And if the grip closes hard on cycle 1 but weakly
# on cycle 10, the tank was not properly charged to begin with.
CYCLE_COUNT    = 10
CYCLE_HOLD_MS  = 600

pivot_up = False
grab_closed = False

pivot_btn_prev = False
grab_btn_prev = False
cycle_btn_prev = False
reset_btn_prev = False


def show():

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
    # Sets the pistons directly rather than going through pivot_set and
    # grab_set, because those each redraw the controller screen and it
    # drops text if you write to it twice this fast. One redraw at the
    # end is enough.
    global pivot_up
    global grab_closed

    pivot_up = PIVOT_START_UP
    grab_closed = GRAB_START_CLOSED

    claw_pivot.set(pivot_up)
    claw_grab.set(grab_closed)

    show()


def hold(ms):
    # A wait we can interrupt. Sits for ms milliseconds but keeps checking
    # B the whole time, so a test going wrong can be stopped without
    # pulling the battery. Returns True if B was pressed.
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

    while controller.buttonB.pressing():
        wait(20, MSEC)


def main():
    global pivot_btn_prev
    global grab_btn_prev
    global cycle_btn_prev
    global reset_btn_prev

    reset_both()

    while True:

        # Each of these fires on the press, not while the button is held.
        # Without the _prev comparison the piston would re-fire fifty
        # times a second for as long as your thumb is down.
        pivot_btn = controller.buttonY.pressing()
        grab_btn = controller.buttonA.pressing()
        cycle_btn = controller.buttonX.pressing()
        reset_btn = controller.buttonDown.pressing()

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
