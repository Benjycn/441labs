# stepper_class_shiftregister_multiprocessing_completed.py
#
# Stepper class – Lab 8 completed version
#
# Multiprocessing is used instead of threading because the Global Interpreter Lock (GIL)
# on the Raspberry Pi Zero prevents true parallel execution of threads.
# Using multiprocessing allows two or more motors to run simultaneously.

import time
import multiprocessing
from shifter import Shifter   # custom class for controlling 74HC595 shift register


class Stepper:
    """
    Supports operation of an arbitrary number of stepper motors using
    one or more shift registers.

    A class attribute (shifter_outputs) keeps track of all
    shift register output values for all motors. This makes
    simultaneous operation possible when different processes are running.

    Motor instantiation order is inverted from shift register outputs:
    Motor 1 uses upper bits, Motor 2 uses lower bits, etc.
    """

    # Class attributes
    num_steppers = 0        # track number of Stepper instances created
    shifter_outputs = 0     # shared register output for all motors
    seq = [0b0001, 0b0011, 0b0010, 0b0110,
           0b0100, 0b1100, 0b1000, 0b1001]  # CCW sequence for 28BYJ-48
    delay = 1200            # delay between steps [µs]
    steps_per_degree = 4096 / 360  # 4096 steps per revolution

    def __init__(self, shifter, lock):
        self.s = shifter
        self.angle = 0
        self.step_state = 0
        self.shifter_bit_start = 4 * Stepper.num_steppers
        self.lock = lock
        Stepper.num_steppers += 1

    # internal sign function
    def __sgn(self, x):
        return 0 if x == 0 else int(abs(x) / x)

    # perform a single step in given direction
    def __step(self, direction):
        self.step_state = (self.step_state + direction) % 8

        # update this motor's 4-bit output pattern
        # first clear this motor's bits
        Stepper.shifter_outputs &= ~(0b1111 << self.shifter_bit_start)
        # then set the new pattern
        Stepper.shifter_outputs |= Stepper.seq[self.step_state] << self.shifter_bit_start

        # shift out to 74HC595
        self.s.shiftByte(Stepper.shifter_outputs)

        # update angle
        self.angle = (self.angle + direction / Stepper.steps_per_degree) % 360

    # internal rotation method (executed in separate process)
    def __rotate(self, delta):
        with self.lock:  # ensure one motor accesses hardware at a time
            num_steps = int(Stepper.steps_per_degree * abs(delta))
            direction = self.__sgn(delta)
            for _ in range(num_steps):
                self.__step(direction)
                time.sleep(Stepper.delay / 1e6)

    # public rotation method that spawns a process
    def rotate(self, delta):
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()

    # move motor to absolute angle using shortest rotation path
    def goAngle(self, target):
        # normalize target
        target %= 360
        curr = self.angle % 360
        diff = target - curr

        # choose shortest path
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360

        # run as separate process
        p = multiprocessing.Process(target=self.__rotate, args=(diff,))
        p.start()

    # set motor zero position
    def zero(self):
        self.angle = 0


# Example use:
if __name__ == '__main__':
    s = Shifter(data=16, latch=20, clock=21)
    lock = multiprocessing.Lock()

    # instantiate 2 stepper motors
    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    # zero motors
    m1.zero()
    m2.zero()

    # Demonstration sequence
    print("Starting motor sequence...")

    # Move one motor sequentially
    m1.rotate(90)
    m1.rotate(-45)
    m1.rotate(90)
    m1.rotate(-45)

    # Run both motors simultaneously using the same lock object
    m2.rotate(180)
    m2.rotate(-45)
    m2.rotate(45)
    m2.rotate(-180)

    # main loop continues while motors operate in background
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\nExiting program")
