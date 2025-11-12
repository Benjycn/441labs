# Reference structure and philisophies of github - ddevoe - stepper_class_shiftregister_multiprocessing.py
import time
import multiprocessing
from ctypes import c_double # leveraged Chatgpt, c_double is used to make angle a shared variable to modify
from shifter import Shifter  

class Stepper:
    num_steppers = 0
    shifter_outputs = multiprocessing.Value("i", 0)  # shared register output across all motors
    seq = [0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001]  # CCW sequence
    delay = 1200            # delay between steps [µs]
    steps_per_degree = 4096 / 360  # steps per degree of shaft rotation

    def __init__(self, shifter, lock):
        self.s = shifter
        self.step_state = 0
        self.shifter_bit_start = 4 * Stepper.num_steppers  # starting bit position in byte
        self.angle = multiprocessing.Value(c_double, 0.0)  # shared angle variable
        self.lock = lock                                   # multiprocessing lock
        Stepper.num_steppers += 1

    def __sgn(self, x):
        return 0 if x == 0 else int(abs(x) / x)

    def __step(self, direction):
        self.step_state = (self.step_state + direction) % 8 # to know where it is in the CCW sequence
        pattern = Stepper.seq[self.step_state]

        with self.lock: # acquire and release combined --> GeeksforGeeks "How to lock critical sections"
            Stepper.shifter_outputs.value &= ~(0b1111 << self.shifter_bit_start) # &= compare left and right, left shift
            Stepper.shifter_outputs.value |= pattern << self.shifter_bit_start # compare seq with shift outputs, outputs correct binary for motor
            self.s.shiftByte(Stepper.shifter_outputs.value) # send to the shift register

        with self.angle.get_lock():
            self.angle.value = (self.angle.value + direction / Stepper.steps_per_degree) % 360

        time.sleep(Stepper.delay / 1e6)

    def __rotate(self, delta): # private
        num_steps = int(Stepper.steps_per_degree * abs(delta)) # find the right number of steps
        direction = self.__sgn(delta) # find direction +/- 1

        for _ in range(num_steps):
            self.__step(direction)

    def rotate(self, delta): # public 
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()

# leveraged chatgpt for get.lock
    def goAngle(self, target): # move angle using shortest possible path
        with self.angle.get_lock():
            curr = self.angle.value % 360.0
        target = target % 360.0

        diff = target - curr
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360

        p = multiprocessing.Process(target=self.__rotate, args=(diff,))
        p.start()

        with self.angle.get_lock():
            self.angle.value = target

    def zero(self): # reset to 0 degrees
        with self.angle.get_lock():
            self.angle.value = 0.0

# reference from ddevo enme441-pi github, stepper shift multiprocessing
if __name__ == '__main__':
    s = Shifter(data=16, latch=20, clock=21)
    lock = multiprocessing.Lock()

    # instantiate two stepper motors
    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    # zero motors
    m1.zero()
    m2.zero()

    print("Starting simultaneous stepper motor sequence...")

    m1.goAngle(90)
    m2.goAngle(-90)

    m1.goAngle(-45)
    m2.goAngle(45)

    m1.goAngle(-135)
    m2.goAngle(135)

    m1.goAngle(0)
    m2.goAngle(0)

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nProgram terminated.")
