import RPi.GPIO as GPIO
import time

# test shift register code, problem 2, referenced module 5 last page

"""
GPIO.setmode(GPIO.BCM)
serialPin, latchPin, clockPin = 23, 24, 25

GPIO.setup(serialPin, GPIO.OUT)
GPIO.setup(latchPin, GPIO.OUT, initial=0)
GPIO.setup(clockPin, GPIO.OUT, initial=0)

def ping(p):
    GPIO.output(p, 1)
    time.sleep(0)
    GPIO.output(p, 0)

def shiftByte(b):
    for i in range(8):
        GPIO.output(serialPin, b & (1 << i))
        ping(clockPin)
    ping(latchPin)

try:
    while 1:
        for i in range(2**8):
            shiftByte(i)
            time.sleep(0.5)
except KeyboardInterrupt:
    GPIO.cleanup()
"""

# referenced module 2, page 86 for classes and objects example
class Shifter:
    def __init__(self, serialPin, latchPin, clockPin): # instantiation
        # constructor
        self.serialPin = serialPin
        self.latchPin = latchPin
        self.clockPin = clockPin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.serialPin, GPIO.OUT)
        GPIO.setup(self.latchPin, GPIO.OUT, initial=0)
        GPIO.setup(self.clockPin, GPIO.OUT, initial=0)

# referenced module 2, page 92, private vs public class variables/methods
    def __ping(self, p): # private
        GPIO.output(p, 1)
        time.sleep(0)
        GPIO.output(p, 0)

    def shiftByte(self, b): # public
        for i in range(8):
            GPIO.output(self.serialPin, b & (1 << i))
            self.__ping(self.clockPin)
        self.__ping(self.latchPin)


# 4 - random walk
#import RPi.GPIO as GPIO
#import time
import random
# import class, https://www.geeksforgeeks.org/python/how-to-import-a-class-from-another-file-in-python/
#from shifter import Shifter 


"""
bug = Shifter(serialPin=23, latchPin=24, clockPin=25)

try:
    position = 0  
    leds = 8 

    while True:
        # leveraged chatgpt to learn the bitwise left shift 
        # --> moves 1 bit to left by position, so that only 1 LED on 
        led_pattern = 1 << position
        bug.shiftByte(led_pattern)

        # random between -1 or +1 position, https://www.geeksforgeeks.org/python/random-choices-method-in-python/ 
        step = random.choice([-1, 1])
        position += step

        if position < 0:
            position = 0
        elif position > leds - 1:
            position = leds - 1

        time.sleep(0.05)

except KeyboardInterrupt:
    GPIO.cleanup()
"""

# leveraged chatgpt to help with module 6 threading, so that lightningbug runs concurrently
import threading

class Bug:
    def __init__(self, timestep = 0.1, x = 3, isWrapOn = False):
        self.timestep = timestep
        self.x = x # 0-7
        self.isWrapOn = isWrapOn
        self.__shifter = Shifter(23, 24, 25) # private
        self.__running = False
        self.__thread = None

    def __lightningbug(self):
        leds = 8
        while self.__running:
            led_pattern = 1 << self.x
            self.__shifter.shiftByte(led_pattern)

            step = random.choice([-1, 1])
            new_x = self.x + step

            if self.isWrapOn:
                self.x = new_x % leds # leveraged chatgpt to wrap around with % for pos and neg
            else:
                if new_x < 0:
                    self.x = 0 # left bound
                elif new_x >= leds:
                    self.x = leds - 1 # right bound
                else:
                    self.x = new_x

            time.sleep(self.timestep)

    def start(self):
        if not self.__running:
            self.__running = True
            self.__thread = threading.Thread(target=self.__lightningbug, daemon=True) # referenced module 6 page 7 to run function concurrently
            self.__thread.start() 

    def stop(self):
        self.__running = False
        if self.__thread:
            self.__thread.join() # module 6, "force calling process to wait for the thread to end before continuing"
        self.__shifter.shiftByte(0)

    def wrap(self):
        self.isWrapOn = not self.isWrapOn
        print(f"Wrap mode: {'ON' if self.isWrapOn else 'OFF'}")

    def speed(self):
        self.timestep = 0.03 if self.timestep == 0.1 else 0.1
        print(f"Speed changed to: {self.timestep}")

s1 = 17 
s2 = 27  
s3 = 22  

GPIO.setmode(GPIO.BCM)
GPIO.setup(s1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(s2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(s3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

bug = Bug() # default timestep=0.1, x=3, isWrapOn=False

def s1_switch(pin):
    if bug._Bug__running: # used Chatgpt to help with name-mangling
        bug.stop()
    else:
        bug.start()

def s2_switch(pin):
    bug.wrap()

def s3_switch(pin):
    bug.speed()

GPIO.add_event_detect(s1, GPIO.RISING, callback=s1_switch, bouncetime=300)
GPIO.add_event_detect(s2, GPIO.RISING, callback=s2_switch, bouncetime=300)
GPIO.add_event_detect(s3, GPIO.RISING, callback=s3_switch, bouncetime=300)

try:
    while True:
        time.sleep(1)  

except KeyboardInterrupt:
    print('\nExiting')
    bug.stop()
    GPIO.cleanup()
