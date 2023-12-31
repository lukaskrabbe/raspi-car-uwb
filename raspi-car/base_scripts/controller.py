import pigpio
import time,sys, tty, termios, os, readchar
from threading import Thread
from base import set_speed

input = 'controller'

# Initially set the speed to 0
set_speed(0, 0)

if input == 'controller':
    from evdev import InputDevice, categorize, ecodes

    #creates object 'gamepad' to store the data
    gamepad = InputDevice('/dev/input/event0')

    trigger_left = 10
    trigger_right = 9

    min_val = 0
    max_val = 1023

    speed_left = 0
    speed_right = 0

    def update_speed_thread():
        while True:
            time.sleep(0.01)
            print("Setting speed to ({}, {})".format(speed_left, speed_right))
            set_speed(speed_left, speed_right)

    t = Thread(target=update_speed_thread)
    t.start()

    #loop and filter by event code and print the mapped label
    for event in gamepad.read_loop():
        if event.code == trigger_left:
            speed_left = (event.value-min_val) / float(max_val)
        if event.code == trigger_right:
            speed_right = (event.value-min_val) / float(max_val)
