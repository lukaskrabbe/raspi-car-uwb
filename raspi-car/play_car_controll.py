"""
Car Controller, implements a easy Interface to drive the Car
author: Lukas Krabbe
"""


from base_scripts import base
import time

def forward(speed=0.5, hold_time=0):
    """
    drives the vehicle forward
    :param speed:
    :param time:
    :return:
    """
    if speed > 1.0:
        raise ValueError('The Speedlimit is fixed by 1.0')

    if hold_time < 0:
        raise ValueError("Can't wait negativ Time")

    for speed_tmp in [x * 0.1 for x in range(0, int(speed*10))]:
        base.set_speed(speed_tmp-0.022, speed_tmp+0.015)
        time.sleep(0.2)

    time.sleep(hold_time)

    base.set_speed(0.0, 0.0)

def left(angle=90):
    """
    makes a hard Turn to the left
    ! angle is just a Approximation !
    :param angle: angle on wich the Car should turn Right !Approximantion!
    :return:
    """

    if angle > 180:
        raise ValueError('The angle has to be smaller than 180')

    hold_time = (angle/18.75)

    base.set_speed(0.0, 0.1)

    time.sleep(hold_time)

    base.set_speed(0.0, 0.0)

def right(angle=90):
    """
    makes a hard Turn to the right
    ! angle is just a Approximation !
    :param angle: angle on wich the Car should turn Right !Approximantion!
    :return:
    """

    if angle > 180:
        raise ValueError('The angle has to be smaller than 180')

    hold_time = (angle/18.75)

    base.set_speed(0.1, 0.0)

    time.sleep(hold_time)

    base.set_speed(0.0, 0.0)



if __name__ == '__main__':

    forward(speed=0.3, hold_time=9)
