"""

"""


from uni_scripts import base
import time

def forward(speed=0.5, time=0):
    """
    drives the vehicle forward

    :param speed:
    :param time:
    :return:
    """
    if speed > 1.0:
        raise ValueError('The Speedlimit is fixed by 1.0')

    if time < 0:
        raise ValueError("Can't wait negativ Time")

    for speed_tmp in [x * 0.1 for x in range(0, (speed*10))]:
        base.set_speed(speed_tmp, speed_tmp)
        time.sleep(0.2)

    time.sleep(time)



if __name__ == '__main__':
    forward(0.3,1)