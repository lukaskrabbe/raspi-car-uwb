from uni_scripts import base
import time




if __name__ == '__main__':
    base.set_speed(0.0, 0.1)

    time.wait(1)

    base.set_speed(0.0, 0.2)

    time.wait(2)

    base.set_speed(0.0, 0.1)

    time.wait(1)

    base.set_speed(0.0, 0.0)