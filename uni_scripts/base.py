import pigpio
import time,sys, tty, termios, os, readchar
from threading import Thread

#
# pigpio uses BROADCOM PIN NUMBERING !!!
#
MotLeft = 18 # Physical Pin #12
MotRight = 19 # Pysical Pin #35

GPIO = pigpio.pi()

# Set the GPIO-Mode to ALT5 for HW-PWM
GPIO.set_mode(MotLeft, pigpio.ALT5)
GPIO.set_mode(MotRight, pigpio.ALT5)

# Der Servo-Motor wird ueber die Impulslaenge gesteuert.
# Der Motor erwartet Impulse von 700us bis 2300us bei 50 Hz
# 50 Hz machen 20ms Periodendauer.
# Der Befehl fuer die "Geschwindigkeit" der Motoren lautet:
# GPIO.hardware_PWM(GPIO, 50, 115000)
# Die Werte haben folgende Bedeutung:
# GPIO gibt die Pinnummer an, an der das Signal ausgegeben werden soll.
# Der mittlere Wert (50) gibt die Frequenz an.
# Der letzte Wert ist der Duty Cycle oder auf deutsch das Tastverhaeltnis.
# Dee Duty Cycle gibt das Verhaeltnis von Impulsdauer zu Periodendauer an.
# 115000 sind 11,5% von 20ms macht 2,3ms .
# Daraus ergibt sich als Einstellbereich fuer die Geschwindigkeit von Bereich
# von 35000 (700us) bis 1150000 (2,3ms).
# Laut Datenblatt drehen sich die Achsen der Motoren bei 75000 (1,5ms) nicht.
# 1,5 ms = 75000 Stillstand der Motoren
# Kleiner 1,5ms bzw. 75000 drehen im Uhrzeigersinn (CW)
# groesser 1,5ms bzw. 75000 drehen entgegen des Uhrzeigersinns (CCW)


# Start the signal generation


#Position "90" (1.5ms pulse) is stop, "180" (2ms pulse) is full speed
# forward, "0" (1ms pulse) is full speed backwards



# We allow speeds from -1.0 to 1.0 for each tire
def set_speed(l, r):
    # hertz
    hertz = 50

    r *= -1 # flip the right side


    # clamp values to valid ranges
    l = max(-1.0, min(1.0, l))
    r = max(-1.0, min(1.0, r))

    # duty length in milli seconds, e.g. 20ms for 50 hertz
    full_duty_length = (1000.0/hertz)
    min_duty_ratio = 1.0 / full_duty_length # the min duty ratio in percent
    max_duty_ratio = 2.0 / full_duty_length # the max duty ratio in percent

    duty_ratio_l = min_duty_ratio + (max_duty_ratio-min_duty_ratio)*(0.5*(l+1.0))
    duty_ratio_r = min_duty_ratio + (max_duty_ratio-min_duty_ratio)*(0.5*(r+1.0))

    GPIO.hardware_PWM(MotLeft, hertz, (int)(duty_ratio_l*1000000.0))
    GPIO.hardware_PWM(MotRight, hertz, (int)(duty_ratio_r*1000000.0))


if __name__ == '__main__':
    set_speed(float(sys.argv[1]), float(sys.argv[2]))
