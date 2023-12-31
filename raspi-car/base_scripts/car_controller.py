import time
import math
import logging
import numpy as np
from threading import Lock, Thread
from base_scripts import base

class Car:

    def get_speed(self):
        with self.lock:
            return self.speed

    def set_speed(self, speed):
        with self.lock:
            self.speed = speed

    def get_number_of_estimations(self):
        with self.lock:
            return self.number_of_estimations

    def get_number_of_read_estimations(self):
        with self.lock:
            return self.number_of_read_estimations

    def increment_number_of_read_estimations(self):
        with self.lock:
            self.number_of_read_estimations += 1

    def get_current_estimation_x(self):
        with self.lock:
            return self.current_estimation_x

    def get_current_estimation_y(self):
        with self.lock:
            return self.current_estimation_y

    def set_current_estimation_x_y(self, new_estimation_x, new_estimation_y):
        with self.lock:
            self.current_estimation_x = new_estimation_x
            self.current_estimation_y = new_estimation_y
            self.number_of_estimations += 1

    def get_is_reached_destination(self):
        with self.lock:
            return self.is_reached_destination

    def set_is_reached_destination(self):
        with self.lock:
            self.is_reached_destination = True


    def PID2(self, estimation_x, estimation_y, acceptable_error):
        """
        PID controller for direction and angle calculation
        :param estimation_x:
        :param estimation_y:
        :param acceptable_error:
        :return:
        """

        #calculate current direction from current estimation and previous estimation
        current_direction_x = estimation_x - self.previous_pos_x
        current_direction_y = estimation_y - self.previous_pos_y
        logging.info('PID: previous_pos(x,y): (%d, %d)' % (self.previous_pos_x, self.previous_pos_y))
        logging.info('PID: current_direction(x,y): (%d, %d)' % (current_direction_x, current_direction_y))

        # save current estimations for next round
        self.previous_pos_x = estimation_x
        self.previous_pos_y = estimation_y
        logging.info('PID: estimation(x,y): (%d, %d)' % (estimation_x, estimation_y))

        # calculate goal direction from goal position and current estimation
        goal_direction_x = self.goal_pos_x - estimation_x
        goal_direction_y = self.goal_pos_y - estimation_y
        logging.info('PID: goal_direction(x,y): (%d, %d)' % (goal_direction_x, goal_direction_y))
        
        # calculate error rates for goal position
        error_x = abs(estimation_x - self.goal_pos_x)
        error_y = abs(estimation_y - self.goal_pos_y)
        logging.info('PID: error(x,y): (%d, %d)' % (error_x, error_y))

        # angle is always between 0.0 and 180.0
        angle = np.degrees(self.angle_between((current_direction_x, current_direction_y), (goal_direction_x, goal_direction_y)))
        logging.info('PID: angle to goal: %d' % angle)

        distance = math.sqrt((goal_direction_x - current_direction_x) * (goal_direction_x - current_direction_x) + \
                             (goal_direction_y - current_direction_y) * (goal_direction_y - current_direction_y))
        logging.info('PID: distance to goal: %d' % distance)

        if distance < acceptable_error:
            # Position achieved
            base.set_speed(0.0, 0.0)
            self.speed = 0
            logging.info('PID: Arrived at Goal Destination, distance: %d' % distance)
            return True
        elif goal_direction_x - current_direction_x > 0:
            # Turn right
            hold_time = 0.7
            logging.info('PID: turning RIGHT with angle: %f and hold_time: %d' % (angle, hold_time))
            if(self.continous_drive_mode):
                self.drive_continuous(speed=0.3, change=(angle/180))
            else:
                self.drive(speed=0.3, hold_time=hold_time, change=(angle/180))
            
            return False

        elif goal_direction_x - current_direction_x < 0:
            # Turn left
            hold_time = 0.7
            logging.info('PID: turning LEFT with angle: %f and hold_time: %d' % (angle, hold_time))
            if(self.continous_drive_mode):
                self.drive_continuous(speed=0.3, change=(-1)*(angle/180))
            else:
                self.drive(speed=0.3, hold_time=hold_time, change=(-1)*(angle/180))
            
            return False



    def drive(self, speed=0.5, hold_time=0, change=0):
        """
        Drives the vehicle forward for a specific amount of time
        :param speed:
        :param hold_time:
        :param change:
        :return:
        """

        if speed > 1.0:
            raise ValueError('The Speedlimit is fixed by 1.0')

        if hold_time < 0:
            raise ValueError("Can't wait negativ Time")

        if abs(change) > 1.0 :
            raise ValueError("Change value should be between -1.0 and 1.0")

        if change < 0:
            # left
            l_coefficient = 1 + (change/2)
            r_coefficient = 1 # - (change/2)
        else:
            # right
            l_coefficient = 1 # + (change/2)
            r_coefficient = 1 - (change/2)

        for speed_tmp in [x * 0.1 for x in range(0, int(speed*10))]:
            base.set_speed((speed_tmp * l_coefficient) + self.calibration_left_wheel,
                           (speed_tmp * r_coefficient) + self.calibration_right_wheel)
            time.sleep(0.01)

        time.sleep(hold_time)

        for speed_tmp in [x * 0.1 for x in range(0, int(speed*10))]:
            base.set_speed(((speed-speed_tmp) * l_coefficient) - self.calibration_left_wheel,
                           ((speed-speed_tmp) * r_coefficient) - self.calibration_right_wheel)
            time.sleep(0.01)

        base.set_speed(0.0, 0.0)

    def drive_continuous(self, speed=0.5, change=0):
        """
        Drives the vehicle forward without any breaks
        :param speed:
        :param change:
        :return:
        """

        if speed > 1.0:
            raise ValueError('The Speedlimit is fixed by 1.0')

        if abs(change) > 1.0 :
            raise ValueError("Change value should be between -1.0 and 1.0")

        if change < 0:
            # left
            l_coefficient = 1 + (change/2)
            r_coefficient = 1 # - (change/2)
        else:
            # right
            l_coefficient = 1 # + (change/2)
            r_coefficient = 1 - (change/2)

        if (self.speed < speed):
            # increse the current speed to match goal speed
            '''
            for speed_tmp in [x * 0.1 for x in range(self.speed, int(speed*10))]:
                base.set_speed((speed_tmp * l_coefficient) + self.calibration_left_wheel,
                            (speed_tmp * r_coefficient) + self.calibration_right_wheel)
                time.sleep(0.01)
            '''
            base.set_speed((speed * l_coefficient) + self.calibration_left_wheel,
                            (speed * r_coefficient) + self.calibration_right_wheel)

        elif (self.speed > speed):
            # decrease the current speed to match goal speed
            '''
            for speed_tmp in [x * 0.1 for x in reversed(range(self.speed, int(speed*10)))]:
                base.set_speed(((self.speed-speed_tmp) * l_coefficient) - self.calibration_left_wheel,
                            ((self.speed-speed_tmp) * r_coefficient) - self.calibration_right_wheel)
                time.sleep(0.01)
            '''
            base.set_speed((speed * l_coefficient) + self.calibration_left_wheel,
                            (speed * r_coefficient) + self.calibration_right_wheel)
        self.speed = speed


    def unit_vector(self, vector):
        """
        Returns the unit vector of the vector.  
        :param vector:
        """
        return vector / np.linalg.norm(vector)

    def angle_between(self, v1, v2):
        """
        Returns the angle in radians between vectors 'v1' and 'v2'
        :param v1:
        :param v2:
        """
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))



    def __init__(self, goal_x, goal_y, calibration_left_wheel, calibration_right_wheel, drive_mode):
        """
        :param goal_x:
        :param goal_y:
        :param calibration_left_wheel:
        :param calibration_right_wheel:
        :param drive_mode:
        """
        self.goal_pos_x = goal_x
        self.goal_pos_y = goal_y
        self.calibration_left_wheel = calibration_left_wheel
        self.calibration_right_wheel = calibration_right_wheel
        self.is_reached_destination = False
        self.mode = 0 #0 for stationary or 1 for moving
        self.continous_drive_mode = drive_mode
        self.current_estimation_x = 0
        self.current_estimation_y = 0
        self.previous_pos_x = 0
        self.previous_pos_y = 0
        self.previous_direction_x = 0
        self.previous_direction_y = 0
        self.speed = 0
        self.number_of_estimations = 0
        self.number_of_read_estimations = 0
        self.lock = Lock()
