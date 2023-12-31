import numpy as np
import logging
import time
from threading import Lock, Thread
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from base_scripts import car_controller
from base_scripts import listen



NUMBER_OF_NODES = 7
NUMBER_OF_DISTANCES_PER_ANCHOR = 1
MAX_ERROR_VALUE = 20    # In CM total Distance
READINGS_FOR_INITIALIZATION = 5
USE_KALMAN_FILTER = True

FINAL_POSITION = (80, 200) # (X, Y)

birkan_responder_locations = [{8192: (0, 0, 0)}, {8193: (0, 100, 0)},
                             {8194: (0, 200, 0)}, {8195: (128, 200, 0)},
                             {8196: (128, 100, 0)}, {8197: (128, 0, 0)}, 
                             {8198: (64, 300, 0)}]

lukas_responder_locations = [{8196: (-30, 20, 0)}, {8194: (340, 44, 0)},
                             {8195: (108, 340, 0)}, {8197: (393, 380, 2)},
                             {8193: (0, 0, 32)}, {8198: (102, 0, 19)},
                             {8199: (-18, 200, 18)}]

lukas_calibration_left_wheel = -0.022
lukas_calibration_right_wheel = +0.015

birkan_calibration_left_wheel = 0.0
birkan_calibration_right_wheel = 0.02

def main():
    logging.basicConfig(level='DEBUG')

    rpi_car = car_controller.Car(FINAL_POSITION[0], FINAL_POSITION[1], lukas_calibration_left_wheel, lukas_calibration_right_wheel, False)

    thread_sense = Thread(target=sense, args=(rpi_car,))
    logging.info('Created a thread for location estimation')
    thread_drive = Thread(target=drive, args=(rpi_car,))
    logging.info('Created a thread for car movement')
    thread_sense.start()
    logging.info('Started the thread for location estimation')
    thread_drive.start()
    logging.info('Started the thread for car movement')


def sense(rpi_car):
    """
    Calculates the current position estimation by
        Reading sensor data, 
        Applying localisation,
        And then passing it through Kalman filter.
    """
    
    #  First construct the filter with the required dimensionality
    # 2 dimensional position (x,y) and 2 dimensional velocity (vx, vy)
    # Warning: dim_x cannot be bigger than 4 as Q_discrete_white_noise only works for dim_x values between 2-4
    filter = KalmanFilter(dim_x=4, dim_z=2)

    # Assign the initial value for the state (position and velocity)
    filter.x = np.array([[0.],
                         [0.],
                         [0],
                         [0]])

    # Define the state transition matrix
    filter.F = np.array([[1., 0., 1., 0.],
                         [0., 1., 0., 1.],
                         [0., 0., 1., 0.],
                         [0., 0., 0., 1.]])

    # Define the measurement function
    filter.H = np.array([[1., 0., 0., 0.],
                         [0., 1., 0., 0.]])

    # Define the covariance matrix
    filter.P *= 1000.
    # filter.P = np.array([[1000.,    0.],
    #                    [   0., 1000.] ])

    # Assign the measurement noise. Here the dimension is 1x1, a scalar is used
    # This must be a 2 dimensional array, as must all the matrices.
    # state uncertainty
    filter.R = 5
    # filter.R = np.array([[5.]])

    # Assign the process noise
    # process uncertainty
    filter.Q = Q_discrete_white_noise(dim=4, dt=0.1, var=0.13)

    while not rpi_car.get_is_reached_destination():
        start = time.time_ns() 

        target_position = listen.get_coordinates(number_of_nodes=NUMBER_OF_NODES,
                                                 iter_counter=NUMBER_OF_DISTANCES_PER_ANCHOR,
                                                 responder_locations=lukas_responder_locations)
        print(target_position)
        coordinates = [target_position.__dict__['x'], target_position.__dict__['y']]

        if(USE_KALMAN_FILTER):
            filter.predict()
            filter.update(coordinates)

            # filter.x includes (x,y,vx,vy) estimates
            estimate_pos_x = filter.x.tolist()[0][0]
            estimate_pos_y = filter.x.tolist()[1][0]

            rpi_car.set_current_estimation_x_y(estimate_pos_x, estimate_pos_y)
            logging.info('Sensing: New position estimation(x,y): (%d, %d)' % (estimate_pos_x, estimate_pos_y))
        else:
            rpi_car.set_current_estimation_x_y(target_position.__dict__['x'], target_position.__dict__['y'])
            logging.info('Sensing: New position estimation(x,y): (%d, %d)' % (target_position.__dict__['x'], target_position.__dict__['y']))

        end = time.time_ns() 
        logging.info('Sensing: It took %d miliseconds for sensor sensing' % ((end - start)/1000000))

def drive(rpi_car):

    local_number_of_estimations = 0

    while not rpi_car.get_is_reached_destination():

        if (local_number_of_estimations < rpi_car.get_number_of_estimations()):
            logging.info('Driving: New data received!')

            local_number_of_estimations += 1
            local_estimation_x = rpi_car.get_current_estimation_x()
            local_estimation_y = rpi_car.get_current_estimation_y()

            # stationary estimation mode
            if rpi_car.mode == 0:
                
                # estimate initial position with 10 readings
                if local_number_of_estimations == READINGS_FOR_INITIALIZATION:
                    rpi_car.previous_pos_x = local_estimation_x
                    rpi_car.previous_pos_y = local_estimation_y
                    logging.info('Driving: first_init_pos_estimation(x,y): (%d, %d)' % (local_estimation_x, local_estimation_y))
                    rpi_car.drive(speed=0.3, hold_time=1, change=0)

                # estimate new position after going forward with 10 new readings
                #  and set continuous driving mode
                elif local_number_of_estimations == READINGS_FOR_INITIALIZATION*2:
                    rpi_car.mode = 1
                    logging.info('Driving: second_init_pos_estimation(x,y): (%d, %d)' % (local_estimation_x, local_estimation_y))
                    logging.info('Driving: Initialization Done, turn into Drive Mode')

            # continous estimation and driving mode
            elif rpi_car.mode == 1:
                logging.info('Driving: pos_estimation(x,y): (%d, %d)' %(local_estimation_x, local_estimation_y))
                if rpi_car.PID2(local_estimation_x, local_estimation_y, acceptable_error=MAX_ERROR_VALUE):
                    logging.info('Driving: Arrived at Final Possition!')
                    rpi_car.set_is_reached_destination()
            
        #else:
        #    logging.info('Driving: No new data received!')



if __name__ == '__main__':
    main()