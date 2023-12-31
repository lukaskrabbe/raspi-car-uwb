"""
Simple example program that sends a MIN frame and displays all received MIN frames.
"""
from time import sleep


import struct
import logging
import pandas as pd
import localization as lx

from base_scripts import distance_data
from base_scripts import min


# Linux USB serial ports are of the form '/dev/ttyACM*'
# macOS USB serial ports are of the form '/dev/tty.usbmodem*'
# Windows randomly assigns COM ports, depending on the driver for the USB serial chip.
# Genuine FTDI chips tend to end up at the same port between reboots. YMMV.
MIN_PORT = '/dev/tty.usbmodem1421'
MIN_PORT = '/dev/ttyACM0'
#MIN_PORT = '/dev/tty.usbmodem0007601203521' #BLACK


def _get_coordinates_on_distance(responder_locations, distance, solver='LSE'):
    """

    :param responder_locations:
    :param distance:
    :param solver:
    :return:
    """
    logging.info('Started: Calculate Position based on Coordinates using %s' % solver)
    P = lx.Project(mode='3D', solver=solver)

    for index, row in distance.iterrows():
        for dict in responder_locations:
            if index in dict.keys():
                position = dict[index]
                P.add_anchor(str(index), position)

    t, label = P.add_target()

    for index, row in distance.iterrows():
        for dict in responder_locations:
            if index in dict.keys():
                t.add_measure(str(index), row['Distance in CentiMeters'])

    P.solve()
    position = t.loc
    logging.info('Finished: Calculate Position based on Coordinates')
    logging.info('Position: %s' %position)

    return position


def _wait_for_frames(min_handler: min.MINTransportSerial):
    """

    :param min_handler:
    :return:
    """
    while True:
        # The polling will generally block waiting for characters on a timeout
        # How much CPU time this takes depends on the Python serial implementation
        # on the target machine
        frames = min_handler.poll()
        if frames:
            return frames


def get_coordinates(number_of_nodes, iter_counter=1, responder_locations=[]):
    """

    :param number_of_nodes:
    :param responder_locations:
    :return:
    """
    logging.info('Started: Request Data from Serial Port')

    min_handler = min.MINTransportSerial(port=MIN_PORT, loglevel='INFO')

    buffer_source_data = []
    counter = 0
    done = False
    iter_done = False
    number_of_values = 0
    while True:
        for frame in _wait_for_frames(min_handler):
            value = struct.unpack('HHHdd', frame.payload)

            id = value[0]
            dest_addr = value[1]
            source_addr = value[2]
            distance = value[3]
            distance_bias = value[4]

            if not any(source_data.source_addr == source_addr for source_data in buffer_source_data):
                # Initialize list of distance_data_set for dest_addr and source_addr‚
                if len(buffer_source_data) < number_of_nodes:
                    buffer_source_data.append(distance_data.distance_data_set(dest_addr, source_addr))
                    # Add distance to distance_data_set in source_data
                    for source_data in buffer_source_data:
                        if source_data.source_addr == source_addr:
                            source_data.set_distance(distance)
                            source_data.set_distance_bias(distance_bias)
                            number_of_values += 1
            else:
                # Add distance to distance_data_set in source_data
                for source_data in buffer_source_data:
                    if source_data.source_addr == source_addr:
                        source_data.set_distance(distance)
                        source_data.set_distance_bias(distance_bias)
                        number_of_values += 1

            if all(source_data.get_distance_size() >= iter_counter for source_data in buffer_source_data) and len(buffer_source_data) >= number_of_nodes:
                iter_done = True
                break

        if iter_done:
            iter_done = False
            break
        else:
            # sleep(0.5)
            logging.info('No sensor data, currently sleeping!!!')
            counter += 1
            if counter > (number_of_nodes * 2):
                logging.error('Finished: Request Data from Serial Port, got zero Values!')
                return None

    logging.info('Finished: Request Data from Serial Port, got %d Values' % number_of_values)

    data = []
    for object in buffer_source_data:
        value = (object.get_source_addr(), object.get_distance_norm(), object.get_distance_bias_norm())
        data.append(value)

    data_frame = pd.DataFrame(data, columns=['Source', 'Distance', 'Distance with Range Bias']).set_index('Source')
    data_frame['Distance in CentiMeters'] = (data_frame['Distance with Range Bias'] * 100).to_list()

    # print(data_frame)

    if number_of_nodes >= 3:
        return _get_coordinates_on_distance(responder_locations, data_frame[['Distance in CentiMeters']])
    else:
        return (0, 0, 0)




