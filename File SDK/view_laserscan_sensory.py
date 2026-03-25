import cv2 as cv2
import zmq
import numpy as np
import RHMsg.LaserscanMeasurement

from ruamel.yaml import YAML

import ws_api.ws_session as ws_session_lib


def process_laserscan_sensor_message(buffer):
    laserscan_sensor_msg = RHMsg.LaserscanMeasurement.LaserscanMeasurement.GetRootAsLaserscanMeasurement(buffer, 0)

    # read and validate the header (if it exists, otherwise print warning message)
    if laserscan_sensor_msg.Header() is not None:
        header = laserscan_sensor_msg.Header()
        try:
            #assert header.Source() == RHMsg.Source.Source.POINTCLOUD_SENSOR

            timestamp = header.Timestamp()
            sequence_no = header.SequenceNo()

            #assert timestamp > self.last_timestamp
            last_timestamp = timestamp
            #assert sequence_no > self.last_sequence_no
            last_sequence_no = sequence_no

        except Exception as error:
            print("ERROR:", error)
    else:
        print("Warning: received buffer with no header.")

    ranges_length=laserscan_sensor_msg.RangesLength()
    ranges_array=laserscan_sensor_msg.RangesAsNumpy()
    
    '''
    for r in range(0,ranges_length):
        range_value=laserscan_sensor_msg.Ranges(r)
        print (range_value)
    '''

    print (sequence_no, timestamp, ranges_length)
    print (ranges_array)
     


worldsim_host="http://localhost:8080"
ego_name="ego"
sensor_name="LaserScanner"
scenario_filename="./scenarios/wolfsburg_sensors.json"

wss=ws_session_lib.worldsim_session(worldsim_host,fps=60)
wss.run_worldsim_distributed(sensor_node=True)
wss.reset()
wss.load_scenario_file(scenario_filename)
wss.scenario_play()

ws_sensor=ws_session_lib.worldsim_sensor(worldsim_host,ego_name,sensor_name)
endpoint, tcp, port = ws_sensor.get_sensor_endpoint_by_name()
print (sensor_name, endpoint, tcp, port)

sensor_config=ws_sensor.get_sensor_config_by_name()
print (sensor_config)


context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt(zmq.CONFLATE, 1) # Get the latest message
socket.connect(endpoint)
socket.setsockopt_string(zmq.SUBSCRIBE, '')
socket.setsockopt(zmq.RCVTIMEO, 1000)

while True:
    try:
        buffer = socket.recv()
        if buffer is not None:
            # Do something with the message
            process_laserscan_sensor_message(buffer)

    except zmq.ZMQError as e:
        if e.errno == zmq.EAGAIN:
            # this is not really an error, just no message was received
            pass
        else:
            print("ZMQ error: " + e.strerror)