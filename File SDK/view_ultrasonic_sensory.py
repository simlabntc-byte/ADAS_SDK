import cv2 as cv2
import zmq
import numpy as np
from ruamel.yaml import YAML
import RHMsg.UltrasonicMeasurement

import ws_api.ws_session as ws_session_lib

# Supported Objects
# environment.structure.building
# environment.support.*
# object.animal
# object.vehicle
# object.pedestrian
# object.obstacle*
# object.construction*
# sign*


def process_virtual_sensor_message(buffer):
    virtual_sensor_msg = RHMsg.UltrasonicMeasurement.UltrasonicMeasurement.GetRootAsUltrasonicMeasurement(buffer, 0)

    # read and validate the header (if it exists, otherwise print warning message)
    if virtual_sensor_msg.Header() is not None:
        header = virtual_sensor_msg.Header()
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

    range=virtual_sensor_msg.Range()
    valid=virtual_sensor_msg.Valid()

    print (sequence_no, timestamp, range, valid)
     


worldsim_host="http://localhost:8080"
ego_name="ego"
sensor_name="Ultrasonic"
scenario_filename="./scenarios/wolfsburg_sensors.json"

wss=ws_session_lib.worldsim_session(worldsim_host,fps=60)
wss.run_worldsim_distributed(sensor_node=True)
wss.reset()
wss.load_scenario_file(scenario_filename)
wss.scenario_play()


ws_sensor=ws_session_lib.worldsim_sensor(worldsim_host,ego_name,sensor_name)
endpoint, tcp, port = ws_sensor.get_sensor_endpoint_by_name()
print (sensor_name, endpoint, tcp, port)



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
            process_virtual_sensor_message(buffer)

    except zmq.ZMQError as e:
        if e.errno == zmq.EAGAIN:
            # this is not really an error, just no message was received
            pass
        else:
            print("ZMQ error: " + e.strerror)