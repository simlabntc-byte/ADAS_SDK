import cv2 as cv2
import zmq
import numpy as np
from ruamel.yaml import YAML
import RHMsg.RadarMeasurement
import RHMsg.PointCloudPoint

import ws_api.ws_session as ws_session_lib

def process_radar_message(buffer):
    radar_msg = RHMsg.RadarMeasurement.RadarMeasurement.GetRootAsRadarMeasurement(buffer, 0)


    # read and validate the header (if it exists, otherwise print warning message)
    if radar_msg.Header() is not None:
        header = radar_msg.Header()
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

    print ("Messages ", radar_msg.RadarSensorLength())
    # read radar data
    for ii in range(radar_msg.RadarSensorLength()):
        # get one from the array like so:
        radar_object = radar_msg.RadarSensor(ii)
        # and access its fields like so:
        # a track id for the object
        object_id = radar_object.Id()
        # status of the track
        # (enum, either RHMsg::RadarObjectStatus_NEW or RHMsg::RadarObjectStatus_TRACKED)
        status = radar_object.Status()
        # range (meters to the object)
        object_range = radar_object.Range()
        # yaw angle or azimuth between the sensor and the object (radians)
        yaw = radar_object.Yaw()
        # power (in decibels) of the return
        power_db = radar_object.PowerDb()
        # rate of change of the range from sensor to object (meters per second)
        range_rate = radar_object.RangeRate()
        # the object's speed in a direction perpendicular to the sensor (meters per second)
        lat_rate = radar_object.LatRate()   

        print (object_id, status, object_range, yaw, power_db, range_rate, lat_rate)


worldsim_host="http://localhost:8080"
ego_name="ego"
sensor_name="Radar"
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
            process_radar_message(buffer)

    except zmq.ZMQError as e:
        if e.errno == zmq.EAGAIN:
            # this is not really an error, just no message was received
            pass
        else:
            print("ZMQ error: " + e.strerror)