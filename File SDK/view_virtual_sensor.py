import cv2 as cv2
import zmq
import numpy as np
from ruamel.yaml import YAML
import RHMsg.VirtualSensorMeasurement

import ws_api.ws_session as ws_session_lib
import csv
import json

lane_marking_verbose=False
map_object_verbose=False
obsticle_verbose=False


with open('./data/2025.1_tags.json', 'r') as file:
    id_lookup_2025=json.load(file)
for k, v in id_lookup_2025.items():
    id_lookup_2025[k]=", ".join(str(x) for x in v)
print (id_lookup_2025)


def process_virtual_sensor_message(buffer):
    virtual_sensor_msg = RHMsg.VirtualSensorMeasurement.VirtualSensorMeasurement.GetRootAsVirtualSensorMeasurement(buffer, 0)

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


    print (f'Message {sequence_no} at sim time {timestamp} Contains...')

    lane_markings_length=virtual_sensor_msg.LaneMarkingsLength()
    print (lane_markings_length, ' Lane Markings')
    for i in range(0,lane_markings_length): 
        lane_marking=virtual_sensor_msg.LaneMarkings(i)
        lmc0=lane_marking.C0()
        lmc1=lane_marking.C1()
        lmc2=lane_marking.C2()
        lmc3=lane_marking.C3()
        section_count=lane_marking.SectionsLength()
        point_count=lane_marking.PointsLength()
        #TODO - process sections and points into array
        if lane_marking_verbose:
            print ('    ', i,'lane making: ', lmc0, lmc1, lmc2,lmc3, section_count, point_count)

    map_objects_length=virtual_sensor_msg.MapObjectsLength()
    print (map_objects_length,' Map Objects')
    for i in range(0,map_objects_length):
        map_object=virtual_sensor_msg.MapObjects(i)
        object_size=[map_object.Size().X(), map_object.Size().Y(),map_object.Size().Z()]
        object_rotation=[map_object.Rotation().EX(), map_object.Rotation().EY(), map_object.Rotation().EZ(), map_object.Rotation().E0()]
        object_center=[map_object.Center().X(),map_object.Center().Y(),map_object.Center().Z()]
        tag=map_object.IdTag().Tag()
        object_name='unknown'
        if f'{tag}' in id_lookup_2025.keys():
            object_name=id_lookup_2025[f'{tag}']
            print ('    Object:', i ,tag, object_name)
        if map_object_verbose:            
            print ('        ', 'Object Size:', object_size)
            print ('        ', 'Object Center:', object_center)
            print ('        ', 'Object Rotation:', object_rotation )

    obsticle_length=virtual_sensor_msg.ObstaclesLength()
    print (obsticle_length,' Obsticles')
    for i in range(0,obsticle_length):   
        obsticle=virtual_sensor_msg.Obstacles(i)
        obsticle_size=[obsticle.Size().X(), obsticle.Size().Y(), obsticle.Size().Z()]
        obsticle_rotation=[obsticle.Rotation().EX(), obsticle.Rotation().EY(), obsticle.Rotation().EZ(), obsticle.Rotation().E0()]
        obsticle_center=[obsticle.Center().X(),obsticle.Center().Y(),obsticle.Center().Z()]
        tag=obsticle.IdTag().Tag()
        obsticle_name='unknown'
        if f'{tag}' in id_lookup_2025.keys():
            obsticle_name=id_lookup_2025[f'{tag}']
            print ('    Obsticle', i, tag, obsticle_name)
        if obsticle_verbose:            
            print ('        ', 'Obsticle Size:', obsticle_size )
            print ('        ', 'Obsticle Center:', obsticle_center)
            print ('        ', 'Obsticle Rotation:', obsticle_rotation )    




worldsim_host="http://localhost:8080"
ego_name="ego"
sensor_name="VirtualCamera"
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