import cv2 as cv2
import zmq
import numpy as np
from ruamel.yaml import YAML
import RHMsg.Source
#import RHMsg.SemanticImageMeasurement
import RHMsg.ImageMeasurement
import RHMsg.ImageEncoding

import ws_api.ws_session as ws_session_lib
import csv
import json


with open('./data/2025.1_tags.json', 'r') as file:
    id_lookup_2025=json.load(file)
for k, v in id_lookup_2025.items():
    id_lookup_2025[k]=", ".join(str(x) for x in v)
print (id_lookup_2025)

def process_image_message(buffer):
        #image_msg = RHMsg.SemanticImageMeasurement.SemanticImageMeasurement.GetRootAsSemanticImageMeasurement(buffer, 0)
        image_msg = RHMsg.ImageMeasurement.ImageMeasurement.GetRootAsImageMeasurement(buffer, 0)

        # read and validate the header (if it exists, otherwise print warning message)
        if image_msg.Header() is not None:
            header = image_msg.Header()
            assert header.Source() == RHMsg.Source.Source.CAMERA_SENSOR or header.Source() == RHMsg.Source.Source.SEMANTIC_SENSOR
            timestamp = header.Timestamp()
            sequence_no = header.SequenceNo()
        else:
            print("Warning: received buffer with no header.")

        #print (sequence_no, timestamp)

        # read image metadata
        encoding = image_msg.Encoding()
        width = image_msg.Width()
        height = image_msg.Height()

        pixel_values = image_msg.ValuesAsNumpy().copy()
        unique_ids=np.unique(pixel_values[::3])

        #pixel_values[::2]=0
        #pixel_values[::3]=0

        print (len(unique_ids))
        tag_name='Unknown'
        for value in unique_ids:
            if f'{value}' in id_lookup_2025.keys():
                tag_name=id_lookup_2025[f'{value}']
            print (value, tag_name)
        #print ("unique_ids", len(unique_ids))

        # read and validate the pixel values (if they exist, otherwise print warning message)
        if pixel_values.size != 0:

            open_cv_image = pixel_values.reshape(height,width,3)

            #all_rgb_codes = open_cv_image.reshape(-1, open_cv_image.shape[-1])
            #unique_rbg = np.unique(all_rgb_codes, axis=0)
            #print (unique_rbg)
            #print ("unique rbg values", len(unique_rbg))


            #open_cv_image = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2RGB)
            #open_cv_image = cv2.cvtColor(open_cv_image, cv2.COLOR_RGB2HSV)
            #open_cv_image = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2GRAY)
            #open_cv_image = cv2.resize(open_cv_image, (640,480), interpolation = cv2.INTER_AREA)

            cv2.imshow("Semantic Camera Sensor",open_cv_image)
            cv2.waitKey(1)

        else:
            print("Warning: received image message with no pixels.")


worldsim_host="http://localhost:8080"
ego_name="ego"
sensor_name="SemanticCamera"
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

print ("Running loop")

while True:
    try:
        buffer = socket.recv()
        if buffer is not None:
            # Do something with the message
            print ('** Processing Message **')
            process_image_message(buffer)

    except zmq.ZMQError as e:
        if e.errno == zmq.EAGAIN:
            # this is not really an error, just no message was received
            print ("No Msg")
            pass
        else:
            print("ZMQ error: " + e.strerror)