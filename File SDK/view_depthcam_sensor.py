import zmq
import cv2 as cv2
import numpy as np
from ruamel.yaml import YAML
import RHMsg.Source
import RHMsg.DepthImageMeasurement
import RHMsg.ImageEncoding

import ws_api.ws_session as ws_session_lib

first_image=True
def process_depthcamera_image_message(buffer):
        global max_dist,first_image
        depth_image_msg = RHMsg.DepthImageMeasurement.DepthImageMeasurement.GetRootAsDepthImageMeasurement(buffer, 0)

        # read and validate the header (if it exists, otherwise print warning message)
        if depth_image_msg.Header() is not None:
            header = depth_image_msg.Header()
            assert header.Source() == header.Source() == RHMsg.Source.Source.DEPTH_SENSOR
            timestamp = header.Timestamp()
            sequence_no = header.SequenceNo()
        else:
            print("Warning: received buffer with no header.")

        pixel_length=depth_image_msg.PixelsLength() 

        # read image metadata
        encoding = depth_image_msg.Encoding()
        width = depth_image_msg.Width()
        height = depth_image_msg.Height()
        
        # read and validate the pixel values (if they exist, otherwise print warning message)
        if depth_image_msg.PixelsLength() != 0:
            if first_image:
                first_image=False
                return
            # check the size of the pixels vector
            o = depth_image_msg._tab.Bytes
            pixel_array=np.frombuffer(o, np.float32, offset=0)

            # Fudge trim the array first & last 2 points are extra data from flatbuffer
            pixel_array=pixel_array[12:-8]

            # Get every second point - Depth array
            depth_array = pixel_array[::2].copy()
            depth_array[depth_array == np.inf] = max_dist
            n = 2
            Intensity_array=pixel_array[n-1::n].copy()
            Intensity_array[Intensity_array>0]=1

            depth_array=255*(depth_array/max_dist)*Intensity_array
            #depth_array=depth_array.astype(np.uint8)
            #depth_array = (depth_array-np.min(depth_array))/(np.max(depth_array)-np.min(depth_array))
            print (depth_array)

            open_cv_image=depth_array.reshape(height,width)
            open_cv_image=open_cv_image.astype('uint8')
            #open_cv_image = cv2.cvtColor(open_cv_image, cv2.COLOR_GRAY2BGR)
            #open_cv_image = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2RGB)
            #open_cv_image = cv2.resize(open_cv_image, (640,480), interpolation = cv2.INTER_AREA)
            #cv2.imshow("image", open_cv_image.astype(np.uint8))

            cv2.imshow("DepthCamera Sensor",open_cv_image)
            cv2.waitKey(1)


            '''
            if (pixels) {                
                            size = width * height;
            
                            for (int i = 0; i < width; i++) {
                                for (int j = 0; j < height; j++) {
                                    auto depth_pixel = pixels->Get(i + j * width);
                                    if ((double)depth_pixel->depth() != std::numeric_limits< double >::infinity()) {
                                        if ((double)depth_pixel->intensity() > 0) {
                                            y0[i * height + j] = ((uint8_t)255 * depth_pixel->depth() / max_depth);
                                            y1[i * height + j] = ((uint8_t)255 * depth_pixel->depth() / max_depth);
                                            y2[i * height + j] = ((uint8_t)255 * depth_pixel->depth() / max_depth);
                                        }
                                    } else {
                                        y0[i * height + j] = 255;
                                        y1[i * height + j] = 255;
                                        y2[i * height + j] = 255;
                                    }
                                }
                            }
                        }
            '''

            #else:
            #    assert depth_image_msg.PixelsLength() == width * height
        else:
            print("Warning: received image message with no pixels.")


worldsim_host="http://localhost:8080"
ego_name="ego"
sensor_name="DepthCamera"
scenario_filename="./scenarios/wolfsburg_sensors.json"

wss=ws_session_lib.worldsim_session(worldsim_host,fps=60)
wss.run_worldsim_distributed(sensor_node=True)
wss.reset()
wss.load_scenario_file(scenario_filename)
wss.scenario_play()

ws_sensor=ws_session_lib.worldsim_sensor(worldsim_host,ego_name,sensor_name)
endpoint, tcp, port = ws_sensor.get_sensor_endpoint_by_name()
print (sensor_name, endpoint, tcp, port)


sensor_info=ws_sensor.get_sensor_config_by_name()
max_dist=sensor_info["max_dist"]
print ("max_dist ",max_dist)

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect(endpoint)
socket.setsockopt_string(zmq.SUBSCRIBE, '')
socket.setsockopt(zmq.RCVTIMEO, 1000)

while True:
    try:
        buffer = socket.recv()
        if buffer is not None:
            # Do something with the message
            process_depthcamera_image_message(buffer)

    except zmq.ZMQError as e:
        if e.errno == zmq.EAGAIN:
            # this is not really an error, just no message was received
            print ("No message")
            pass
        else:
            print("ZMQ error: " + e.strerror)