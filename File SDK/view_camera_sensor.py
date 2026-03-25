import cv2 as cv2
import zmq
import RHMsg.Source
import RHMsg.ImageMeasurement
import RHMsg.ImageEncoding
import ws_api.ws_session as ws_session_lib

def process_image_message(buffer):
        image_msg = RHMsg.ImageMeasurement.ImageMeasurement.GetRootAsImageMeasurement(buffer, 0)

        # read and validate the header (if it exists, otherwise print warning message)
        if image_msg.Header() is not None:
            header = image_msg.Header()
            assert header.Source() == RHMsg.Source.Source.CAMERA_SENSOR or header.Source() == RHMsg.Source.Source.SEMANTIC_SENSOR
            timestamp = header.Timestamp()
            sequence_no = header.SequenceNo()
        else:
            print("Warning: received buffer with no header.")

        # read image metadata
        encoding = image_msg.Encoding()
        width = image_msg.Width()
        height = image_msg.Height()

        pixel_values = image_msg.ValuesAsNumpy()

        # read and validate the pixel values (if they exist, otherwise print warning message)
        if pixel_values.size != 0:
            # check the size of the pixels vector
            if encoding is RHMsg.ImageEncoding.ImageEncoding.RGB8:
                assert len(pixel_values) == width * height * 3

                open_cv_image = pixel_values.reshape(height,width,3)
                open_cv_image = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2RGB)
                #open_cv_image = cv2.resize(open_cv_image, (640,480), interpolation = cv2.INTER_AREA)
                cv2.imshow("Camera Sensor",open_cv_image)
                cv2.waitKey(1)
                
            else:
                assert len(pixel_values) == width * height
        else:
            print("Warning: received image message with no pixels.")


worldsim_host="http://localhost:8080"
ego_name="ego"
sensor_name="Camera"
scenario_filename="./scenarios/wolfsburg_sensors.json"

wss=ws_session_lib.worldsim_session(worldsim_host,fps=60)
wss.run_worldsim()
#wss.run_worldsim_distributed(sensor_node=True)
#wss.reset()
wss.connect_to_worldim() 

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
            process_image_message(buffer)

    except zmq.ZMQError as e:
        if e.errno == zmq.EAGAIN:
            # this is not really an error, just no message was received
            pass
        else:
            print("ZMQ error: " + e.strerror)