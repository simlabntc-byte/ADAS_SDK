import time
import sqlite3
import cv2 as cv2
import RHMsg.Source
import RHMsg.ImageMeasurement
import RHMsg.ImageEncoding
import os.path

def zmq_message_replay(db_file, sensorname, sensortype, replay_interval=1.0):
    # Open a connection to the SQLite database
    conn = sqlite3.connect(db_file)
    c = conn.cursor()

    print(f"Replaying messages from SQLite database {db_file}...")

    try:
        # Query all messages from the database
        c.execute(f"SELECT timestamp, message FROM messages WHERE sensorname='{sensorname}' AND sensortype='{sensortype}' ORDER BY id")
        rows = c.fetchall()

        for row in rows:
            timestamp, message = row

            # Replay the binary message through the PUB socket
            process_image_message(message)

            # Optionally print the replayed message to the console
            #print(f"Replaying binary message (stored at {timestamp}) of size {len(message)} bytes")

            # Wait for the specified interval before replaying the next message
            time.sleep(replay_interval)

    except KeyboardInterrupt:
        print("Message replay stopped.")
    finally:
        conn.close()

def process_image_message(buffer):
        image_msg = RHMsg.ImageMeasurement.ImageMeasurement.GetRootAsImageMeasurement(buffer, 0)

        # read and validate the header (if it exists, otherwise print warning message)
        if image_msg.Header() is not None:
            header = image_msg.Header()
            assert header.Source() == RHMsg.Source.Source.CAMERA_SENSOR or header.Source() == RHMsg.Source.Source.SEMANTIC_SENSOR
            timestamp = header.Timestamp()
            sequence_no = header.SequenceNo()
            print (sequence_no,timestamp)
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



if __name__ == "__main__":
    db_file = f"./zmq_recordings/zmq_messages_2026_03_24_11_38_30.db"
    sensorName="Camera"
    sensorType='camera'
    replay_interval = 0.01  # Time (in seconds) between replayed messages

    if os.path.isfile(db_file):
        zmq_message_replay(db_file, sensorName, sensorType, replay_interval)
    else:
        print (f"{db_file} does not exist")
