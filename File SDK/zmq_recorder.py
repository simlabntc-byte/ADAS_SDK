import time
from datetime import datetime
import sqlite3
import os
import zmq
import ws_api.ws_session as ws_session_lib


def initialize_database(db_file):
    """Initialize the SQLite database with the necessary table."""
    conn = sqlite3.connect(db_file)
    c = conn.cursor()

    # Create a table to store binary messages
    c.execute('''CREATE TABLE IF NOT EXISTS messages
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  timestamp TEXT NOT NULL,
                  sensorname TEXT NOT NULL,
                  sensortype TEXT NOT NULL,
                  message BLOB NOT NULL)''')

    conn.commit()
    conn.close()

def zmq_message_recorder(bind_address, sensorname, sensortype, db_file):
    """Record ZMQ binary messages into an SQLite database."""
    # Initialize the SQLite database
    initialize_database(db_file)

    # Create a ZeroMQ context
    context = zmq.Context()
    # Create a SUB socket to subscribe to messages
    socket = context.socket(zmq.SUB)
    # Set to get the lastest message
    #socket.setsockopt(zmq.CONFLATE, 1) # Get the latest message
    # Connect the socket to the given address
    socket.connect(bind_address)
    # Subscribe to all topics (empty string means subscribe to all)
    socket.setsockopt_string(zmq.SUBSCRIBE, '')

    # Open a connection to the SQLite database
    conn = sqlite3.connect(db_file)
    c = conn.cursor()
    #c.execute("PRAGMA synchronous = OFF")
    #c.execute("PRAGMA journal_mode = OFF")

    print(f"Recording messages from {bind_address} into SQLite database: {db_file}...")

    try:
        while True:
            try:
                buffer = socket.recv()
                if buffer is not None:
                    # Received a binary message from the socket

                    # Record the message with a timestamp
                    timestamp=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    c.execute("INSERT INTO messages (timestamp,sensorname, sensortype, message) VALUES (?, ?, ?, ?)", (timestamp, sensorname, sensortype,  buffer))
                    conn.commit()

                    # Optionally print the message to the console
                    print(f"[{timestamp}] Received binary message of size {len(buffer)} bytes")

            except zmq.ZMQError as e:
                if e.errno == zmq.EAGAIN:
                    # this is not really an error, just no message was received
                    pass
                else:
                    print("ZMQ error: " + e.strerror)


    except KeyboardInterrupt:
        print("Message recording stopped.")
    except Exception as err:
        print(f"Unexpected {err=}, {type(err)=}")
    finally:
        conn.close()


if __name__ == "__main__":
    worldsim_host="http://localhost:8080"
    ego_name="ego"
    scenario_filename="./scenarios/prova.json"
    sensorName="Camera"
    sensoreType="camera"

    #sensorName="VirtualCamera"
    #sensoreType="virtualcamera"

    wss=ws_session_lib.worldsim_session(worldsim_host,fps=60)
    wss.run_worldsim_deterministic()
    wss.reset()
    wss.load_scenario_file(scenario_filename)
    wss.scenario_play()

    ws_sensor=ws_session_lib.worldsim_sensor(worldsim_host,ego_name,sensorName)
    endpoint, tcp, port = ws_sensor.get_sensor_endpoint_by_name()
    print (sensorName, endpoint, tcp, port)
    zmq_address = endpoint

    if not os.path.exists("./zmq_recordings"):
        os.makedirs("./zmq_recordings")
        
    # The SQLite database file where the messages will be recorded
    timestamp = time.strftime('%Y_%m_%d_%H_%M_%S')
    db_file = f"./zmq_recordings/zmq_messages_{timestamp}.db"

    zmq_message_recorder(zmq_address, sensorName, sensoreType,  db_file)


