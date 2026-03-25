import zmq
import time
import sqlite3
import os.path

def zmq_message_replay_stream(db_file, replay_address, sensorname, sensortype, replay_interval=1.0):
    """Replay ZMQ binary messages from an SQLite database."""
    # Create a ZeroMQ context
    context = zmq.Context()

    # Create a PUB socket to replay messages
    socket = context.socket(zmq.PUB)

    # Bind the socket to the given replay address
    socket.bind(replay_address)

    # Open a connection to the SQLite database
    conn = sqlite3.connect(db_file)
    c = conn.cursor()

    print(f"Replaying messages from SQLite database {db_file} to {replay_address}...")

    try:
        # Query all messages from the database
        c.execute(f"SELECT timestamp, message FROM messages WHERE sensorname='{sensorname}' AND sensortype='{sensortype}' ORDER BY id")
        rows = c.fetchall()

        for row in rows:
            timestamp, message = row

            # Replay the binary message through the PUB socket
            socket.send(message)

            # Optionally print the replayed message to the console
            print(f"Replaying binary message (stored at {timestamp}) of size {len(message)} bytes")

            # Wait for the specified interval before replaying the next message
            time.sleep(replay_interval)

    except KeyboardInterrupt:
        print("Message replay stopped.")
    finally:
        conn.close()


if __name__ == "__main__":

    db_file = f"./zmq_recordings/zmq_messages_2024_11_27_15_15_02.db"
    sensorName="Camera"
    sensorType='camera'
    replay_address = "tcp://localhost:5556"
    replay_interval = 0.01  # Time (in seconds) between replayed messages

    if os.path.isfile(db_file):
        zmq_message_replay_stream(db_file, replay_address, sensorName, sensorType, replay_interval=1.0)
    else:
        print (f"{db_file} does not exist")


