import zmq
import RHMsg.GroupedStateMeasurement
import ws_api.ws_session as ws_session_lib

# Custom state message handler to extract GPS Data of desired vehicle
def get_gps_loction(buffer, ego_uuid_str):
    states_msg = RHMsg.GroupedStateMeasurement.GroupedStateMeasurement.GetRootAsGroupedStateMeasurement(buffer, 0)

    # read and validate the header (if it exists, otherwise print warning message)
    if states_msg.Header() is not None:
        header = states_msg.Header()
        try:
            timestamp = header.Timestamp()
            sequence_no = header.SequenceNo()

            #assert timestamp > self.last_timestamp
            last_timestamp = timestamp
            #assert sequence_no > self.last_sequence_no
            last_sequence_no = sequence_no

            #print (timestamp, sequence_no)

        except Exception as error:
            print("ERROR:", error)
    else:
        print("Warning: received buffer with no header.")

    returndata={"X":0,
              "Y":0,
              "Z":0,
              "lon":0,
              "lat":0,
              "alt":0}

    # read physics vehicles state data (these are vehicles using PhysX,
    # typically being driven by the traffic system)
    for ii in range(states_msg.PhysicsVehicleStatesLength()):
        physics_vehicle_state = states_msg.PhysicsVehicleStates(ii)
        # print ("PhysicsVehicleStatesLength",states_msg.PhysicsVehicleStatesLength())
        uuid = physics_vehicle_state.Header().Uuid()
        uuid_parts = ego_uuid_str.split('-')
        uuid_a = int(uuid_parts[0][:8], 16)
        uuid_b = int(uuid_parts[1][:4] + uuid_parts[2][:4], 16)
        uuid_c = int(uuid_parts[3][:4] + uuid_parts[4][:4], 16)
        uuid_d = int(uuid_parts[4][4:], 16)
        if uuid_a==uuid.A() and uuid_b==uuid.B() and uuid_c==uuid.C() and uuid_d==uuid.D():
            location=physics_vehicle_state.Location()
            location_wgs84 = physics_vehicle_state.LocationWgs84()
            returndata["X"]=location.X()
            returndata["Y"]=location.Y()
            returndata["Z"]=location.Z()
            returndata["lat"]=location_wgs84.Y()
            returndata['lon']=location_wgs84.X()
            returndata["alt"]=location_wgs84.Z()

    '''
    #########################################################################################################
    # Examples of state messages (PhysicsVehicleStates, AdvancedVehicle, KinematicStates, PedestrianStates) #
    #########################################################################################################
    
    # read physics vehicles state data (these are vehicles using PhysX,
    # typically being driven by the traffic system)
    print ("PhysicsVehicleStatesLength",states_msg.PhysicsVehicleStatesLength())
    for ii in range(states_msg.PhysicsVehicleStatesLength()):
        physics_vehicle_state = states_msg.PhysicsVehicleStates(ii)
        # and access its fields like so:
        # agent's UUID
        uuid = physics_vehicle_state.Header().Uuid()
        # location in the world's local Cartesian frame
        location = physics_vehicle_state.Location()
        # location in a WGS84 coordinate system (i.e. GPS)
        # x is longitude (degrees), y is latitude (degrees), z is altitude (meters)
        location_wgs84 = physics_vehicle_state.LocationWgs84()
        # orientation (as quaternion) in the world's local Cartesian frame
        orientation = physics_vehicle_state.Orientation()
        # velocity in the agent's body frame
        velocity = physics_vehicle_state.Velocity()
        # angular velocity in the agent's body frame
        angular_velocity = physics_vehicle_state.AngularVelocity()
        # collisions with other agents
        for ci in range(physics_vehicle_state.Header().CollisionsLength()):
            collision = physics_vehicle_state.Header().Collisions(ci)

    # read advanced vehicles state data
    print ("AdvancedVehicleStatesLength",states_msg.AdvancedVehicleStatesLength())
    for ii in range(states_msg.AdvancedVehicleStatesLength()):
        advanced_vehicle_state = states_msg.AdvancedVehicleStates(ii)
        # and access its fields like so:
        # agent's UUID
        uuid = advanced_vehicle_state.Header().Uuid()
        # location in the world's local Cartesian frame
        location = advanced_vehicle_state.Location()
        # location in a WGS84 coordinate system (i.e. GPS)
        # x is longitude (degrees), y is latitude (degrees), z is altitude (meters)
        location_wgs84 = advanced_vehicle_state.LocationWgs84()
        # orientation (as quaternion) in the world's local Cartesian frame
        orientation = advanced_vehicle_state.Orientation()
        # velocity in the agent's body frame
        velocity = advanced_vehicle_state.Velocity()
        # angular velocity in the agent's body frame
        angular_velocity = advanced_vehicle_state.AngularVelocity()
        # collisions with other agents
        for ci in range(advanced_vehicle_state.Header().CollisionsLength()):
            collision = advanced_vehicle_state.Header().Collisions(ci)

    # read kinematic state data (the most general state output from the engine)
    print ("KinematicStatesLength",states_msg.KinematicStatesLength())
    for ii in range(states_msg.KinematicStatesLength()):
        kinematic_state = states_msg.KinematicStates(ii)
        # and access its fields like so:
        # agent's UUID
        uuid = kinematic_state.Header().Uuid()
        # location in the world's local Cartesian frame
        location = kinematic_state.Location()
        # location in a WGS84 coordinate system (i.e. GPS)
        # x is longitude (degrees), y is latitude (degrees), z is altitude (meters)
        location_wgs84 = kinematic_state.LocationWgs84()
        # orientation (as quaternion) in the world's local Cartesian frame
        orientation = kinematic_state.Orientation()
        # velocity in the agent's body frame
        velocity = kinematic_state.Velocity()
        # angular velocity in the agent's body frame
        angular_velocity = kinematic_state.AngularVelocity()
        # collisions with other agents
        for ci in range(kinematic_state.Header().CollisionsLength()):
            collision = kinematic_state.Header().Collisions(ci)

    # read pedestrian state data
    print ("PedestrianStatesLength",states_msg.PedestrianStatesLength())
    for ii in range(states_msg.PedestrianStatesLength()):
        pedestrian_state = states_msg.PedestrianStates(ii)
        # and access its fields like so:
        # agent's UUID
        uuid = pedestrian_state.Header().Uuid()
        # location in the world's local Cartesian frame
        location = pedestrian_state.Location()
        # location in a WGS84 coordinate system (i.e. GPS)
        # x is longitude (degrees), y is latitude (degrees), z is altitude (meters)
        location_wgs84 = pedestrian_state.LocationWgs84()
        # orientation (as quaternion) in the world's local Cartesian frame
        orientation = pedestrian_state.Orientation()
        # velocity in the agent's body frame
        velocity = pedestrian_state.Velocity()
        # angular velocity in the agent's body frame
        angular_velocity = pedestrian_state.AngularVelocity()
        # collisions with other agents
        for ci in range(pedestrian_state.Header().CollisionsLength()):
            collision = pedestrian_state.Header().Collisions(ci)
        '''
    return returndata


worldsim_host="http://localhost:8080"
scenario_filename="./scenarios/mcity_solo_wander.json"
ego_name="ego"
wss=ws_session_lib.worldsim_session(worldsim_host)
wss.run_worldsim()
wss.reset()
wss.load_scenario_file(scenario_filename)

# Get the states endpoint for the ego vehicle
ego_uuid_str=wss.get_ego_vehicle_uuid_str()
endpoint=wss.get_states_endpoint()
print ("states_endpoint",endpoint)

# Connect to the ZMQ bus via the states endpoint
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt(zmq.CONFLATE, 1) # Get the latest message
socket.connect(endpoint)
socket.setsockopt_string(zmq.SUBSCRIBE, '')
socket.setsockopt(zmq.RCVTIMEO, 1000)

# Start the Scenario
wss.scenario_play()

# Main Loop
while True:
    try:
        buffer = socket.recv()
        if buffer is not None:
            # Do something with the message
            position_dict=get_gps_loction(buffer,ego_uuid_str)
            print (position_dict)

    except zmq.ZMQError as e:
        if e.errno == zmq.EAGAIN:
            # this is not really an error, just no message was received
            pass
        else:
            print("ZMQ error: " + e.strerror)