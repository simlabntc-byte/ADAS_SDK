import json
import numpy as np
import pygame

import zmq
import RHMsg.GroupedStateMeasurement
import RHMsg.HeaderData
import RHMsg.KinematicStateMeasurement
import RHMsg.KinematicActuationCommand
import RHMsg.Source
import RHMsg.UUID

import ws_api.ws_session as ws_session_lib


# Consumes the resulting state from the system
class StateConsumer:
    def __init__(self, endpoint, uuids):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(endpoint)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, '') # Remove any filtering of messages, receive everything
        self.socket.setsockopt(zmq.RCVTIMEO, 0) # Don't block if there aren't any state updates
        self.uuids = uuids

    def receive(self):
        try:
            buffer = self.socket.recv()
            if buffer is not None:
                states_msg = RHMsg.GroupedStateMeasurement.GroupedStateMeasurement.GetRootAsGroupedStateMeasurement(buffer, 0)
                #print (states_msg.KinematicStatesLength())
                #for ii in range(states_msg.PhysicsVehicleStatesLength()):
                #    physics_vehicle_state = states_msg.PhysicsVehicleStates(ii)
                
                #if states_msg.KinematicStatesLength()==1:
                #    physics_vehicle_state = states_msg.PhysicsVehicleStates(1)
                #    location=[physics_vehicle_state.Location().X(),physics_vehicle_state.Location().Y(),physics_vehicle_state.Location().Z()]
                #    print (location)


                # Get Vehicle Position
                #print (physics_vehicle_state.Location)                
                
                traffic_signals_state = states_msg.TrafficSignals()
                signame=[] # Array of UUID
                sigstate=[] # Array of Signal States
                signal_data={}
                for ii in range(traffic_signals_state.NamesLength()):
                    #signame.append(traffic_signals_state.Names(ii).decode('utf-8'))
                    #sigstate.append(traffic_signals_state.States(ii))

                    id=traffic_signals_state.Names(ii).decode('utf-8')
                    value=traffic_signals_state.States(ii)
                    if value != 0:
                        signal_data[id]=value
                
                #print (signame,sigstate)
                return signal_data

                # Note: Need to export the paths.geojson, and use the coordinates to determine which name (actually a signal UUID) is at the desired location.
                #states_array = traffic_signals_state.StatesAsNumpy()
                #print (states_array)


                '''
                class TrafficSignalState(object):
                    off = 1
                    red = 2
                    yellow = 3
                    green = 4
                    red_left = 5
                    yellow_left = 6
                    green_left = 7
                    red_right = 8
                    yellow_right = 9
                    green_right = 10
                    red_up = 11
                    yellow_up = 12
                    green_up = 13
                    red_flashing = 14
                    yellow_flashing = 15
                '''
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                # Not really an error, just an empty message
                pass
            else:
                raise

if __name__ == "__main__":
    report_closest=True
    fps = 60
    frame_time = 1.0 / fps

    worldsim_host="http://localhost:8080"
    scenario_filename="./scenarios/mcity_user_control.json"
    ego_name="ego"
    wss=ws_session_lib.worldsim_session(worldsim_host)
    wss.run_worldsim()
    wss.reset()
   
    print (wss.get_config())

    # Request a configuration change to some agreed upon version and set the simulation to advance without
    # this client directly driving time advance
    # - user_vehicle_kinematic - True if we provide state, False if we provide actuation and the physics engine computes state
    # - async_reporting means the simulation will advance time on its own and broadcast state regularly
    config = { 'version': { 'major': 2025 }, 'async_reporting': True, 'frame_rate': fps, "signal_state_reporting": True, "user_vehicle_snap_to_ground": True}
    #config = { 'version': { 'major': 2023 }, 'async_reporting': True, 'frame_rate': fps,  "user_vehicle_kinematic":True, "user_vehicle_snap_to_ground": True }
    wss.set_config(config)

    # Load the specified scenario
    wss.load_scenario_file(scenario_filename)

    geojson=wss.get_paths_geojson("MCity")
    trafficlight_dict={}
    UUID_list=[]
    coord_list=[]
    features=geojson['features']
    for item in features:
        properties=item['properties']
        if properties['object_type']=='TrafficLight':
            geometry=item['geometry']
            trafficlight_dict[properties['UUID']]={
                'coordinates':geometry['coordinates'],
                'heading':properties['heading'],
                'horizontal':properties['horizontal']
            }
            UUID_list.append(properties['UUID'])
            coord_list.append((geometry['coordinates'][0],geometry['coordinates'][1]))
    coord_list = np.array(coord_list)
    print (f'Found {len(trafficlight_dict)} Traffic Lights in geojson')
    #print (trafficlight_dict)

    traffic_signal_state_lookup={
                    1:'off',
                    2:'red',
                    3:'yellow',
                    4:'green',
                    5:'red_left',
                    6:'yellow_left',
                    7:'green_left',
                    8:'red_right',
                    9:'yellow_right',
                    10:'green_right',
                    11:'red_up',
                    12:'yellow_up',
                    13:'green_up',
                    14:'red_flashing',
                    15:'yellow_flashing'
    }


    ego_config = { 'name': 'ego', 'kinematic': True }
    wss.set_user_controlled_agent(ego_config)
    ego_agent = wss.get_user_controlled_agent(ego_config['name'])
    actuation_endpoint = ego_agent['endpoint']
    ego_uuid = ego_agent['uuid']


    # Connect to the vehicle's actuation socket to take control
    controller = ws_session_lib.VehicleController(actuation_endpoint, ego_uuid)

    # Set up something to read back resulting state from the system
    states_endpoint = wss.get_states_endpoint()
    states = StateConsumer(states_endpoint, [ego_uuid])

    # The 'play' command tells WorldSim to stop waiting for any pending connections and start simulating
    wss.scenario_play()

    # Main simulation loop

    initial_motion_vector=np.array([1,0,0])
    x=0
    y=0
    z=0
    pitch=0.0
    roll=0.0
    yaw=0.0
    ts_disp=0
    sim_time = 0
    linear_velocity=0.01
    angular_velocity=0.1
    
    location = np.array([x,y,z])
    orientation=ws_session_lib.toQuaternion(ws_session_lib.degrees_to_rads(pitch),ws_session_lib.degrees_to_rads(roll),ws_session_lib.degrees_to_rads(yaw),False)

    controller.send_KinematicActuationCommand(sim_time,location,orientation)

    # initialize controller
    pygame.init()
    pygame.joystick.init()
    joystick_count = pygame.joystick.get_count()
    print (f"Found {joystick_count} connected devices")
    #if joystick_count > 1:
    #    raise ValueError("Please Connect one Joystick")
    if joystick_count ==0 :
        raise ValueError("No joystick connected")
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    dead_zone=0.2
     
    while True:
        sim_time = sim_time + frame_time
        pygame.event.get()
        numAxes = joystick.get_numaxes()
        jsInputs = [float(joystick.get_axis(i)) for i in range(numAxes)]
        jsButtons = [float(joystick.get_button(i)) for i in
                    range(joystick.get_numbuttons())] 
        rot=jsInputs[2]
        if rot>dead_zone or rot<-1*dead_zone:
            pass
        else:
            rot=0
        forward_trans=jsInputs[1]
        if forward_trans>dead_zone or forward_trans<-1*dead_zone:
            pass
        else:
            forward_trans=0
        ts_disp=-1*forward_trans*linear_velocity
        yaw=yaw-rot*angular_velocity   


        signal_states=states.receive()
        if signal_states:
            #print (f'{sim_time} - Found {len(signal_states)} Traffic light states')
            if report_closest:
                veh_xy=(location[0],location[1])
                distances = np.linalg.norm(coord_list-veh_xy, axis=1)
                min_index = np.argmin(distances)
                sig_id=UUID_list[min_index]
                print(f"Closest Signal is at {coord_list[min_index]}, distance = {distances[min_index]} meters, state {traffic_signal_state_lookup[signal_states[sig_id]]} - {sig_id}")
            else:
                for sig_id in signal_states:
                    print (f"Signal at {trafficlight_dict[sig_id]['coordinates']} has state={signal_states[sig_id]} - {traffic_signal_state_lookup[signal_states[sig_id]]}")



        motion_vector=ws_session_lib.rotate_vector(initial_motion_vector,ws_session_lib.degrees_to_rads(yaw),[0,0,1])
        uv = motion_vector / np.linalg.norm(motion_vector)
        location = location + ts_disp * uv
        orientation=ws_session_lib.toQuaternion(ws_session_lib.degrees_to_rads(pitch),ws_session_lib.degrees_to_rads(roll),ws_session_lib.degrees_to_rads(yaw),False)
        controller.send_KinematicActuationCommand(sim_time,location,orientation)
        if yaw>=360:
            yaw=0
        if yaw<=-360:
            yaw=0

    
