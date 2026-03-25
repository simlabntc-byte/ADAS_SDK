import json
import math
import sys
import time
import numpy as np

import ws_api.ws_session as ws_session_lib

worldsim_host="http://localhost:8080"
scenario_filename="./scenarios/neighborhood_onlyped.json"
fps=60

if __name__ == "__main__":
    wss=ws_session_lib.worldsim_session(worldsim_host,fps)
    frame_time = 1.0 / fps

    # Run WorldSim if not already running
    wss.run_worldsim()
    print("Running")

    if len(sys.argv) > 1:
        scenario_filename = sys.argv[1:]

    # Request a configuration change to some agreed upon version and set the simulation to advance without
    # this client directly driving time advance
    # - user_vehicle_kinematic - True if we provide state, False if we provide actuation and the physics engine computes state
    # - async_reporting means the simulation will advance time on its own and broadcast state regularly
    config = { 'version': { 'major': 2023 }, 'user_vehicle_kinematic': False, 'async_reporting': True, 'frame_rate': fps }
    wss.set_config(config)

    # Load the specified scenario
    print("Loading WorldSim Scenario '%s'" % scenario_filename)
    with open(scenario_filename, 'rb') as scenario_file:
        scenario = json.load(scenario_file)
        wss.scenario_load_json(scenario)
    time.sleep(5)

    initial_motion_vector=np.array([1,0,0])
    x=0
    y=0
    z=0
    pitch=0.0
    roll=0.0
    yaw=0.0

    initial_location = np.array([x,y,z])
    location = np.array([x,y,z])
    orientation=ws_session_lib.toQuaternion(ws_session_lib.degrees_to_rads(pitch),ws_session_lib.degrees_to_rads(roll),ws_session_lib.degrees_to_rads(yaw),False)

    ped_config = { 'name': 'rando_ped_0', 'kinematic': False }
    wss.set_user_controlled_agent(ped_config)
    ped_agent = wss.get_user_controlled_agent(ped_config['name'])
    actuation_endpoint = ped_agent['endpoint']
    ped_uuid = ped_agent['uuid']
    print(ped_uuid)
    ped_controller = ws_session_lib.PedestrianController(actuation_endpoint, ped_uuid)

    print("Ready to play!")

    wss.scenario_play()
    sim_time = 0

    while True: 
        ped_controller.send(sim_time,location,orientation)
        sim_time = sim_time + frame_time
        time.sleep(frame_time)

        f=0.1
        amp=1.5
        x=x+0.02
        y = amp*np.sin(2*np.pi*f * x)

        location = np.array([x,y,z])
        dx=location[0]-initial_location[0]
        dy=location[1]-initial_location[1]
        yaw=math.tan(dy/dx)

        orientation=ws_session_lib.toQuaternion(ws_session_lib.degrees_to_rads(pitch),ws_session_lib.degrees_to_rads(roll),(yaw),False)
        initial_location=location

        print (x,y,yaw)

    del ped_controller
