from ruamel.yaml import YAML
import time
import logging 
import json

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

worldsim_host="http://localhost:8080"
scenario_filename="./scenarios/wolfsburg_night.json"
ego_name="ego"

import ws_api.ws_session as ws_session_lib

api_light_states={"headlightAngleDegreesDeprecated": 0,
                "lowBeamLeft": True,
                "lowBeamRight": True,
                "highBeamLeft": True,
                "highBeamRight": True,
                "turnSignalLeft": False,
                "turnSignalRight": False,
                "fogLightLeft": False,
                "fogLightRight": False,
                "tailLightLeft": True,
                "tailLightRight": True,
                "brakeLightCenter": False,
                "brakeLightLeft": False,
                "brakeLightRight": False,
                "reverseLamps": False,
                "parkingLamps": False,
                "daytimeRunningLamps": True,
                "light_pitch":0,
                "light_yaw":0}
fps = 120
frame_time = 1.0 / fps

wss=ws_session_lib.worldsim_session(worldsim_host,fps)
wss.run_worldsim()
wss.reset()
# Request a configuration change to some agreed upon version and set the simulation to advance without
# this client directly driving time advance
# - user_vehicle_kinematic - True if we provide state, False if we provide actuation and the physics engine computes state
# - async_reporting means the simulation will advance time on its own and broadcast state regularly
config = { 'version': { 'major': 2025 }, 'user_vehicle_kinematic': False, 'async_reporting': True, 'frame_rate': fps }
wss.set_config(config)  

# Load the specified scenario

print("Loading WorldSim Scenario '%s'" % scenario_filename)
with open(scenario_filename, 'rb') as scenario_file:
    scenario = json.load(scenario_file)
    wss.scenario_load_json(scenario)

# We want to take over control over the ego vehicle and use throttle-brake-steering
ego_config = { 'name': 'ego', 'kinematic': False }
wss.set_user_controlled_agent(ego_config)
ego_agent = wss.get_user_controlled_agent(ego_config['name'])
actuation_endpoint = ego_agent['endpoint']
ego_uuid = ego_agent['uuid']
print (ego_uuid)
# Connect to the vehicle's actuation socket to take control
controller = ws_session_lib.VehicleControllerWithLights(actuation_endpoint, ego_uuid)
print (controller.uuid_a,controller.uuid_b, controller.uuid_c,controller.uuid_d)

# Set up something to read back resulting state from the system
states_endpoint = wss.get_states_endpoint()
states = ws_session_lib.StateConsumer(states_endpoint, [ego_uuid])

# The 'play' command tells WorldSim to stop waiting for any pending connections and start simulating
wss.scenario_play()

sim_time=0

throttle=0
steer=0
brake=0
controller.send(sim_time, throttle, brake, steer, api_light_states) 

lp=0
ly=-30
lyinc=0.1

while True:
    sim_time = sim_time + frame_time
    ts = states.receive()

    #steer=math.sin(sim_time * 0.5)
    #api_light_states['light_pitch']=0
    #api_light_states['light_yaw']=math.sin(sim_time) * 30.0
    api_light_states['light_pitch']=lp
    api_light_states['light_yaw']=ly

    ly=ly+lyinc
    if ly>30 and lyinc>0:
        lyinc=-lyinc
    if ly<-30 and lyinc<0:
        lyinc=-lyinc
        
    time.sleep(0.00001)
    


    steer=0
    controller.send(sim_time, throttle, brake, steer, api_light_states) 
            