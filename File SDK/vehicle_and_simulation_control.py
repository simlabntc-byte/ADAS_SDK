import ws_api.ws_session as ws_session_lib
import numpy as np
import pygame
from ruamel.yaml import YAML
from pathlib import Path
import time

worldsim_host="http://localhost:8080"
scenario_filename="./scenarios/wolfsburg_day.json"
fps=60

# Define Vehicle and Simulation Control 
kinematic_control=False
stepped_sim=True

# Read Config File
configfile=Path("config.yaml")
_config = YAML(typ='safe').load(configfile)
control_device=_config['sim']['default_control']
print (f"using {control_device} for input")

class HMI_controller ():
    def __init__(self):
        self.steer=0
        self.brake=0
        self.throttle=0
        self.handbrake=0
        try:
            # initialize steering wheel
            pygame.joystick.init()

            joystick_count = pygame.joystick.get_count()
            print (f"Found {joystick_count} connected devices")
            #if joystick_count > 1:
            #    raise ValueError("Please Connect one Joystick")
            if joystick_count ==0 :
                raise ValueError("No joystick connected")

            if control_device=='fanatec':
                self._joystick = pygame.joystick.Joystick(1)
            else:
                self._joystick = pygame.joystick.Joystick(0)

            self._joystick.init()
            self._steer_idx = _config['sim']['controls'][control_device]['steering_wheel']
            self._throttle_idx = _config['sim']['controls'][control_device]['throttle']
            self._brake_idx = _config['sim']['controls'][control_device]['brake']
            self._reverse_idx = _config['sim']['controls'][control_device]['reverse']
            self._handbrake_idx = _config['sim']['controls'][control_device]['handbrake']
        except Exception as e: 
            print(e)
            print('Shutting Down')
            pygame.quit()
            print ("Done")
            exit (0)


    def parse_HMI_inputs(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        jsButtons = [float(self._joystick.get_button(i)) for i in
                        range(self._joystick.get_numbuttons())]

        # Invert Control Signals if needed
        if control_device=='fanatec' or control_device=='xbox360':
            ic=-1
        else:
            ic=1

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
    
        steerCmd = ic * jsInputs[self._steer_idx]
        steer_dead_zone=_config['worldsim']['steering_dead_zone']
        if (steerCmd>=-1*steer_dead_zone and steerCmd<=steer_dead_zone):
            steerCmd=0

        throttleCmd = ic * jsInputs[self._throttle_idx]
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1
        throttleCmd=(abs(1-throttleCmd))

        brakeCmd = ic * jsInputs[self._brake_idx]
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1
        brakeCmd=(abs(1-brakeCmd))

        self.steer = steerCmd
        self.brake = brakeCmd
        self.throttle = throttleCmd

        self.handbrake = bool(jsButtons[self._handbrake_idx])

wss=ws_session_lib.worldsim_session(worldsim_host,fps)

#wss.run_worldsim()
if stepped_sim:
    wss.run_worldsim_deterministic()
else:
    wss.run_worldsim()
#wss.run_worldsim_distributed(sensor_node=True)

# Optional (if running again without restarting WorldSim)
wss.reset()

# Request a configuration change to some agreed upon version and set the simulation to advance without
# this client directly driving time advance
# - user_vehicle_kinematic - True if we provide state, False if we provide actuation and the physics engine computes state
# - async_reporting means the simulation will advance time on its own and broadcast state regularly
if stepped_sim:
    if kinematic_control:
        config = { 'version': { 'major': 2023 }, 'async_reporting': False, 'frame_rate': fps,  "user_vehicle_kinematic":True, "user_vehicle_snap_to_ground": True }
    else:
        config = { 'version': { 'major': 2023 },  'async_reporting': False, 'frame_rate': fps, "user_vehicle_kinematic": False, "user_vehicle_snap_to_ground": True  }
else:
    if kinematic_control:
        config = { 'version': { 'major': 2023 }, 'async_reporting': True, 'frame_rate': fps,  "user_vehicle_kinematic":True,"user_vehicle_snap_to_ground": True }
    else:
        config = { 'version': { 'major': 2023 },  'async_reporting': True, 'frame_rate': fps, "user_vehicle_kinematic": False, "user_vehicle_snap_to_ground": True  }    
wss.set_config(config)

# Read back the final config
config = wss.get_config()
print (config)
if stepped_sim:
    sim_control_endpoint = config['sim_control_endpoint']
    if len(sim_control_endpoint) == 0:
        raise Exception("No sim control endpoint!")
else:
    sim_control_endpoint=None

wss.load_scenario_file(scenario_filename)

if kinematic_control:
    ego_config = { 'name': 'ego', 'kinematic': True }
else:
    ego_config = { 'name': 'ego', 'kinematic': False }
wss.set_user_controlled_agent(ego_config)

ego_agent = wss.get_user_controlled_agent(ego_config['name'])
actuation_endpoint = ego_agent['endpoint']
ego_uuid = ego_agent['uuid']

if stepped_sim:
    sim_control = ws_session_lib.SimController(sim_control_endpoint)
else:
    sim_control=None

# Connect to the vehicle's actuation socket to take control
vehicle_controller = ws_session_lib.VehicleController(actuation_endpoint, ego_uuid)

# Set up something to read back resulting state from the system
states_endpoint = wss.get_states_endpoint()
states = ws_session_lib.StateConsumer(states_endpoint, [ego_uuid])

# The 'play' command tells WorldSim to stop waiting for any pending connections and start simulating
wss.scenario_play()


sim_time=0
frame_time=1/fps

# Main simulation loop
if kinematic_control:
    print ("Running Under Kinematic Control")
    initial_motion_vector=np.array([1,0,0])
    x=1216.661
    y=135.213
    z=11.722
    pitch=0.0
    roll=0.0
    yaw=0.0
    ts_disp=0
    linear_velocity=0.1
    angular_velocity=1

    location = np.array([x,y,z])
    orientation=ws_session_lib.toQuaternion(ws_session_lib.degrees_to_rads(pitch),ws_session_lib.degrees_to_rads(roll),ws_session_lib.degrees_to_rads(yaw),False)

    # initialize pygame controller
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

    # TODO - Turn off vehicle interior and wheels views
    # Sending keys to window to turn off vehicle interior
    #newevent = pygame.event.Event(pygame.KEYDOWN, unicode=".", key=pygame.K_PERIOD, mod=pygame.KMOD_NONE) #create the event
    #pygame.event.post(newevent) #add the event to the queue
    vehicle_controller.send_KinematicActuationCommand(sim_time,location,orientation)

else:
    print ("Running Under Physics Control")
    pygame.init()
    my_input_controller=HMI_controller()
    vehicle_controller.send_PhysicsVehicleActuationCommand(0.0, 0.0, 0.0, 0.0)

if stepped_sim:
    print ("Running in stepped mode")
else:
    print (print ("Running in async mode"))


# Main Loop
while True:
    if stepped_sim:
        sim_time = sim_control.advance_to_next_step()
    else:
        sim_time = sim_time + frame_time
    states.receive()


    if kinematic_control:
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
        if yaw>=360:
            yaw=0
        if yaw<=-360:
            yaw=0
        motion_vector=ws_session_lib.rotate_vector(initial_motion_vector,ws_session_lib.degrees_to_rads(yaw),[0,0,1])
        uv = motion_vector / np.linalg.norm(motion_vector)
        location = location + ts_disp * uv
        orientation=ws_session_lib.toQuaternion(ws_session_lib.degrees_to_rads(pitch),ws_session_lib.degrees_to_rads(roll),ws_session_lib.degrees_to_rads(yaw),False)
        vehicle_controller.send_KinematicActuationCommand(sim_time,location,orientation)
        print (f"X: {location[0]}, Y: {location[1]}, Z: {location[2]}, Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}, ") 
        print (sim_time,states.time)


    else:
        pygame.event.get()
        my_input_controller.parse_HMI_inputs()
        vehicle_controller.send_PhysicsVehicleActuationCommand(sim_time, my_input_controller.steer, my_input_controller.throttle, my_input_controller.brake)

        #print (f"{sim_time}, {states.time}, Steer: {my_input_controller.steer}, Throttle: {my_input_controller.throttle}, Brake: {my_input_controller.brake}")
        print (f"Steer: {my_input_controller.steer}, Throttle: {my_input_controller.throttle}, Brake: {my_input_controller.brake}")      
        print (sim_time,states.time)

        if not stepped_sim:
            time.sleep(0.01) # slow loop down so we are not hammering the vehicle controller with to many inputs demands

