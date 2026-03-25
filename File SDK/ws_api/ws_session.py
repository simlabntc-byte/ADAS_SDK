import math
import numpy as np
import winreg
import subprocess
from urllib import parse, request
import time
import json
import requests
import zmq
import flatbuffers
import RHMsg.GroupedStateMeasurement
import RHMsg.HeaderData
import RHMsg.KinematicStateMeasurement
import RHMsg.KinematicActuationCommand
import RHMsg.PhysicsVehicleActuationCommand
import RHMsg.Source
import RHMsg.UUID
import RHMsg.Vec3 as vec3
import RHMsg.Quat4 as quat4
import RHMsg.PedestrianActuationCommand
import RHMsg.ProxyToControlRequest
import RHMsg.ProxyToControlRequestType
import RHMsg.ControlToProxyReply
import RHMsg.ControlToProxyReplyType
import RHMsg.VehicleLights
import RHMsg.AdvancedVehicleLights


exereg=r"SOFTWARE\VI-grade\VI-WorldSim\2025"

class worldsim_session:
    def __init__(self, host,fps=60):
        self.worldsim_host = host
        self.sessionrunning=False
        self.fps=fps

    # Start WorldSim StandAlone Mode
    def run_worldsim(self):
        print("Running WorldSim - StandAlone")
        registry = winreg.ConnectRegistry(None, winreg.HKEY_LOCAL_MACHINE)
        reg_key = winreg.OpenKey(registry, exereg)
        reg_val = winreg.QueryValueEx(reg_key, "InstallDir")
        worldsim_exe = reg_val[0] + "\WorldSim.exe"
        print(worldsim_exe)
        subprocess.Popen([worldsim_exe, "-NodeName=StandAlone", "-windowed", f"-fps={self.fps}", "-focusedAgent=ego"])
        self.wait_for_worldsim()

    # Start WorldSim in Distributed Mode
    def run_worldsim_distributed(self, sensor_node=False):
        print("Running WorldSim - Distributed")
        registry = winreg.ConnectRegistry(None, winreg.HKEY_LOCAL_MACHINE)
        reg_key = winreg.OpenKey(registry, exereg)
        reg_val = winreg.QueryValueEx(reg_key, "InstallDir")
        worldsim_exe = reg_val[0] + "\WorldSim.exe"
        print(worldsim_exe)
        subprocess.Popen([worldsim_exe,"-ServiceAddress=127.0.0.1", "-NodeName=Traffic", f"-fps={self.fps}", "-windowed", "-focusedAgent=ego", "-nullrhi", "-log"])
        subprocess.Popen([worldsim_exe,"-ServiceAddress=127.0.0.1", "-NodeName=Camera", f"-fps={self.fps}", "-windowed", "-wsdisplay", "-log"])
        if sensor_node:
            print ("Starting WorldSim Sensor Node")
            subprocess.Popen([worldsim_exe,"-ServiceAddress=127.0.0.1", "-NodeName=Sensor", "-fps=30",  "-windowed", "-minimized", "-log"])
        self.wait_for_worldsim()


    # Start WorldSim with '-deterministic' to run stepped mode, which enables the sim control endpoint
    def run_worldsim_deterministic(self):
        print("Running WorldSim - Deterministic")
        registry = winreg.ConnectRegistry(None, winreg.HKEY_LOCAL_MACHINE)
        reg_key = winreg.OpenKey(registry, r"SOFTWARE\VI-grade\VI-WorldSim\2025")
        reg_val = winreg.QueryValueEx(reg_key, "InstallDir")
        worldsim_exe = reg_val[0] + "\WorldSim.exe"
        print(worldsim_exe)
        subprocess.Popen([worldsim_exe, "-windowed",  "-deterministic", f"-fps={self.fps}", "user_vehicle_snap_to_ground=True",  "-log"])
        self.wait_for_worldsim()

    def connect_to_worldim (self):
        self.wait_for_worldsim()
        config=self.get_config()
        self.fps=config['frame_rate']


    def wait_for_worldsim (self):
        # Wait until the system is up before returning
        req = request.Request(url=self.worldsim_host + "/worldsim/status")
        while True:
            try:
                with request.urlopen(req) as f:
                    if f.status == 200:
                        print ("Worldsim started - waiting for sync")
                        time.sleep(5)
                        self.sessionrunning=True
                        print ("Worldsim started")
                        print ("Press '.' to toggle vehicle interior, '/' to toggle wheels")
                        return
            except:
                print ("waiting for worldsim")
                pass

    # Load Scenario JSON into running worldsim session
    def load_scenario_file(self, ws_scenario_filename):
        if self.sessionrunning:
            print("Loading WorldSim Scenario '%s'" % ws_scenario_filename)
            with open(ws_scenario_filename, 'rb') as scenario_file:
                scenario = json.load(scenario_file)
                self.scenario_load_json(scenario)
        else:
            print (f"No valid WorldSim session is running on {self.worldsim_host}")
            exit()


    def scenario_load_json(self, scenario):
        """Sends scenario JSON to be loaded"""
        json_data = json.JSONEncoder().encode(scenario)
        put_data = json_data.encode('utf-8')
        req = request.Request(url = self.worldsim_host + "/worldsim/scenario/load", data=put_data, method='PUT')
        req.add_header('Content-Length', len(put_data))
        req.add_header('Content-Type', 'application/json')
        request.urlopen(req)

    def get_scenario(self):
        scenario_status_url=f"{self.worldsim_host}/worldsim/scenario"
        print (scenario_status_url)
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()
        return scenario_dict

    def reset(self):
        """Unloads the scenario for a blank slate without restarting the executable"""
        print("Clearing simulation environment")
        req = request.Request(url=self.worldsim_host + "/worldsim/scenario/stop")
        request.urlopen(req)


    def set_config(self, config):
        """Replaces the current configuration with a new one"""
        json_data = json.JSONEncoder().encode(config)
        put_data = json_data.encode('utf-8')
        req = request.Request(url=self.worldsim_host + "/worldsim/config/sim", data=put_data, method='PUT')
        req.add_header('Content-Length', len(put_data))
        req.add_header('Content-Type', 'application/json')
        request.urlopen(req)


    def get_config(self):
        """Fetches the current simulation configuration"""
        req = request.Request(url=self.worldsim_host + "/worldsim/config/sim")
        with request.urlopen(req) as f:
            body = f.read().decode('utf-8')
            config = json.JSONDecoder().decode(body)
            return config


    def set_user_controlled_agent(self, config):
        """Tells the simulator we want to control a particular vehicle"""
        json_data = json.JSONEncoder().encode(config)
        put_data = json_data.encode('utf-8')
        #req = request.Request(url=worldsim_host + "/worldsim/config/controlled_agents/" + config['name'], data=put_data, method='PUT')
        #req.add_header('Content-Length', len(put_data))
        #req.add_header('Content-Type', 'application/json')
        #request.urlopen(req)

        headers={'Content-Length': f'{len(put_data)}','Content-Type': 'application/json'}
        async_req = requests.put(url=self.worldsim_host + "/worldsim/config/controlled_agents/" + config['name'], data=put_data,headers=headers)


    def get_user_controlled_agent(self, name):
        """Fetches the endpoint for ego vehicle actuation """
        req = request.Request(url=self.worldsim_host + "/worldsim/config/controlled_agents/" + name)
        with request.urlopen(req) as f:
            body = f.read().decode('utf-8')
            config = json.JSONDecoder().decode(body)
            return config


    def get_user_controlled_agents(self):
        """Fetches the endpoint for ego vehicle actuation """
        req = request.Request(url=self.worldsim_host + "/worldsim/config/controlled_agents/")
        with request.urlopen(req) as f:
            body = f.read().decode('utf-8')
            agents_dict = json.JSONDecoder().decode(body)
            return agents_dict


    def get_ego_vehicle_uuid_str(self):
        """Parses the scenario and extracts the UUID for the ego vehicle"""
        req = request.Request(url=self.worldsim_host + "/worldsim/scenario")
        with request.urlopen(req) as f:
            body = f.read().decode('utf-8')
            scenario = json.JSONDecoder().decode(body)
            vehicles = scenario['actors']['vehicles']
            ego_vehicle = [vehicle for vehicle in vehicles if vehicle.get('name') == 'ego'][0]
            return ego_vehicle['uuid']


    def get_states_endpoint(self):
        """Gets the endpoint used to retrieve simulation state"""
        req = request.Request(url=self.worldsim_host + "/worldsim/scenario")
        with request.urlopen(req) as f:
            body = f.read().decode('utf-8')
            scenario = json.JSONDecoder().decode(body)
            # middleware sockets are for SDK use
            return scenario['implementation']['state_endpoints']['middleware']


    def scenario_play(self):
        """Signals we are done configuring and ready for time to start advancing

        Up until this point there can be ports opened and waiting for SDK input. If they are unused
        at this stage they will be closed and default behavior used instead.
        """
        req = request.Request(url=self.worldsim_host + "/worldsim/scenario/play", method='GET')
        request.urlopen(req)



    # Misc rest API endpoints


    def get_assets(self):
        scenario_status_url=f"{self.worldsim_host}/worldsim/assets"
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()
        return scenario_dict

    def get_vehicles(self):
        scenario_status_url=f"{self.worldsim_host}/worldsim/assets/vehicles"
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()
        return scenario_dict

    def get_dashboard_config(self):
        scenario_status_url=f"{self.worldsim_host}/worldsim/config/dashboards/ego"
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()
        return scenario_dict
    
    def set_dashboard(self, dashconfig):
        #dashconfig={
        #        "Cluster": "http://www.google.com",
        #        "Infotainment": "https://www.google.com/maps/@42.6197113,-83.3193236,1146m/data=!3m1!1e3?entry=ttu&g_ep=EgoyMDI0MTEwNi4wIKXMDSoASAFQAw%3D%3D",
        #        "HUD": ""
        #        }
        json_data = json.JSONEncoder().encode(dashconfig)
        put_data = json_data.encode('utf-8')
        req = request.Request(url=self.worldsim_host + "/worldsim/config/dashboards/ego", data=put_data, method='PUT')
        req.add_header('Content-Length', len(put_data))
        req.add_header('Content-Type', 'application/json')
        request.urlopen(req)

    def get_info(self):
        scenario_status_url=f"{self.worldsim_host}/worldsim/describe/info"
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()
        return scenario_dict

    def get_available_tracks(self):
        track_list=[]
        config=self.get_info()
        for item in config:
            if "track:" in item:
                track_list.append(item.split(':')[1])
        return track_list

    def get_paths_geojson(self,level):
        data_url=f"{self.worldsim_host}/worldsim/describe/track/track:{level}/paths.geojson"
        resp = requests.get(data_url)
        scenario_dict = resp.json()
        return scenario_dict
    
    def get_base_prj(self,level):
        data_url=f"{self.worldsim_host}/worldsim/describe/track/track:{level}/base.prj"
        resp = requests.get(data_url)
        return resp.content.decode('ascii')

    def get_environment(self):
        data_url=f"{self.worldsim_host}/worldsim/scenario/environment"
        resp = requests.get(data_url)
        scenario_dict = resp.json()
        return scenario_dict

    def get_vehicles_in_scenario(self):
        scenario=self.get_scenario()
        vehicle_list=scenario['actors']['vehicles']
        vehicle_types=[]
        for vehicle in vehicle_list:
            vehicle_types.append(vehicle['asset']['class_name'])
        return vehicle_types

    def get_vehicle_uuid_str(self,vehicle_name):
        """Parses the scenario and extracts the UUID for the ego vehicle"""
        url=self.worldsim_host + "/worldsim/scenario"
        resp = requests.get(url)
        scenario = resp.json()
        vehicles = scenario['actors']['vehicles']
        vehicle = [vehicle for vehicle in vehicles if vehicle.get('name') == vehicle_name][0]
        return vehicle['uuid']


class VehicleController:
    def __init__(self, endpoint, uuid_str):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB) # We are the publishing side of a PUB-SUB pair
        self.socket.setsockopt(zmq.SNDTIMEO, 1000)
        self.socket.bind(endpoint) # Listen on the port
        self.seq = 0

        # The RightHook wire UUID format is a little awkward to do as the flatbuffer API expects it
        # to be split into chunks.
        #
        # In string format you'd have: "7972ADC5-4CAD-4863-BCF1-86AC8B9F2F69"
        #
        # But it should be split into integers for RHMsg:
        # A = 7972ADC5
        # B = 4CAD4863
        # C = BCF186AC
        # D = 8B9F2F69
        #
        uuid_parts = uuid_str.split('-')
        self.uuid_a = int(uuid_parts[0][:8], 16)
        self.uuid_b = int(uuid_parts[1][:4] + uuid_parts[2][:4], 16)
        self.uuid_c = int(uuid_parts[3][:4] + uuid_parts[4][:4], 16)
        self.uuid_d = int(uuid_parts[4][4:], 16)

    def send_KinematicActuationCommand(self,sim_time, location, orientation):
        fbb = flatbuffers.Builder(0)
        
        # Message header with general timing info
        RHMsg.HeaderData.HeaderDataStart(fbb)
        RHMsg.HeaderData.HeaderDataAddSequenceNo(fbb, self.seq)
        RHMsg.HeaderData.HeaderDataAddSource(fbb, RHMsg.Source.Source.KINEMATIC_STATE) # Kinematics Actuation
        RHMsg.HeaderData.HeaderDataAddTimestamp(fbb, sim_time)
        header = RHMsg.HeaderData.HeaderDataEnd(fbb)

        # Message payload with actual actuation packet
        RHMsg.KinematicActuationCommand.KinematicActuationCommandStart(fbb)
        RHMsg.KinematicActuationCommand.KinematicActuationCommandAddHeader(fbb, header)
        RHMsg.KinematicActuationCommand.KinematicActuationCommandAddUuid(fbb, RHMsg.UUID.CreateUUID(fbb, self.uuid_a, self.uuid_b, self.uuid_c, self.uuid_d))
        RHMsg.KinematicActuationCommand.KinematicActuationCommandAddLocation(fbb, vec3.CreateVec3(fbb,location[0],location[1],location[2]))
        RHMsg.KinematicActuationCommand.KinematicActuationCommandAddOrientation(fbb, quat4.CreateQuat4(fbb,orientation.x,orientation.y,orientation.z,orientation.w))
        # next line needed to add wheel rotation
        RHMsg.KinematicActuationCommand.KinematicActuationCommandAddVelocity(fbb, vec3.CreateVec3(fbb,5,5,0))

        message = RHMsg.KinematicActuationCommand.KinematicActuationCommandEnd(fbb)
        fbb.Finish(message)

        buffer = bytes(fbb.Output())
        self.socket.send(buffer)

        self.seq = self.seq + 1

    def send_PhysicsVehicleActuationCommand(self, sim_time, steer, throttle, brake):
        fbb = flatbuffers.Builder(0)

        # Message header with general timing info
        RHMsg.HeaderData.HeaderDataStart(fbb)
        RHMsg.HeaderData.HeaderDataAddSequenceNo(fbb, self.seq)
        RHMsg.HeaderData.HeaderDataAddSource(fbb, RHMsg.Source.Source.PHYSICS_VEHICLE_ACTUATION) # Actuation is throttle/brake/steering
        RHMsg.HeaderData.HeaderDataAddTimestamp(fbb, sim_time)
        header = RHMsg.HeaderData.HeaderDataEnd(fbb)

        # Message payload with actual actuation packet
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandStart(fbb)
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddHeader(fbb, header)
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddUuid(fbb, RHMsg.UUID.CreateUUID(fbb, self.uuid_a, self.uuid_b, self.uuid_c, self.uuid_d))
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddSteeringDegrees(fbb, steer * 20) 
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddNormalizedThrottle(fbb, throttle)
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddNormalizedBrake(fbb,brake)
        #RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddSteeringDegrees(fbb, math.sin(sim_time * 0.5) * 20) # Swerve back and forth
        #RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddNormalizedThrottle(fbb, 0.25)
        #RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddNormalizedBrake(fbb,0)
        message = RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandEnd(fbb)
        fbb.Finish(message)

        buffer = bytes(fbb.Output())
        self.socket.send(buffer)

        self.seq = self.seq + 1

class VehicleControllerWithLights:
    def __init__(self, endpoint, uuid_str):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB) # We are the publishing side of a PUB-SUB pair
        self.socket.setsockopt(zmq.CONFLATE, 1) # Get the latest message
        self.socket.setsockopt(zmq.SNDTIMEO, 1000)
        self.socket.bind(endpoint) # Listen on the port
        self.seq = 0

        # The RightHook wire UUID format is a little awkward to do as the flatbuffer API expects it
        # to be split into chunks.
        #
        # In string format you'd have: "7972ADC5-4CAD-4863-BCF1-86AC8B9F2F69"
        #
        # But it should be split into integers for RHMsg:
        # A = 7972ADC5
        # B = 4CAD4863
        # C = BCF186AC
        # D = 8B9F2F69
        #
        uuid_parts = uuid_str.split('-')
        self.uuid_a = int(uuid_parts[0][:8], 16)
        self.uuid_b = int(uuid_parts[1][:4] + uuid_parts[2][:4], 16)
        self.uuid_c = int(uuid_parts[3][:4] + uuid_parts[4][:4], 16)
        self.uuid_d = int(uuid_parts[4][4:], 16)

        
    def send(self, sim_time, throttle, brake, steer, light_states):
        fbb = flatbuffers.Builder(0)

        # Message header with general timing info
        RHMsg.HeaderData.HeaderDataStart(fbb)
        RHMsg.HeaderData.HeaderDataAddSequenceNo(fbb, self.seq)
        RHMsg.HeaderData.HeaderDataAddSource(fbb, RHMsg.Source.Source.PHYSICS_VEHICLE_ACTUATION) # Actuation is throttle/brake/steering
        RHMsg.HeaderData.HeaderDataAddTimestamp(fbb, sim_time)
        header = RHMsg.HeaderData.HeaderDataEnd(fbb)

        #headlight_pitch=0
        #headlight_yaw=math.sin(sim_time) * 35.0
        headlight_pitch=light_states['light_pitch']
        headlight_yaw=light_states['light_yaw']
        print (sim_time,"headligh yaw=",headlight_yaw)
        RHMsg.AdvancedVehicleLights.AdvancedVehicleLightsStart(fbb)
        RHMsg.AdvancedVehicleLights.AdvancedVehicleLightsAddHighBeamLeftPitchDegrees(fbb, headlight_pitch)
        RHMsg.AdvancedVehicleLights.AdvancedVehicleLightsAddHighBeamLeftYawDegrees(fbb, headlight_yaw)
        RHMsg.AdvancedVehicleLights.AdvancedVehicleLightsAddHighBeamRightPitchDegrees(fbb, headlight_pitch)
        RHMsg.AdvancedVehicleLights.AdvancedVehicleLightsAddHighBeamRightYawDegrees(fbb, headlight_yaw)
        RHMsg.AdvancedVehicleLights.AdvancedVehicleLightsAddLowBeamLeftPitchDegrees(fbb, headlight_pitch)
        RHMsg.AdvancedVehicleLights.AdvancedVehicleLightsAddLowBeamLeftYawDegrees(fbb, headlight_yaw)
        RHMsg.AdvancedVehicleLights.AdvancedVehicleLightsAddLowBeamRightPitchDegrees(fbb, headlight_pitch)
        RHMsg.AdvancedVehicleLights.AdvancedVehicleLightsAddLowBeamRightYawDegrees(fbb, headlight_yaw)
        adv_lights=RHMsg.AdvancedVehicleLights.AdvancedVehicleLightsEnd(fbb)


        # Message payload with actual actuation packet
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandStart(fbb)
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddHeader(fbb, header)
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddUuid(fbb, RHMsg.UUID.CreateUUID(fbb, self.uuid_a, self.uuid_b, self.uuid_c, self.uuid_d))
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddSteeringDegrees(fbb, steer * 20) # Swerve back and forth
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddNormalizedThrottle(fbb, throttle)
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddNormalizedBrake(fbb,brake)
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddLights(fbb,RHMsg.VehicleLights.CreateVehicleLights(fbb,
                                                                                        headlightAngleDegreesDeprecated=0,
                                                                                        lowBeamLeft=light_states['lowBeamLeft'],
                                                                                        lowBeamRight=light_states['lowBeamRight'],
                                                                                        highBeamLeft=light_states['highBeamLeft'],
                                                                                        highBeamRight=light_states['highBeamRight'],
                                                                                        turnSignalLeft=light_states['turnSignalLeft'],
                                                                                        turnSignalRight=light_states['turnSignalRight'],
                                                                                        fogLightLeft=light_states['fogLightLeft'],
                                                                                        fogLightRight=light_states['fogLightRight'],
                                                                                        tailLightLeft=light_states['tailLightLeft'],
                                                                                        tailLightRight=light_states['tailLightRight'],
                                                                                        brakeLightCenter=light_states['brakeLightCenter'],
                                                                                        brakeLightLeft=light_states['brakeLightLeft'],
                                                                                        brakeLightRight=light_states['brakeLightRight'],
                                                                                        reverseLamps=light_states['reverseLamps'],
                                                                                        parkingLamps=light_states['parkingLamps'],
                                                                                        daytimeRunningLamps=light_states['daytimeRunningLamps']))
        RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandAddAdvancedLights(fbb, adv_lights)  
             
        message = RHMsg.PhysicsVehicleActuationCommand.PhysicsVehicleActuationCommandEnd(fbb)

        fbb.Finish(message)
        buffer = bytes(fbb.Output())
        self.socket.send(buffer)
        self.seq = self.seq + 1



# Consumes the resulting state from the system
class StateConsumer:
    def __init__(self, endpoint, uuids):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(endpoint)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, '') # Remove any filtering of messages, receive everything
        self.socket.setsockopt(zmq.RCVTIMEO, 0) # Don't block if there aren't any state updates
        self.uuids = uuids
        self.time = -1.0

    def receive(self):
        try:
            buffer = self.socket.recv()
            if buffer is not None:
                states_msg = RHMsg.GroupedStateMeasurement.GroupedStateMeasurement.GetRootAsGroupedStateMeasurement(buffer, 0)
                # TODO: Implement whatever state  handling you need here, similar to ExampleSubscriber.py
                self.time = states_msg.Header().Timestamp()
            
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                # Not really an error, just an empty message
                pass
            else:
                raise





# drive a pedestrian
class PedestrianController:
    def __init__(self, endpoint, uuid_str):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB) # We are the publishing side of a PUB-SUB pair
        self.socket.setsockopt(zmq.SNDTIMEO, 1000)
        self.socket.bind(endpoint) # Listen on the port
        self.seq = 0

        # The RightHook wire UUID format is a little awkward to do as the flatbuffer API expects it
        # to be split into chunks.
        #
        # In string format you'd have: "7972ADC5-4CAD-4863-BCF1-86AC8B9F2F69"
        #
        # But it should be split into integers for RHMsg:
        # A = 7972ADC5
        # B = 4CAD4863
        # C = BCF186AC
        # D = 8B9F2F69
        #
        uuid_parts = uuid_str.split('-')
        self.uuid_a = int(uuid_parts[0][:8], 16)
        self.uuid_b = int(uuid_parts[1][:4] + uuid_parts[2][:4], 16)
        self.uuid_c = int(uuid_parts[3][:4] + uuid_parts[4][:4], 16)
        self.uuid_d = int(uuid_parts[4][4:], 16)

    def __del__(self):
        print("closing socket")
        self.context.destroy()        

    def send(self, sim_time , location, orientation, vel_x=1):
        fbb = flatbuffers.Builder(0)

        # Message header with general timing info
        RHMsg.HeaderData.HeaderDataStart(fbb)
        RHMsg.HeaderData.HeaderDataAddSequenceNo(fbb, self.seq)
        self.seq = self.seq + 1
        RHMsg.HeaderData.HeaderDataAddSource(fbb, RHMsg.Source.Source.PEDESTRIAN_ACTUATION) # Actuation is?
        RHMsg.HeaderData.HeaderDataAddTimestamp(fbb, sim_time)
        header = RHMsg.HeaderData.HeaderDataEnd(fbb)

        # Message payload with actual actuation packet
        RHMsg.PedestrianActuationCommand.PedestrianActuationCommandStart(fbb)
        RHMsg.PedestrianActuationCommand.PedestrianActuationCommandAddHeader(fbb, header)
        RHMsg.PedestrianActuationCommand.PedestrianActuationCommandAddUuid(fbb, RHMsg.UUID.CreateUUID(fbb, self.uuid_a, self.uuid_b, self.uuid_c, self.uuid_d))
        RHMsg.PedestrianActuationCommand.PedestrianActuationCommandAddLocation(fbb, vec3.CreateVec3(fbb,location[0],location[1],location[2]))
        RHMsg.PedestrianActuationCommand.PedestrianActuationCommandAddOrientation(fbb,quat4.CreateQuat4(fbb,orientation.x,orientation.y,orientation.z,orientation.w))
        RHMsg.PedestrianActuationCommand.PedestrianActuationCommandAddVelocity(fbb, vec3.CreateVec3(fbb, vel_x, 0.0, 0.0))
        RHMsg.PedestrianActuationCommand.PedestrianActuationCommandAddAngularVelocity(fbb, vec3.CreateVec3(fbb, 0.0, 0.0, 0.0))
        message = RHMsg.PedestrianActuationCommand.PedestrianActuationCommandEnd(fbb)
        fbb.Finish(message)

        buffer = bytes(fbb.Output())
        self.socket.send(buffer)

        self.seq = self.seq + 1





class SimController:
    def __init__(self, endpoint):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.SNDTIMEO, 1000)
        self.socket.connect(endpoint) # Listen on the port
        self.sim_time = 0.0
        self.sim_time_advanced = 0.0

    def advance_to_next_step(self):
        # States coming in were for one step back in time
        self.sim_time = self.sim_time_advanced

        if self.send_advance_request():
            buffer = self.socket.recv()
            if buffer is not None:
                message = RHMsg.ControlToProxyReply.ControlToProxyReply.GetRootAsControlToProxyReply(buffer, 0)
                if message.Reply() == RHMsg.ControlToProxyReplyType.ControlToProxyReplyType.DONE:
                    self.sim_time_advanced = message.Time()
        return self.sim_time

    def send_advance_request(self):
        fbb = flatbuffers.Builder(0)
        RHMsg.ProxyToControlRequest.ProxyToControlRequestStart(fbb)
        RHMsg.ProxyToControlRequest.ProxyToControlRequestAddRequest(fbb, RHMsg.ProxyToControlRequestType.ProxyToControlRequestType.ADVANCE)
        message = RHMsg.ProxyToControlRequest.ProxyToControlRequestEnd(fbb)
        fbb.Finish(message)
        buffer = bytes(fbb.Output())
        try:
            self.socket.send(buffer)
            return True
        except:
            return False

class worldsim_sensor:
    def __init__(self, host, veh_name, sensor_name):
        self.worldsim_host = host
        self.veh_name=veh_name
        self.sensor_name=sensor_name


    def get_sensor_endpoint_by_name(self):
        endpoint=""
        scenario_status_url=f"{self.worldsim_host}/worldsim/scenario"
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()

        ego_vehicle=None
        actors_info=scenario_dict['actors']['vehicles']
        for actor in actors_info:
            print (actor['name'])
            if actor['name']==self.veh_name:
                print ('ego vehicle found')
                ego_vehicle=actor
                break
        if ego_vehicle==None:
            print (f"No actor named '{self.veh_name}' found")
            exit(1)
        sensors=ego_vehicle['sensors']
        print (sensors)
        for sensor in sensors:
            if len(sensors[sensor])>0:
                for sensor_type in sensors[sensor]:
                    if sensor_type["name"]==self.sensor_name:
                        endpoint=sensor_type['implementation']['endpoints']['middleware']

        endpoint_bits=endpoint.split(":")
        tcp=endpoint_bits[0]+endpoint_bits[1]
        sensor_port=endpoint_bits[2]
        return endpoint, tcp, sensor_port

    def get_sensor_config_by_name(self):
        sensor_config=""
        scenario_status_url=f"{self.worldsim_host}/worldsim/scenario"
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()

        ego_vehicle=None
        actors_info=scenario_dict['actors']['vehicles']
        for actor in actors_info:
            print (actor['name'])
            if actor['name']==self.veh_name:
                print ('ego vehicle found')
                ego_vehicle=actor
                break
        if ego_vehicle==None:
            print (f"No actor named '{self.veh_name}' found")
            exit(1)
        sensors=ego_vehicle['sensors']
        for sensor in sensors:
            if len(sensors[sensor])>0:
                for sensor_type in sensors[sensor]:
                    if sensor_type["name"]==self.sensor_name:
                        sensor_config=sensor_type

        return sensor_config


# Math helperfunctions
class Quaternion():
    def __init__(self,in_x,in_y,in_z,in_w):
        self.x = in_x
        self.y = in_y
        self.z = in_z
        self.w = in_w

    def __str__(self):
        return "[" + str(round(self.x,3)) + ", " + str(round(self.y,3)) + ", " + str(round(self.z,3)) + ", " + str(round(self.w,3)) +"]" 

def degrees_to_rads(deg):
  return (deg * math.pi) / 180.0

def toQuaternion(pitch, roll, yaw, log = True):
    # Abbreviations for the various angular functions
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    q = Quaternion(0,0,0,0)
    q.w = cy * cr * cp + sy * sr * sp
    q.x = cy * sr * cp - sy * cr * sp
    q.y = cy * cr * sp + sy * sr * cp
    q.z = sy * cr * cp - cy * sr * sp

    if log:
        print("Transform Euler angles to quaternion(x,y,z,w):")
        print("- Input:")
        print("\t· Roll: " + str(roll))
        print("\t· Pitch: " + str(pitch))
        print("\t· Yaw: " + str(yaw))
        print("Output:\n\t· Quat: " + str(q))

    return q

# Input [x,y,z,w] list with quat
def toEulerAngle(q, log = True):
    # roll (x-axis rotation)
    sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = +2.0 * (q.w * q.y - q.z * q.x)
    if (math.fabs(sinp) >= 1):
        pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = +2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)  
    yaw = math.atan2(siny_cosp, cosy_cosp)

    if log:
        print("Transform quaternion(x,y,z,w) to Euler angles:")
        print("- Input:\n\t· Quat: " + str(q))
        print("Output:")
        print("\t· Roll: " + str(roll))
        print("\t· Pitch: " + str(pitch))
        print("\t· Yaw: " + str(yaw))
    return [roll,pitch,yaw]


def rotate_vector(vector, angle, axis):

    # Convert axis to unit vector
    axis = axis / np.linalg.norm(axis)
    # Calculate the rotation matrix
    s = np.sin(angle)
    c = np.cos(angle)
    t = 1 - c

    rot_matrix = np.array([[t*axis[0]**2+c, t*axis[0]*axis[1]-s*axis[2], t*axis[0]*axis[2]+s*axis[1]],

        [t*axis[0]*axis[1]+s*axis[2], t*axis[1]**2+c, t*axis[1]*axis[2]-s*axis[0]],

        [t*axis[0]*axis[2]-s*axis[1], t*axis[1]*axis[2]+s*axis[0], t*axis[2]**2+c]])

    # Apply the rotation transformation
    rotated_vector = rot_matrix.dot(vector)

    return rotated_vector