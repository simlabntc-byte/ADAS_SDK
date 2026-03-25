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


exereg=r"SOFTWARE\VI-grade\VI-WorldSim\2025"

# This Python class `worldsim_session` provides methods to interact with a WorldSim simulation
# environment, including starting different modes, loading scenarios, setting configurations, and
# retrieving simulation information.
class worldsim_session:
    def __init__(self, host,fps=60):
        """
        This Python function initializes an object with a specified host and optional frames per second
        (fps) value.
        
        :param host: The `host` parameter in the `__init__` method is used to specify the host for the world
        simulation. It is the address or name of the host where the simulation will be running
        :param fps: The `fps` parameter in the `__init__` method is a parameter that represents the frames
        per second (fps) for the simulation. It is set to a default value of 60 if no value is provided when
        creating an instance of the class, defaults to 60 (optional)
        """
        self.worldsim_host = host
        self.sessionrunning=False
        self.fps=fps

    # Start WorldSim StandAlone Mode
    def run_worldsim(self):
        """
        The `run_worldsim` function in Python connects to the Windows registry to retrieve the installation
        directory of WorldSim, then launches the WorldSim executable with specified parameters and waits for
        it to finish.
        """
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
        """
        This Python function runs WorldSim in Distributed Mode with optional sensor node functionality.
        
        :param sensor_node: The `sensor_node` parameter in the `run_worldsim_distributed` function is a
        boolean flag that determines whether to start the WorldSim Sensor Node. If `sensor_node` is set to
        `True`, the function will start the WorldSim Sensor Node subprocess. If it is set to `False,
        defaults to False (optional)
        """
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
        """
        The function `run_worldsim_deterministic` runs the WorldSim application in deterministic mode with
        specified parameters.
        """
        print("Running WorldSim - Deterministic")
        registry = winreg.ConnectRegistry(None, winreg.HKEY_LOCAL_MACHINE)
        reg_key = winreg.OpenKey(registry, r"SOFTWARE\VI-grade\VI-WorldSim\2024")
        reg_val = winreg.QueryValueEx(reg_key, "InstallDir")
        worldsim_exe = reg_val[0] + "\WorldSim.exe"
        print(worldsim_exe)
        subprocess.Popen([worldsim_exe, "-windowed",  "-deterministic", f"-fps={self.fps}", "user_vehicle_snap_to_ground=True",  "-log"])
        self.wait_for_worldsim()

    def connect_to_worldim (self):
        """
        The function `connect_to_worldim` waits for a world simulation to start, retrieves the
        configuration, and sets the frames per second based on the configuration.
        """
        self.wait_for_worldsim()
        config=self.get_config()
        self.fps=config['frame_rate']


    def wait_for_worldsim (self):
        """
        This function waits for a Worldsim system to start up before returning.
        :return: The `wait_for_worldsim` method is returning when the Worldsim system is up and running. It
        waits for the Worldsim status endpoint to return a 200 status code, indicating that the system is
        started. Once the system is up, it sets `self.sessionrunning` to True and prints messages indicating
        that Worldsim has started. Finally, it provides instructions for toggling vehicle interior and
        wheels
        """
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
        """
        The `load_scenario_file` function loads a WorldSim scenario file if a session is running, otherwise
        it prints a message and exits.
        
        :param ws_scenario_filename: The `ws_scenario_filename` parameter is a string that represents the
        filename of the WorldSim scenario file that you want to load in the `load_scenario_file` method.
        This method is responsible for loading a WorldSim scenario from a file with the given filename
        """
        if self.sessionrunning:
            print("Loading WorldSim Scenario '%s'" % ws_scenario_filename)
            with open(ws_scenario_filename, 'rb') as scenario_file:
                scenario = json.load(scenario_file)
                self.scenario_load_json(scenario)
        else:
            print (f"No valid WorldSim session is running on {self.worldsim_host}")
            exit()


    def scenario_load_json(self, scenario):
        """
        This Python function sends a scenario JSON to be loaded using a PUT request.
        
        :param scenario: The `scenario` parameter in the `scenario_load_json` method is a JSON object that
        contains data related to a scenario. This data will be encoded into JSON format using the
        `json.JSONEncoder().encode(scenario)` method before being sent to the specified URL for loading into
        the system
        """
        json_data = json.JSONEncoder().encode(scenario)
        put_data = json_data.encode('utf-8')
        req = request.Request(url = self.worldsim_host + "/worldsim/scenario/load", data=put_data, method='PUT')
        req.add_header('Content-Length', len(put_data))
        req.add_header('Content-Type', 'application/json')
        request.urlopen(req)

    def get_scenario(self):
        """
        The `get_scenario` function sends a GET request to a specified URL to retrieve and return scenario
        information in JSON format.
        :return: The `get_scenario` method returns a dictionary containing the scenario status information
        obtained from the specified URL after making a GET request and converting the response to JSON
        format.
        """
        scenario_status_url=f"{self.worldsim_host}/worldsim/scenario"
        print (scenario_status_url)
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()
        return scenario_dict

    def reset(self):
        """
        The `reset` function clears the simulation environment without restarting the executable.
        """
        """Unloads the scenario for a blank slate without restarting the executable"""
        print("Clearing simulation environment")
        req = request.Request(url=self.worldsim_host + "/worldsim/scenario/stop")
        request.urlopen(req)


    def set_config(self, config):
        """
        The `set_config` function replaces the current configuration with a new one by sending a PUT request
        with the new configuration data in JSON format to a specified URL.
        
        :param config: The `config` parameter in the `set_config` method is a dictionary containing the
        configuration data that you want to set for the world simulation. This configuration data will be
        encoded into JSON format before sending it to the server for updating the configuration
        """
        """Replaces the current configuration with a new one"""
        json_data = json.JSONEncoder().encode(config)
        put_data = json_data.encode('utf-8')
        req = request.Request(url=self.worldsim_host + "/worldsim/config/sim", data=put_data, method='PUT')
        req.add_header('Content-Length', len(put_data))
        req.add_header('Content-Type', 'application/json')
        request.urlopen(req)


    def get_config(self):
        """
        The `get_config` function fetches the current simulation configuration from a specified URL.
        :return: The `get_config` method returns the current simulation configuration fetched from the
        specified URL. It makes a request to the URL, reads the response, decodes it as a JSON object, and
        then returns the configuration.
        """
        """Fetches the current simulation configuration"""
        req = request.Request(url=self.worldsim_host + "/worldsim/config/sim")
        with request.urlopen(req) as f:
            body = f.read().decode('utf-8')
            config = json.JSONDecoder().decode(body)
            return config


    def set_user_controlled_agent(self, config):
        """
        The function `set_user_controlled_agent` sends a PUT request to a specified URL with JSON data to
        inform the simulator about controlling a particular vehicle.
        
        :param config: The `config` parameter in the `set_user_controlled_agent` method is a dictionary
        containing information about the vehicle you want to control. It likely includes details such as the
        name of the vehicle (`config['name']`) and possibly other configuration parameters specific to the
        vehicle or its control settings
        """
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
        """
        The function `get_user_controlled_agent` fetches the endpoint for ego vehicle actuation based on the
        provided agent name.
        
        :param name: The `name` parameter in the `get_user_controlled_agent` function is used to specify the
        name of the user-controlled agent for which the endpoint for ego vehicle actuation needs to be
        fetched
        :return: The `get_user_controlled_agent` function returns the configuration for the user-controlled
        agent with the specified name.
        """
        """Fetches the endpoint for ego vehicle actuation """
        req = request.Request(url=self.worldsim_host + "/worldsim/config/controlled_agents/" + name)
        with request.urlopen(req) as f:
            body = f.read().decode('utf-8')
            config = json.JSONDecoder().decode(body)
            return config


    def get_user_controlled_agents(self):
        """
        The function `get_user_controlled_agents` fetches the endpoint for ego vehicle actuation from a
        specified URL.
        :return: The `get_user_controlled_agents` method returns a dictionary containing information about
        the user-controlled agents in the simulation environment.
        """
        """Fetches the endpoint for ego vehicle actuation """
        req = request.Request(url=self.worldsim_host + "/worldsim/config/controlled_agents/")
        with request.urlopen(req) as f:
            body = f.read().decode('utf-8')
            agents_dict = json.JSONDecoder().decode(body)
            return agents_dict


    def get_ego_vehicle_uuid_str(self):
        """
        The function `get_ego_vehicle_uuid_str` parses a scenario to extract the UUID for the ego vehicle.
        :return: The function `get_ego_vehicle_uuid_str` returns the UUID of the ego vehicle extracted from
        the scenario data.
        """
        """Parses the scenario and extracts the UUID for the ego vehicle"""
        req = request.Request(url=self.worldsim_host + "/worldsim/scenario")
        with request.urlopen(req) as f:
            body = f.read().decode('utf-8')
            scenario = json.JSONDecoder().decode(body)
            vehicles = scenario['actors']['vehicles']
            ego_vehicle = [vehicle for vehicle in vehicles if vehicle.get('name') == 'ego'][0]
            return ego_vehicle['uuid']


    def get_states_endpoint(self):
        """
        The function `get_states_endpoint` retrieves the endpoint used to retrieve simulation state from a
        specified URL.
        :return: The code snippet is returning the endpoint used to retrieve simulation state from a JSON
        response obtained by making a request to a specific URL. The endpoint is extracted from the
        'state_endpoints' key nested within the 'implementation' key of the JSON response.
        """
        """Gets the endpoint used to retrieve simulation state"""
        req = request.Request(url=self.worldsim_host + "/worldsim/scenario")
        with request.urlopen(req) as f:
            body = f.read().decode('utf-8')
            scenario = json.JSONDecoder().decode(body)
            # middleware sockets are for SDK use
            return scenario['implementation']['state_endpoints']['middleware']


    def scenario_play(self):
        """
        The `scenario_play` function sends a GET request to a specified URL to signal that configuration is
        complete and time can start advancing.
        """
        """Signals we are done configuring and ready for time to start advancing

        Up until this point there can be ports opened and waiting for SDK input. If they are unused
        at this stage they will be closed and default behavior used instead.
        """
        req = request.Request(url=self.worldsim_host + "/worldsim/scenario/play", method='GET')
        request.urlopen(req)



    # Misc rest API endpoints


    def get_assets(self):
        """
        The `get_assets` function sends a GET request to a specified URL and returns the JSON response.
        :return: The `get_assets` method is returning a dictionary containing information about the assets
        from the specified URL `scenario_status_url`.
        """
        scenario_status_url=f"{self.worldsim_host}/worldsim/assets"
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()
        return scenario_dict

    def get_vehicles(self):
        """
        The `get_vehicles` function sends a GET request to a specified URL to retrieve information about
        vehicles in a simulation scenario.
        :return: The `get_vehicles` method is making a GET request to the `scenario_status_url` endpoint to
        retrieve information about vehicles from the world simulation host. It then converts the response
        JSON data into a dictionary named `scenario_dict` and returns this dictionary containing information
        about vehicles.
        """
        scenario_status_url=f"{self.worldsim_host}/worldsim/assets/vehicles"
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()
        return scenario_dict

    def get_dashboard_config(self):
        """
        The `get_dashboard_config` function sends a GET request to retrieve dashboard configuration data
        from a specified URL.
        :return: The `get_dashboard_config` method returns a dictionary containing the configuration
        settings for the dashboard related to the "ego" scenario.
        """
        scenario_status_url=f"{self.worldsim_host}/worldsim/config/dashboards/ego"
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()
        return scenario_dict
    
    def set_dashboard(self, dashconfig):
        """
        The `set_dashboard` function takes a dashboard configuration dictionary, encodes it to JSON, and
        sends a PUT request to update the dashboard configuration on a specified URL.
        
        :param dashconfig: The `dashconfig` parameter is a dictionary that contains configuration settings
        for different dashboard components. Here is an example of the `dashconfig` dictionary:
        """
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
        """
        The `get_info` function sends a GET request to a specific URL and returns the JSON response as a
        dictionary.
        :return: The `get_info` method returns a dictionary containing information about a scenario from a
        specified URL endpoint.
        """
        scenario_status_url=f"{self.worldsim_host}/worldsim/describe/info"
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()
        return scenario_dict

    def get_available_tracks(self):
        """
        The function `get_available_tracks` extracts track names from a configuration and returns them as a
        list.
        :return: The `get_available_tracks` method returns a list of available tracks extracted from the
        configuration data.
        """
        track_list=[]
        config=self.get_info()
        for item in config:
            if "track:" in item:
                track_list.append(item.split(':')[1])
        return track_list

    def get_paths_geojson(self,level):
        """
        This Python function retrieves paths data in GeoJSON format for a specific level from a WorldSim
        API.
        
        :param level: The `level` parameter in the `get_paths_geojson` function is used to specify the level
        of the track for which you want to retrieve the paths in GeoJSON format
        :return: The function `get_paths_geojson` returns a dictionary containing the paths in GeoJSON
        format for a specific level in the world simulation.
        """
        scenario_status_url=f"{self.worldsim_host}/worldsim/describe/track/track:{level}/paths.geojson"
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()
        return scenario_dict

    def get_environment(self):
        """
        The `get_environment` function sends a GET request to a specified URL and returns the JSON response.
        :return: The `get_environment` method returns a dictionary containing information about the scenario
        environment fetched from the specified URL `scenario_status_url`.
        """
        scenario_status_url=f"{self.worldsim_host}/worldsim/scenario/environment"
        resp = requests.get(scenario_status_url)
        scenario_dict = resp.json()
        return scenario_dict

    def get_vehicles_in_scenario(self):
        """
        The function `get_vehicles_in_scenario` retrieves a list of vehicle types from a given scenario.
        :return: The function `get_vehicles_in_scenario` returns a list of vehicle types present in the
        scenario.
        """
        scenario=self.get_scenario()
        vehicle_list=scenario['actors']['vehicles']
        vehicle_types=[]
        for vehicle in vehicle_list:
            vehicle_types.append(vehicle['asset']['class_name'])
        return vehicle_types

    def get_vehicle_uuid_str(self,vehicle_name):
        """
        This Python function retrieves the UUID of a specified vehicle from a scenario by parsing the
        scenario data.
        
        :param vehicle_name: The `vehicle_name` parameter is a string that represents the name of a vehicle
        for which you want to retrieve the UUID
        :return: The function `get_vehicle_uuid_str` returns the UUID of the vehicle with the specified name
        in the scenario.
        """
        """Parses the scenario and extracts the UUID for the ego vehicle"""
        url=self.worldsim_host + "/worldsim/scenario"
        resp = requests.get(url)
        scenario = resp.json()
        vehicles = scenario['actors']['vehicles']
        vehicle = [vehicle for vehicle in vehicles if vehicle.get('name') == vehicle_name][0]
        return vehicle['uuid']


# The `VehicleController` class in Python defines methods for sending kinematic and physics vehicle
# actuation commands over ZeroMQ.
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




# Consumes the resulting state from the system
# This class `StateConsumer` sets up a ZeroMQ subscriber to receive state updates and handles the
# received messages.
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
# The `PedestrianController` class in Python sets up a ZeroMQ PUB socket to send pedestrian actuation
# commands with specific data structures.
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



# The `SimController` class in Python uses ZeroMQ to communicate with a simulation endpoint, allowing
# for advancing the simulation to the next step.
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


# Math helperfunctions
class Quaterniond():
    def __init__(self,in_x,in_y,in_z,in_w):
        self.x = in_x
        self.y = in_y
        self.z = in_z
        self.w = in_w

    def __str__(self):
        return "[" + str(round(self.x,3)) + ", " + str(round(self.y,3)) + ", " + str(round(self.z,3)) + ", " + str(round(self.w,3)) +"]" 

def degrees_to_rads(deg):
    """
    The function `degrees_to_rads` converts degrees to radians in Python.
    
    :param deg: The parameter `deg` in the `degrees_to_rads` function represents an angle measurement in
    degrees that you want to convert to radians
    :return: The function `degrees_to_rads` takes an input in degrees and returns the equivalent value
    in radians.
    """
    return (deg * math.pi) / 180.0

def toQuaternion(pitch, roll, yaw, log = True):
    """
    The function `toQuaternion` converts Euler angles (pitch, roll, yaw) to a quaternion representation
    and optionally logs the transformation.
    
    :param pitch: The pitch parameter in the `toQuaternion` function represents the rotation around the
    x-axis in Euler angles. It is half the angle of rotation around the x-axis
    :param roll: The roll parameter typically refers to the rotation around the x-axis in a 3D space. It
    represents the tilting motion of an object from side to side
    :param yaw: The yaw parameter represents the rotation around the vertical axis (z-axis) in the 3D
    space. It is one of the Euler angles that define the orientation of an object in three-dimensional
    space
    :param log: The `log` parameter in the `toQuaternion` function is a boolean flag that determines
    whether to print the transformation process from Euler angles to quaternion along with the input and
    output values. If `log` is set to `True`, the function will print the transformation details. If
    `log` is, defaults to True (optional)
    :return: The function `toQuaternion` returns a Quaternion object `q` that represents the
    transformation of Euler angles (pitch, roll, yaw) to quaternion (x, y, z, w) format.
    """
    # Abbreviations for the various angular functions
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    q = Quaterniond(0,0,0,0)
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
    """
    The function `toEulerAngle` converts a quaternion to Euler angles (roll, pitch, yaw) and optionally
    logs the input and output.
    
    :param q: The `q` parameter in the `toEulerAngle` function represents a quaternion. Quaternions are
    a mathematical way to represent rotations in 3D space. Quaternions have four components: `w`, `x`,
    `y`, and `z`, where `w` is the
    :param log: The `log` parameter in the `toEulerAngle` function is a boolean flag that determines
    whether to print the transformation process from quaternion to Euler angles. If `log` is set to
    `True`, the function will print the input quaternion and the output Euler angles (roll, pitch, yaw,
    defaults to True (optional)
    :return: The function `toEulerAngle` is returning a list containing the Euler angles for roll,
    pitch, and yaw in that order.
    """
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
    """
    The function `rotate_vector` rotates a given vector by a specified angle around a specified axis
    using a rotation matrix.
    
    :param vector: The `vector` parameter represents the vector that you want to rotate. It should be a
    3-dimensional vector represented as a NumPy array with shape (3,). This vector will be rotated by
    the specified angle around the specified axis
    :param angle: The `angle` parameter in the `rotate_vector` function represents the angle by which
    you want to rotate the input vector around the specified axis. This angle is typically given in
    radians
    :param axis: The `axis` parameter in the `rotate_vector` function represents the axis of rotation
    for the vector. It is a 3D vector that defines the direction around which the vector will be rotated
    :return: The function `rotate_vector` returns the vector `vector` rotated by the specified `angle`
    around the `axis` provided.
    """

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