import zmq
import numpy as np
import time
import math
from ruamel.yaml import YAML
from pathlib import Path

# Import delle librerie WorldSim
import ws_api.ws_session as ws_session_lib
import RHMsg.VirtualSensorMeasurement
import RHMsg.GroupedStateMeasurement

# ==========================================
# Funzioni di Supporto
# ==========================================
def get_ego_speed(buffer, ego_uuid_str):
    """Estrae la velocità del veicolo dal buffer degli stati ZMQ"""
    try:
        states_msg = RHMsg.GroupedStateMeasurement.GroupedStateMeasurement.GetRootAsGroupedStateMeasurement(buffer, 0)
        
        # Parsing dell'UUID come nel tuo file
        uuid_parts = ego_uuid_str.split('-')
        uuid_a = int(uuid_parts[0][:8], 16)
        uuid_b = int(uuid_parts[1][:4] + uuid_parts[2][:4], 16)
        uuid_c = int(uuid_parts[3][:4] + uuid_parts[4][:4], 16)
        uuid_d = int(uuid_parts[4][4:], 16)

        # Cerchiamo tra i PhysicsVehicleStates
        for ii in range(states_msg.PhysicsVehicleStatesLength()):
            physics_vehicle_state = states_msg.PhysicsVehicleStates(ii)
            uuid = physics_vehicle_state.Header().Uuid()
            
            if uuid_a==uuid.A() and uuid_b==uuid.B() and uuid_c==uuid.C() and uuid_d==uuid.D():
                velocity = physics_vehicle_state.Velocity()
                # Calcolo del modulo della velocità (vettore 3D) in m/s
                return math.sqrt(velocity.X()**2 + velocity.Y()**2 + velocity.Z()**2)
    except Exception as e:
        print(f"Errore parsing velocità: {e}")
    
    return None

# ==========================================
# Parametri di Configurazione
# ==========================================
worldsim_host = "http://localhost:8080"
scenario_filename = "./scenarios/wolfsburg_day.json" # o "./scenarios/prova.json"
ego_name = "ego"
sensor_name = "VirtualCamera"
fps = 60

# Parametri ADAS
TARGET_SPEED_KMH = 15.0
TARGET_SPEED_MS = TARGET_SPEED_KMH / 3.6
CC_KP = 0.3                   # Guadagno proporzionale dell'acceleratore
AEB_DISTANCE_THRESHOLD = 20.0 # Metri
AEB_LATERAL_MARGIN = 2.0      # Larghezza (metri)
LKA_KP = 0.1                  # Guadagno proporzionale per lo sterzo

# ==========================================
# Inizializzazione WorldSim
# ==========================================
print("Inizializzazione sessione WorldSim...")
wss = ws_session_lib.worldsim_session(worldsim_host, fps)
wss.run_worldsim_deterministic()
wss.reset()

config = { 'version': { 'major': 2023 }, 'async_reporting': False, 'frame_rate': fps, "user_vehicle_kinematic": False, "user_vehicle_snap_to_ground": True }
wss.set_config(config)
sim_control_endpoint = wss.get_config()['sim_control_endpoint']
sim_control = ws_session_lib.SimController(sim_control_endpoint)

wss.load_scenario_file(scenario_filename)
ego_config = { 'name': ego_name, 'kinematic': False }
wss.set_user_controlled_agent(ego_config)

# Ottenimento UUID e Controller
ego_agent = wss.get_user_controlled_agent(ego_config['name'])
ego_uuid_str = wss.get_ego_vehicle_uuid_str()
vehicle_controller = ws_session_lib.VehicleController(ego_agent['endpoint'], ego_agent['uuid'])

context = zmq.Context()

# --- 1. Setup Socket Telecamera ---
ws_sensor = ws_session_lib.worldsim_sensor(worldsim_host, ego_name, sensor_name)
camera_endpoint, tcp, port = ws_sensor.get_sensor_endpoint_by_name()
print(f"Connesso al sensore {sensor_name} su {camera_endpoint}")

camera_socket = context.socket(zmq.SUB)
camera_socket.setsockopt(zmq.CONFLATE, 1)
camera_socket.connect(camera_endpoint)
camera_socket.setsockopt_string(zmq.SUBSCRIBE, '')
camera_socket.setsockopt(zmq.RCVTIMEO, 10)

# --- 2. Setup Socket Stati (Velocità) ---
states_endpoint = wss.get_states_endpoint()
print(f"Connesso agli stati del veicolo su {states_endpoint}")

states_socket = context.socket(zmq.SUB)
states_socket.setsockopt(zmq.CONFLATE, 1)
states_socket.connect(states_endpoint)
states_socket.setsockopt_string(zmq.SUBSCRIBE, '')
states_socket.setsockopt(zmq.RCVTIMEO, 10)

wss.scenario_play()
print("Simulazione avviata in modalità Autonoma (AEB + LKA + Cruise Control).")

# ==========================================
# Ciclo di Controllo Principale
# ==========================================
sim_time = 0.0
current_speed_ms = 0.0 # Teniamo in memoria l'ultima velocità letta

try:
    while True:
        sim_time = sim_control.advance_to_next_step()
        
        # 1. Lettura Velocità dal socket stati
        try:
            states_buffer = states_socket.recv()
            if states_buffer is not None:
                new_speed = get_ego_speed(states_buffer, ego_uuid_str)
                if new_speed is not None:
                    current_speed_ms = new_speed
        except zmq.ZMQError as e:
            pass # Ignoriamo se il buffer è vuoto per questo frame

        current_speed_kmh = current_speed_ms * 3.6

        # Inizializzazioni di default
        status_msg = ""
        steer_cmd = 0.0
        throttle_cmd = 0.0
        brake_cmd = 0.0
        aeb_active = False

        # 2. Lettura dati telecamera
        camera_buffer = None
        try:
            camera_buffer = camera_socket.recv()
        except zmq.ZMQError as e:
            pass

        # 3. Processamento e Logica ADAS
        if camera_buffer is not None:
            virtual_sensor_msg = RHMsg.VirtualSensorMeasurement.VirtualSensorMeasurement.GetRootAsVirtualSensorMeasurement(camera_buffer, 0)
            
            # --- LOGICA AEB ---
            obsticles_length = virtual_sensor_msg.ObstaclesLength()
            for i in range(obsticles_length):
                obsticle = virtual_sensor_msg.Obstacles(i)
                center_x = obsticle.Center().X() 
                center_y = obsticle.Center().Y() 

                if 0 < center_x < AEB_DISTANCE_THRESHOLD and abs(center_y) < AEB_LATERAL_MARGIN:
                    aeb_active = True
                    obs_x, obs_y = center_x, center_y 
                    break 

            if aeb_active:
                brake_cmd = 1.0
                throttle_cmd = 0.0
                status_msg = f"🚨 [AEB ATTIVO] Freno: 100% | Ostacolo a X:{obs_x:.1f}m Y:{obs_y:.1f}m"
            else:
                # --- LOGICA CRUISE CONTROL ---
                speed_error = TARGET_SPEED_MS - current_speed_ms
                
                if speed_error > 0:
                    throttle_cmd = CC_KP * speed_error
                    throttle_cmd = max(0.0, min(throttle_cmd, 1.0))
                    brake_cmd = 0.0
                else:
                    throttle_cmd = 0.0
                    # Applica freno solo se sta superando la velocità target
                    if speed_error < -0.5: 
                        brake_cmd = min(abs(speed_error) * 0.1, 1.0)
                
                status_msg = f"🟢 [CRUISE] Vel: {current_speed_kmh:.1f}/{TARGET_SPEED_KMH} km/h | Thr: {throttle_cmd:.2f} | Brk: {brake_cmd:.2f}"

                # --- LOGICA LKA ---
                lane_markings_length = virtual_sensor_msg.LaneMarkingsLength()
                left_c0, right_c0 = None, None

                for i in range(lane_markings_length):
                    lm = virtual_sensor_msg.LaneMarkings(i)
                    c0 = lm.C0()
                    if c0 > 0 and (left_c0 is None or c0 < left_c0): left_c0 = c0
                    elif c0 < 0 and (right_c0 is None or c0 > right_c0): right_c0 = c0

                if left_c0 is not None and right_c0 is not None:
                    lane_center_offset = (left_c0 + right_c0) / 2.0
                    steer_cmd = LKA_KP * lane_center_offset
                    steer_cmd = max(min(steer_cmd, 1.0), -1.0)
                    status_msg += f" | 🛣️ [LKA] Offset: {lane_center_offset:.2f}m | Steer: {steer_cmd:.2f}"
                else:
                    status_msg += f" | ⚠️ [LKA] Linee perse"

            print(f"[{sim_time:.2f}s] {status_msg}")

        # 4. Invio comandi
        vehicle_controller.send_PhysicsVehicleActuationCommand(sim_time, steer_cmd, throttle_cmd, brake_cmd)

except KeyboardInterrupt:
    print("\nChiusura del programma...")