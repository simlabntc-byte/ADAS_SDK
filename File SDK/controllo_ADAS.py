import cv2
import zmq
import numpy as np
import time
import math
from ultralytics import YOLO

# Import delle librerie WorldSim
import ws_api.ws_session as ws_session_lib
import RHMsg.ImageMeasurement
import RHMsg.ImageEncoding
import RHMsg.Source
import RHMsg.GroupedStateMeasurement

# ==========================================
# Inizializzazione YOLO
# ==========================================
print("Caricamento modello YOLOv8...")
yolo_model = YOLO("yolov8n.pt") # YOLOv8 Nano: veloce e leggero

# ==========================================
# Funzioni di Supporto (Telemetria e Percezione)
# ==========================================
def get_ego_speed(buffer, ego_uuid_str):
    """Estrae la velocità del veicolo dal buffer degli stati ZMQ"""
    try:
        states_msg = RHMsg.GroupedStateMeasurement.GroupedStateMeasurement.GetRootAsGroupedStateMeasurement(buffer, 0)
        uuid_parts = ego_uuid_str.split('-')
        uuid_a = int(uuid_parts[0][:8], 16)
        uuid_b = int(uuid_parts[1][:4] + uuid_parts[2][:4], 16)
        uuid_c = int(uuid_parts[3][:4] + uuid_parts[4][:4], 16)
        uuid_d = int(uuid_parts[4][4:], 16)

        for ii in range(states_msg.PhysicsVehicleStatesLength()):
            physics_vehicle_state = states_msg.PhysicsVehicleStates(ii)
            uuid = physics_vehicle_state.Header().Uuid()
            
            if uuid_a==uuid.A() and uuid_b==uuid.B() and uuid_c==uuid.C() and uuid_d==uuid.D():
                velocity = physics_vehicle_state.Velocity()
                return math.sqrt(velocity.X()**2 + velocity.Y()**2 + velocity.Z()**2)
    except Exception:
        pass
    return None

def stima_distanza_ostacolo(bbox, img_height, img_width):
    """Converte un bounding box (pixel) in una stima di distanza (metri X, Y)"""
    x_min, y_min, x_max, y_max = bbox
    bottom_center_x = (x_min + x_max) / 2.0
    bottom_y = y_max
    
    # Stima asse X (Longitudinale): basata su quanto il box è vicino al fondo dell'immagine
    pixel_dall_orizzonte = bottom_y - (img_height / 2.0)
    if pixel_dall_orizzonte <= 0: 
        return 100.0, 0.0 # Troppo lontano
    
    # COSTANTE DA TARARE SUL TUO SIMULATORE
    distanza_x_metri = 400.0 / pixel_dall_orizzonte 
    
    # Stima asse Y (Laterale): basata su quanto il box è distante dal centro orizzontale
    centro_img_x = img_width / 2.0
    offset_pixel_x = bottom_center_x - centro_img_x
    distanza_y_metri = (offset_pixel_x / img_width) * (distanza_x_metri * 0.5) 
    
    return distanza_x_metri, distanza_y_metri

def rileva_linee_corsia(image_bgr):
    """Rileva le linee con confini spaziali rigidi e calcolo stabile sul fondo."""
    img_height, img_width = image_bgr.shape[:2]
    centro_x = img_width // 2
    
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    
    # Maschera
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, img_height), 
        (img_width, img_height), 
        (int(img_width*0.6), int(img_height*0.55)), 
        (int(img_width*0.4), int(img_height*0.55))
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_edges = cv2.bitwise_and(edges, mask)
    
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, threshold=40, minLineLength=40, maxLineGap=20)
    
    left_lines = []
    right_lines = []
    
    # MURO INVISIBILE
    MARGIN = int(img_width * 0.05) 

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1: continue
            
            pendenza = (y2 - y1) / (x2 - x1)
            
            if abs(pendenza) < 0.4 or abs(pendenza) > 4.0: 
                continue 
            
            # TORNATI ALLA STABILITA': Calcoliamo l'intersezione esattamente sul fondo dell'immagine
            x_intersezione = int(x1 + (img_height - y1) / pendenza)
            
            if pendenza < 0 and x_intersezione < (centro_x - MARGIN):
                left_lines.append(x_intersezione)
                cv2.line(image_bgr, (x1, y1), (x2, y2), (255, 0, 0), 2)
                
            elif pendenza > 0 and x_intersezione > (centro_x + MARGIN):
                right_lines.append(x_intersezione)
                cv2.line(image_bgr, (x1, y1), (x2, y2), (0, 0, 255), 2)

    left_c0_pixel = max(left_lines) if left_lines else None
    right_c0_pixel = min(right_lines) if right_lines else None
                
    # Disegno dei pallini sul fondo dell'immagine
    if left_c0_pixel: cv2.circle(image_bgr, (left_c0_pixel, img_height-10), 10, (255, 0, 0), -1)
    if right_c0_pixel: cv2.circle(image_bgr, (right_c0_pixel, img_height-10), 10, (0, 0, 255), -1)

    pixel_to_meters = 0.015 # Scala corretta
    left_c0 = (left_c0_pixel - centro_x) * pixel_to_meters if left_c0_pixel else None
    right_c0 = (right_c0_pixel - centro_x) * pixel_to_meters if right_c0_pixel else None
    
    return left_c0, right_c0, image_bgr

# ==========================================
# Parametri di Configurazione
# ==========================================
worldsim_host = "http://localhost:8080"
scenario_filename = "./scenarios/prova2.json" # Assicurati che abbia macchine e pedoni!
ego_name = "ego"
sensor_name = "Camera" # Usiamo la telecamera normale!
fps = 60

# Parametri ADAS (Aggiorna questa sezione)
TARGET_SPEED_KMH = 15.0
TARGET_SPEED_MS = TARGET_SPEED_KMH / 3.6
CC_KP = 0.3
AEB_DISTANCE_THRESHOLD = 5.0
AEB_LATERAL_MARGIN = 1.0

# Nuovi Parametri PID per lo Sterzo (LKA)
# ==========================================
# Variabili di stato per il PID dello sterzo (LKA)
# ==========================================
LKA_KP = 0.08  # Dimezzato rispetto al tuo 0.15 originale. L'auto sarà meno "nervosa".
LKA_KD = 0.08  # Frena dolcemente le oscillazioni.
LKA_KI = 0.00  # Spento.
ALPHA_SMOOTHING = 0.2  # Torniamo a 0.2 per filtrare bene il rumore senza addormentare lo sterzo.

prev_lane_error = 0.0
integral_lane_error = 0.0
prev_steer_cmd = 0.0
smoothed_lane_error = 0.0

# ==========================================
# Inizializzazione WorldSim
# ==========================================
print("Inizializzazione sessione WorldSim...")
wss = ws_session_lib.worldsim_session(worldsim_host, fps)
wss.run_worldsim_deterministic()
wss.reset()

config = { 'version': { 'major': 2023 }, 'async_reporting': False, 'frame_rate': fps, "user_vehicle_kinematic": False, "user_vehicle_snap_to_ground": True }
wss.set_config(config)
sim_control = ws_session_lib.SimController(wss.get_config()['sim_control_endpoint'])

wss.load_scenario_file(scenario_filename)
ego_config = { 'name': ego_name, 'kinematic': False }
wss.set_user_controlled_agent(ego_config)

ego_agent = wss.get_user_controlled_agent(ego_config['name'])
ego_uuid_str = wss.get_ego_vehicle_uuid_str()
vehicle_controller = ws_session_lib.VehicleController(ego_agent['endpoint'], ego_agent['uuid'])

context = zmq.Context()

# Socket Stati (Velocità)
states_endpoint = wss.get_states_endpoint()
states_socket = context.socket(zmq.SUB)
states_socket.setsockopt(zmq.CONFLATE, 1)
states_socket.connect(states_endpoint)
states_socket.setsockopt_string(zmq.SUBSCRIBE, '')
states_socket.setsockopt(zmq.RCVTIMEO, 10)

# Socket Telecamera
ws_sensor = ws_session_lib.worldsim_sensor(worldsim_host, ego_name, sensor_name)
camera_endpoint, _, _ = ws_sensor.get_sensor_endpoint_by_name()
camera_socket = context.socket(zmq.SUB)
camera_socket.setsockopt(zmq.CONFLATE, 1)
camera_socket.connect(camera_endpoint)
camera_socket.setsockopt_string(zmq.SUBSCRIBE, '')
camera_socket.setsockopt(zmq.RCVTIMEO, 10)

wss.scenario_play()
print("Simulazione avviata! (Premi 'q' sulla finestra del video per uscire)")

# ==========================================
# Ciclo di Controllo Principale
# ==========================================
sim_time = 0.0
current_speed_ms = 0.0

try:
    while True:
        sim_time = sim_control.advance_to_next_step()
        
        # 1. Aggiorna la telemetria (Velocità)
        try:
            states_buffer = states_socket.recv()
            if states_buffer is not None:
                new_speed = get_ego_speed(states_buffer, ego_uuid_str)
                if new_speed is not None: current_speed_ms = new_speed
        except zmq.ZMQError:
            pass 

        current_speed_kmh = current_speed_ms * 3.6
        status_msg = ""
        steer_cmd, throttle_cmd, brake_cmd = 0.0, 0.0, 0.0
        aeb_active = False

        # 2. Leggi e Processa Immagine
        camera_buffer = None
        try:
            camera_buffer = camera_socket.recv()
        except zmq.ZMQError:
            pass

        if camera_buffer is not None:
            image_msg = RHMsg.ImageMeasurement.ImageMeasurement.GetRootAsImageMeasurement(camera_buffer, 0)
            pixel_values = image_msg.ValuesAsNumpy()
            
            if pixel_values.size != 0:
                width, height = image_msg.Width(), image_msg.Height()
                frame = pixel_values.reshape(height, width, 3)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # A seconda di WorldSim, potresti dover usare COLOR_RGB2BGR
                
                # --- LOGICA AEB (YOLOv8) ---
                results = yolo_model(frame, verbose=False)
                for r in results:
                    for box in r.boxes:
                        cls = int(box.cls[0])
                        # 0: person, 2: car, 3: motorcycle, 5: bus, 7: truck
                        if cls in [0, 2, 3, 5, 7]: 
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            obs_x, obs_y = stima_distanza_ostacolo([x1, y1, x2, y2], height, width)
                            
                            # Disegna il box su schermo
                            color = (0, 0, 255) if (0 < obs_x < AEB_DISTANCE_THRESHOLD and abs(obs_y) < AEB_LATERAL_MARGIN) else (0, 255, 255)
                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                            cv2.putText(frame, f"X:{obs_x:.1f}m Y:{obs_y:.1f}m", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                            
                            # Attivazione AEB
                            if color == (0, 0, 255): 
                                aeb_active = True
                                obs_x_log, obs_y_log = obs_x, obs_y

                if aeb_active:
                    brake_cmd, throttle_cmd = 1.0, 0.0
                    status_msg = f"🚨 [AEB] Ostacolo a X:{obs_x_log:.1f}m Y:{obs_y_log:.1f}m"
                    cv2.putText(frame, "AEB ACTIVE!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)
                else:
                    # --- LOGICA CRUISE CONTROL ---
                    speed_error = TARGET_SPEED_MS - current_speed_ms
                    if speed_error > 0:
                        throttle_cmd = max(0.0, min(CC_KP * speed_error, 1.0))
                    elif speed_error < -0.5: 
                        brake_cmd = min(abs(speed_error) * 0.1, 1.0)
                    
                    status_msg = f"🟢 [CC] {current_speed_kmh:.1f} km/h"

                    # --- LOGICA LKA (OpenCV) ---
                    # --- LOGICA LKA (OpenCV + PID + Filtro) ---
                    left_c0, right_c0, frame = rileva_linee_corsia(frame)
                    
                    if left_c0 is not None and right_c0 is not None:
                        # 1. Calcolo dell'errore grezzo
                        raw_lane_error = (left_c0 + right_c0) / 2.0
                        
                        # 2. Filtro Passa-Basso (Smoothing) per ignorare il rumore di OpenCV
                        smoothed_lane_error = (ALPHA_SMOOTHING * raw_lane_error) + ((1.0 - ALPHA_SMOOTHING) * smoothed_lane_error)
                        
                        # 3. Calcolo Controllore PID
                        p_term = LKA_KP * smoothed_lane_error
                        d_term = LKA_KD * (smoothed_lane_error - prev_lane_error)
                        
                        integral_lane_error += smoothed_lane_error
                        integral_lane_error = max(min(integral_lane_error, 5.0), -5.0) # Anti-windup
                        i_term = LKA_KI * integral_lane_error
                        
                        raw_steer_cmd = p_term + i_term + d_term
                        
                        # 4. Filtro Meccanico: Limita quanto il volante può girare in un singolo frame
                        MAX_STEER_CHANGE = 0.05
                        if raw_steer_cmd > prev_steer_cmd + MAX_STEER_CHANGE:
                            steer_cmd = prev_steer_cmd + MAX_STEER_CHANGE
                        elif raw_steer_cmd < prev_steer_cmd - MAX_STEER_CHANGE:
                            steer_cmd = prev_steer_cmd - MAX_STEER_CHANGE
                        else:
                            steer_cmd = raw_steer_cmd
                            
                        # Saturazione ai limiti di sterzo (-1.0, 1.0)
                        steer_cmd = max(min(steer_cmd, 1.0), -1.0)
                        
                        # 5. Aggiornamento stato per il prossimo frame
                        prev_lane_error = smoothed_lane_error
                        prev_steer_cmd = steer_cmd
                        
                        status_msg += f" | 🛣️ [LKA PID] Err: {smoothed_lane_error:.2f} | Str: {steer_cmd:.2f}"
                    else:
                        # Se perde le linee, non dare strattoni ma riallinea il volante dolcemente
                        steer_cmd = prev_steer_cmd * 0.9
                        prev_steer_cmd = steer_cmd
                        status_msg += f" | ⚠️ [LKA] Linee perse, centraggio dolce"

                print(f"[{sim_time:.2f}s] {status_msg}")
                
                # Mostra la Dashboard
                cv2.imshow("ADAS Dashboard (YOLO + OpenCV)", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        # 3. Invio comandi veicolo
        vehicle_controller.send_PhysicsVehicleActuationCommand(sim_time, steer_cmd, throttle_cmd, brake_cmd)

except KeyboardInterrupt:
    pass
finally:
    print("\nChiusura del programma...")
    cv2.destroyAllWindows()