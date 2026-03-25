import cv2 as cv2
import zmq
import numpy as np
from ruamel.yaml import YAML
import RHMsg.PointCloudMeasurement
import RHMsg.PointCloudPoint
import open3d as o3d

import ws_api.ws_session as ws_session_lib

first_frame=True
def process_point_cloud_message(buffer):
    global frame,vis,first_frame
    pointcloud_msg = RHMsg.PointCloudMeasurement.PointCloudMeasurement.GetRootAsPointCloudMeasurement(buffer, 0)

    if first_frame:
        first_frame=False
        return

    # read and validate the header (if it exists, otherwise print warning message)
    if pointcloud_msg.Header() is not None:
        header = pointcloud_msg.Header()
        try:
            #assert header.Source() == RHMsg.Source.Source.POINTCLOUD_SENSOR
            timestamp = header.Timestamp()
            sequence_no = header.SequenceNo()
            #assert timestamp > self.last_timestamp
            last_timestamp = timestamp
            #assert sequence_no > self.last_sequence_no
            last_sequence_no = sequence_no

        except Exception as error:
            print("ERROR:", error)
    else:
        print("Warning: received buffer with no header.")

    num_points=pointcloud_msg.PointsLength()
    #print ('***')
    print ('ts: ',timestamp,'seq_no: ',sequence_no, 'PointsLength', num_points)



    o = pointcloud_msg._tab.Bytes
    xyz_array=np.frombuffer(o, np.float32, offset=0)
    # Fudge trim the array first & last 2 points are extra data from flatbuffer
    xyz_array=xyz_array[8:-8]
    #print ('Post Trim ' , len(xyz_array)/4)
    xyz_points=xyz_array.copy()
    # print first & last 20 points
    #print ("method 1")
    #print (len(xyz_points), len(xyz_points)/4 )
    #print (xyz_points[0:40])
    #print (xyz_points[num_points-20:num_points])

    # drop every fouth number - removing Intensity
    #view_points = xyz_points[np.mod(np.arange(xyz_points.size),4)<3]
    #view_points=np.reshape(view_points,(num_points,3))
    #print (view_points[0:10])

    '''
    buffer_list=[]
    for i in range(0,num_points):
        o = pointcloud_msg.Points(i)
        #print (o.X(), o.Y(), o.Z(), o.Intensity())
        x=o.X()
        y=o.Y()
        z=o.Z()
        i=o.Intensity()
        buffer_list.append(x)
        buffer_list.append(y)
        buffer_list.append(z)
        buffer_list.append(i)
    buffer_array=np.array(buffer_list, dtype=np.float32)
    print (np.where(np.isinf(buffer_array)))
    xyz_points=buffer_array.copy()
    # print first & last 20 points
    print ("method 2")
    print (len(xyz_points), len(xyz_points)/4 )
    print (xyz_points[0:20])
    print (xyz_points[num_points-20:num_points])
    '''

    '''
    # Cleanup INF values in array 
    #print (np.where(np.isinf(xyz_points)))
    inf_val=np.where(np.isinf(xyz_points))
    for idx in inf_val:
        xyz_points[idx]=np.nan
    '''
    
    # drop every fouth number - removing Intensity
    xyz_points = xyz_points[np.mod(np.arange(xyz_points.size),4)<3]
    xyz_points=np.reshape(xyz_points,(num_points,3))


    pcd.points = o3d.utility.Vector3dVector(xyz_points)

    if frame == 0:
        vis.add_geometry(pcd)
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

    frame=frame +1

#main

frame=0
vis = o3d.visualization.Visualizer()
vis.create_window(
    window_name='Lidar Sensor',
    width=640,
    height=480,
    left=480,
    top=270)
vis.get_render_option().background_color = [0.05, 0.05, 0.05]
vis.get_render_option().point_size = 1
#vis.get_render_option().show_coordinate_frame = True
pcd=o3d.geometry.PointCloud()




worldsim_host="http://localhost:8080"
ego_name="ego"
sensor_name="PointCloudSensor"
scenario_filename="./scenarios/wolfsburg_sensors.json"

wss=ws_session_lib.worldsim_session(worldsim_host,fps=60)
wss.run_worldsim_distributed(sensor_node=True)
wss.reset()
wss.load_scenario_file(scenario_filename)
wss.scenario_play()

ws_sensor=ws_session_lib.worldsim_sensor(worldsim_host,ego_name,sensor_name)
endpoint, tcp, port = ws_sensor.get_sensor_endpoint_by_name()
print (sensor_name, endpoint, tcp, port)


context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt(zmq.CONFLATE, 1) # Get the latest message
socket.connect(endpoint)
socket.setsockopt_string(zmq.SUBSCRIBE, '')
socket.setsockopt(zmq.RCVTIMEO, 1000)

while True:
    try:
        buffer = socket.recv()
        if buffer is not None:
            # Do something with the message
            process_point_cloud_message(buffer)

    except zmq.ZMQError as e:
        if e.errno == zmq.EAGAIN:
            # this is not really an error, just no message was received
            pass
        else:
            print("ZMQ error: " + e.strerror)