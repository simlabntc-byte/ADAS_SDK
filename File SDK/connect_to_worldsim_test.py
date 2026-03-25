import ws_api.ws_session as ws_session_lib

worldsim_host="http://localhost:8080"
ego_name="ego"

wss=ws_session_lib.worldsim_session(worldsim_host)
wss.connect_to_worldim()

#scenario=wss.get_scenario()
#print (scenario)

config=wss.get_config()
print (config)

'''
dashboard=wss.get_dashboard_config()
print (dashboard)

dashboard_json={
  "Cluster": "http://www.google.com",
  "Infotainment": "https://www.google.com/maps/@42.6197113,-83.3193236,1146m/data=!3m1!1e3?entry=ttu&g_ep=EgoyMDI0MTEwNi4wIKXMDSoASAFQAw%3D%3D",
  "HUD": ""
}
wss.set_dashboard(dashboard_json)
'''

#dashboard=wss.get_dashboard_config()
#print (dashboard)

#tracklist=wss.get_available_tracks()
#print (tracklist)

#geojson=wss.get_paths_geojson("Wolfsburg")
#print (geojson)

#environment=wss.get_environment()
#print (environment)

#veh=wss.get_vehicles_in_scenario()
#print (veh)

#string=wss.get_vehicle_uuid_str('ego')
#print (string)

#string=wss.get_base_prj('MCity')
#print (string)