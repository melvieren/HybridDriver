import json
import sys
import os

EXE_PATH = "./x64/Release/TrackingWorker.exe"
CONFIG_DIR = "C:\\Program Files (x86)\\Steam\\config\\"

if len(sys.argv) > 1:
	if sys.argv[1] == '-h':
		print('usage: python install_overlay.py [EXE_PATH]')
	EXE_PATH = sys.argv[1]

vrmanifest = {
   "applications" : [
      {
         "app_key" : "hybridtracking.overlay",
         "autolaunch" : True,
         "binary_path_windows" : os.path.abspath(EXE_PATH),
         "is_dashboard_overlay" : True,
         "launch_type" : "binary",
         "strings" : {
            "en_us" : {
               "description" : "steamvr hybrid tracking ovelay",
               "name" : "Steamvr HybridTracking Worker"
            }
         }
      }
   ],
   "source" : "builtin"
}

vrappconfig =  { "autolaunch" : True, "last_launch_time" : "0" }
appconfig = None

with open(CONFIG_DIR + 'appconfig.json', 'r') as fp:
	appconfig = json.loads(fp.read())

if appconfig == None:
	print('Could not read appconfig.json.')
	exit(0)

with open(CONFIG_DIR + 'hybridtracking.vrmanifest', 'w+') as fp:
	fp.write(json.dumps(vrmanifest, sort_keys=True, indent=4))

with open(CONFIG_DIR + 'vrappconfig\\hybridtracking.vrappconfig', 'w+') as fp:
	fp.write(json.dumps(vrappconfig, sort_keys=True, indent=4))

new_path = CONFIG_DIR + 'hybridtracking.vrmanifest'
if not new_path in appconfig['manifest_paths']:
	appconfig['manifest_paths'].append(CONFIG_DIR + 'hybridtracking.vrmanifest')
	
with open(CONFIG_DIR + 'appconfig.json', 'w+') as fp:
	fp.write(json.dumps(appconfig, sort_keys=True, indent=4))

print('[+] Successfully installed overlay!')
