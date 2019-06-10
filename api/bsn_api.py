from flask import Flask, request, session, jsonify
import subprocess
import os
import json
import signal
from time import sleep

commands = [
    ['roslaunch', 'launch/centralhub.launch'],
    ['roslaunch', 'launch/thermometer.launch'],
    ['roslaunch', 'launch/oximeter.launch'],
    ['roslaunch', 'launch/ecg.launch'],
    ['roslaunch', 'launch/bloodpressure.launch']
]

app = Flask(__name__)
processes_pids = []

def stop_execution():
    global processes_pids
    # Send a Sigkill sign for all the processes pids
    for pid in processes_pids:
        os.kill(pid, signal.SIGINT)
    processes_pids = []        

def start_execution(path):
    global processes_pids

    # Initialize odsupercomponent
    # configuration_path = os.getcwd()  + '/odv/sim/configs/' + path
    od_process = subprocess.Popen(['roscore'], stdout=subprocess.PIPE, cwd=os.getcwd())
    sleep(1)
    processes_pids.append(od_process.pid)

    # Initialize sensors
    for command in commands:
        process = subprocess.Popen(command, stdout=subprocess.PIPE)
        processes_pids.append(process.pid)

    # Returns all the pids started
    return processes_pids

def check_status(pid):
    p = os.popen(' '.join(['ps','-p', str(pid), '-o', 'cmd'])).read()
    if ( p == "CMD\n" ):
        return('inexistent')

    else:        
        if 'defunct' in p:            
            return('zombie')
        else:
            return('active')

# Get all categories available
def get_configs():
    configuration_map = {}
    path = 'odv/sim/configs'
    # Get all folders from configs
    # Each one is a categorie
    categories = next(os.walk(path))[1]

    for categorie in categories:
        # Explore each subfolder to get all subcategories
        new_path = path + '/' + categorie
        configuration_map[categorie] = next(os.walk(new_path))[1]

    # Returns a map associating categorie and subcategories
    return configuration_map

# Check if requested path of configuration exists
# def is_configuration_available(path):
    # Check if there is a configuration in specified folder
    # path = os.getcwd() + '/odv/sim/configs/' + path + '/configuration'
    # return os.path.exists(path)

# Returns all configurations available
@app.route('/config', methods=['POST', 'GET'])
def config():
    configs_dict = request.get_json()
    for key in configs_dict:
        with open("launch/" + key + ".launch", 'w') as launchfile:
            launchfile.write(configs_dict[key])
    launchfile.close()
    return "ok"

# Show active processes
@app.route('/status')
def status():
    global processes_pids

    if processes_pids == []:
        return 'Inactive'

    response = ''
    for pid in processes_pids:
        response += str(pid)  + check_status(pid) + '<br>'

    return response

# Start bsn execution
@app.route('/start')
def start():
    global processes_pids
    path = request.args.get('path')    
    try:        
        processes_pids = start_execution(path)
        print(processes_pids)
        return 'ok'    
    except Exception as e:
        return 'execution error, exception: ' + str(e)

# Stop bsn execution
@app.route('/stop')
def stop():    
    global processes_pids
    try:
        stop_execution()
        return 'ok'
    except:
        processes_pids = []
        return 'error'

@app.route('/')
def hello_world():
    return '<h1>Welcome to bsn</h1>'
    
app.run(host='0.0.0.0', debug=True)