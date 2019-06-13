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
roscore_port = 11311
bsn_dict = {}

def stop_execution(ip):
    pids = bsn_dict[ip]
    print(pids)
    # Send a Sigkill sign for all the processes pids
    for pid in pids:
        os.kill(pid, signal.SIGINT)
        os.wait()
    bsn_dict.pop(ip, None)
    print(bsn_dict)

def start_execution():
    global roscore_port
    processes_pids = []
    # Initialize odsupercomponent
    # configuration_path = os.getcwd()  + '/odv/sim/configs/' + path
    os.environ["ROS_MASTER_URI"] = "http://localhost:" + str(roscore_port)
    ros_process = subprocess.Popen(['roscore', "-p", str(roscore_port)], stdout=subprocess.PIPE, cwd=os.getcwd())
    print("ok")
    roscore_port += 1
    sleep(5)
    processes_pids.append(ros_process.pid)
    print("inicia a execucao")

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
    if bsn_dict == {}:
        return 'Inactive'

    response = ''
    for ips in bsn_dict: 
        for pid in bsn_dict[ips]:
            response += str(pid)  + " " + check_status(pid) + '<br>'
        response += "<br>"
        print(response)

    return response

# Start bsn execution
@app.route('/start', methods=['POST', 'GET'])
def start():
    ip = request.remote_addr
    print(ip)
    try:
        if ip in bsn_dict.keys():
            if len(bsn_dict[ip]) > 0: 
                return "error, multiple start"
        bsn_dict[ip] = start_execution()
        print(bsn_dict)
        return 'ok'
    except Exception as e:
        return 'execution error, exception: ' + str(e)

# Stop bsn execution
@app.route('/stop')
def stop():    
    ip = request.remote_addr
    try:
        if ip in bsn_dict.keys():
            stop_execution(ip)
            return 'ok'
        else:
            return "nothing to stop"
    except Exception as e:
        bsn_dict[ip] = []
        return 'error: ' + str(e)

@app.route('/')
def hello_world():
    return '<h1>Welcome to bsn</h1>'
    
app.run(host='0.0.0.0', debug=True)